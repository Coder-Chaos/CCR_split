#include "tcp_pos.hpp"
#include <string.h>
namespace ccr_split {
    
    int Estimator::recv_imu()
    {
        char imu_buff[sizeof(IMU_DATA)];
        float ax;
        int nb = imu_server_.TcpRecvfromClient(imu_buff, sizeof(IMU_DATA));
        //memcpy(&imu, imu_buff, sizeof(IMU_DATA)); 
        memcpy(&imu.timestamp, imu_buff, sizeof(uint64_t)); 
        memcpy(&imu.accel, imu_buff + 8, sizeof(Eigen::Vector3f));
        memcpy(&imu.gyro, imu_buff + 8 + sizeof(Eigen::Vector3f), sizeof(Eigen::Vector3f));
        memcpy(&imu.accelValid, imu_buff + 32, 1);
        memcpy(&imu.gyroValid, imu_buff + 33, 1);
        
        //printf("len %dacc%d;%d: %f; %f; %f\n", sizeof(IMU_DATA), imu.accelValid, 
        //    imu.gyroValid, imu.accel(0), imu.accel(1), imu.accel(2));
		//printf("len %d，gyro%d;%d: %f; %f; %f\n", sizeof(IMU_DATA), imu.accelValid, 
        //    imu.gyroValid, imu.gyro(0), imu.gyro(1), imu.gyro(2));
        bzero(imu_buff,sizeof(IMU_DATA));
        return nb;

    }

    int Estimator::recv_rtk()
    {
        char rtk_buff[RTK_BUFFER_SIZE];
        int ret,i;
        //float fX,fY,fH;
        //char buf[BUF];
        //printf("recv_can start\n");
        int nb = rtk_server_.TcpRecvfromClient(rtk_buff, sizeof(rtk_buff));
        if(nb > 1)
        {  
            if (strstr(rtk_buff, "$GNGGA")!= NULL) //查找buf中是否有"$GPGGA"字符串，并将其在buf中的位置返回
            {
                if((i=strlen(strstr(rtk_buff, "$GNGGA"))) > GNGGA_SIZE)    //保证buf数组中有一个完整的 "$GNGGA,——"字符串
                {
                    ret=sscanf(rtk_buff,"$GNGGA,%*f,%f,%c,%f,%c,%d,%*d,%*f,%f", &rtk.fX, &rtk.cX, 
                        &rtk.fY, &rtk.cY, &rtk.nQ, &rtk.fH);
                    if(ret==6&&(rtk.nQ==1||rtk.nQ==2)){
                        printf("cX:fX=%c:%f\ncY:fY=%c:%f\nfH=%f\n",rtk.cX, rtk.fX, 
                        rtk.cY,rtk.fY,rtk.fH);//此处可将解析出的数据发送给其他进程
                    } else{
                        //printf("strlen= %d; rtk.nQ = %d\n", i, rtk.nQ);
                    }
                }
            }
            bzero(rtk_buff,RTK_BUFFER_SIZE);
        }
        return nb;
    }



    Vector<float, Estimator::n_x> Estimator::dynamics(
	    float t,
	    const Vector<float, Estimator::n_x> &x,
	    const Vector<float, Estimator::n_u> &u)
    {
    	return _A * x + _B * u;
    }


	void Estimator::initP()
	{
		_P.setZero();
		// initialize to twice valid condition
		
		_P(X_z, X_z) = 2 * EST_STDDEV_Z_VALID * EST_STDDEV_Z_VALID;
		// use vxy thresh for vz init as well
		_P(X_vz, X_vz) = 2 * _vz_pub_thresh * _vz_pub_thresh;
		// initialize bias uncertainty to small values to keep them stable
		_P(X_bz, X_bz) = 1e-6;
		_P(X_tz, X_tz) = 2 * EST_STDDEV_TZ_VALID * EST_STDDEV_TZ_VALID;
	}

	void Estimator::initSS()
	{
		initP();

		// dynamics matrix
		_A.setZero();
		// derivative of position is velocity
		_A(X_z, X_vz) = 1;

		// input matrix
		_B.setZero();
		_B(X_vz, U_az) = 1;

		// update components that depend on current state
		updateSSStates();
		updateSSParams();
	}

    void Estimator::updateSSStates()
    {
    	// derivative of velocity is accelerometer acceleration
    	// (in input matrix) - bias (in body frame)
    
    	_A(X_vz, X_bz) = -1;
    }

    void Estimator::updateSSParams()
    {
    	// input noise covariance matrix
    	_R.setZero();
    	_R(U_az, U_az) = _accel_z_stddev * _accel_z_stddev;

    	// process noise power matrix
    	_Q.setZero();
    	float pn_p_sq = _pn_p_noise_density * _pn_p_noise_density;
    	float pn_v_sq = _pn_v_noise_density * _pn_v_noise_density;
    	_Q(X_z, X_z) = pn_p_sq;
    	_Q(X_vz, X_vz) = pn_v_sq;

    	// technically, the noise is in the body frame,
    	// but the components are all the same, so
    	// ignoring for now
    	float pn_b_sq = _pn_b_noise_density * _pn_b_noise_density;
    	_Q(X_bz, X_bz) = pn_b_sq;

    	// terrain random walk noise ((m/s)/sqrt(hz)), scales with velocity
    	float pn_t_sq = _pn_t_noise_density * _pn_t_noise_density;
    	_Q(X_tz, X_tz) = pn_t_sq;
    }

    void Estimator::pos_predict(float dt)
    {
    	// get acceleration
    	
    	_u(U_az) = CONSTANTS_ONE_G*(cosf(_theta) -_accel(2));

    	// update state space based on new states
    	updateSSStates();

    	// continuous time kalman filter prediction
    	// integrate runge kutta 4th order
    	// TODO move rk4 algorithm to matrixlib
    	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
    	float h = dt;
    	Vector<float, n_x> k1, k2, k3, k4;
		//printf("old_x=[%6.2f, %6.2f, %6.2f, %6.2f]\n", _x(0), _x(1), _x(2), _x(3));
    	k1 = dynamics(0, _x, _u);
    	k2 = dynamics(h / 2, _x + k1 * h / 2, _u);
    	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);
    	k4 = dynamics(h, _x + k3 * h, _u);
    	Vector<float, n_x> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);
		//printf("dx=[%6.2f, %6.2f, %6.2f, %6.2f]\n", dx(0), dx(1), dx(2), dx(3));
		//printf("theta%6.2f, _accel%6.2f, _u%6.2f, K1%6.2f, %6.2f, %6.2f, %6.2f]\n", 
		//	_theta, _accel(0), _u(U_az), k1(0), k2(0), k3(0), k4(0));
		//printf("pos_x=[%f, %f, %f, %f]\n", _x(0), _x(1), _x(2), _x(3));
		//sleep(10);
        // don't integrate z if no valid z data
	    /*if (!(_estimatorInitialized & EST_Z))  {
	    	dx(X_z) = 0;
	    }

	    // don't integrate tz if no valid tz data
	    if (!(_estimatorInitialized & EST_TZ))  {
	    	dx(X_tz) = 0;
	    }else {
	    	dx(X_tz) = 0;
	    }*/

        // saturate bias
	    float bz = dx(X_bz) + _x(X_bz);

	    if (fabs(bz) > BIAS_MAX) {
	    	bz = BIAS_MAX * bz / fabs(bz);
	    	dx(X_bz) = bz - _x(X_bz);
	    }

	    // propagate
	    _x += dx;
	    Matrix<float, n_x, n_x> dP = (_A * _P + _P * _A.transpose() +
	    			      _B * _R * _B.transpose() + _Q) * h;

	    // covariance propagation logic
	    for (int i = 0; i < n_x; i++) {
	    	if (_P(i, i) > Ps_MAX) {
	    		// if diagonal element greater than max, stop propagating
	    		dP(i, i) = 0;
	    		for (int j = 0; j < n_x; j++) {
	    			dP(i, j) = 0;
	    			dP(j, i) = 0;
	    		}
	    	}
	    }

	    _P += dP;
		
		for(int i = 0; i<4; i++)
			_xf(i) = _filter.apply(_x(i));

		//printf("pos_filter=[%6.2f, %6.2f, %6.2f, %6.2f]\n", _x(0), _x(1), _x(2), _x(3));
		//_filter.apply(_x(X_tz) - _x(X_z));
	        
    }

	void Estimator::gpsInit1()
	{
		// check for good gps signal

		printf("[l29] gps init \n");
		// measure
		Vector<double, 3> y_org;
		Vector<double, 3> temp;
		Vector<double, n_y_gps> y;
		printf("[l34] gps init \n");
	}
    

    void Estimator::pos_update(){
        
        uint64_t last_time = 0;
		_x.setZero();
		int i;
		//printf("pos147_x=[%f, %f, %f, %f]\n", _x(0), _x(1), _x(2), _x(3));
        while(!_pos_should_exit){

        /* time from previous iteration */
		struct timeval tv;
        gettimeofday(&tv,NULL);
        uint64_t now = tv.tv_sec*1000000 + tv.tv_usec;
		const float val = (now  - last_time) / 1e6f;
        const float dt = (val < _dt_min) ? _dt_min : ((val > _dt_max) ? _dt_max : val);
		last_time = now;
		_timeStamp = now;
		if(!first_pos){
			_time_origin = _timeStamp;
			first_pos =true;
		}	
        //printf("dt=[%6.5f,%6.2f]\n", dt, _x(0));

        // update parameters
        // is xy valid?
        // is z valid?
		
		// do prediction
		pos_predict(dt);
		//printf("pos_x1=[%f, %f, %f, %f]\n", _x(0), _x(1), _x(2), _x(3));
		// sensor corrections/ initializations
		_gpsUpdated =true;
		//gpsInit();
		if (_gpsUpdated) {
			//printf("gps start=[%6.2f]\n", _x(0));
			if (_sensorTimeout & SENSOR_GPS) {
				//printf("gpsInit0\n");
				gpsInit();
				
			} else {
				//printf("gpsCorrect\n");
				gpsCorrect();
				
			}
		}
		// propagate delayed state, no matter what
		// if state is frozen, delayed state still
		// needs to be propagated with frozen state
		usleep(1000 * 20);
		float dt_hist = 1.0e-6f * (_timeStamp - _time_last_hist);

		if (_time_last_hist == 0 ||
		    (dt_hist > HIST_STEP)) {
			_x0 = _x;
			_time_last_hist = _timeStamp;
			//printf("acc=[%6.2f, %6.2f, %6.2f]\n", _accel(0),_accel(1), _accel(2));
			//printf("gyro=[%6.2f, %6.2f, %6.2f]\n", _gyro(0),_gyro(1), _gyro(2));
			//printf("att_theta%6.2f°;psi%6.2f°; pos_x=[%6.2fcm, %6.2fcm/s]\n", 
			//	_theta/PAI*180, _psi/PAI*180, _x(0)*100, _x(1)*100);
			
			//printf("pos_x=[%6.2f, %6.2f, %6.2f, %6.2f]\n", _x(0), _x(1), _x(2), _x(3));
		}
		_dt_now=1.0e-6f * (_timeStamp - _time_origin);
		//if(i%100 == 0)	
		//	printf("Time%6.4f; att_theta%6.2f°;psi%6.2f°; pos_x=[%6.2fcm, %6.2fcm/s]; acc=%6.3fm/s2\n", 
		//		_dt_now, _theta/PAI*180, _psi/PAI*180, _x(0)*100, _x(1)*100, _u(U_az));
		i ++;
		

        }
    }

    bool Estimator::att_run(){
        
        uint64_t last_time = 0;
        int i;
        
        while(!_att_should_exit){
			
			/* time from previous iteration */
			struct timeval tv;
        	gettimeofday(&tv,NULL);
        	uint64_t now = tv.tv_sec*1000000 + tv.tv_usec;
			const float val = (now  - last_time) / 1e6f;
        	const float dt = (val < _dt_min) ? _dt_min : ((val > _dt_max) ? _dt_max : val);
			last_time = now;
			if(!first_att){
				_time_att_origin = now;
				first_att =true;
			}
        	// Update sensors
        	//printf("imu_recv!\n");
			if (recv_imu() > 0) {
				// Feed validator with recent sensor data

				if (imu.gyroValid ) {
					/*_gyro(0) = imu.gyro(0);
					_gyro(1) = imu.gyro(1);
					_gyro(2) = imu.gyro(2);
					*/
					//转动
					_gyro(0) = imu.gyro(0);
					_gyro(1) = imu.gyro(2);
					_gyro(2) = imu.gyro(1);
				}

				if (imu.accelValid) {
					/*_accel(0) = imu.accel(0);
					_accel(1) = imu.accel(1);
					_accel(2) = imu.accel(2);
					*/
					_accel(0) = imu.accel(0);
					_accel(1) = imu.accel(2);
					_accel(2) = imu.accel(1);

					if (_accel.length() < 0.01f) {
						printf("WARNING: degenerate accel!");
						//continue;
					}
				}
				//printf("att_update!_gyro(1)%6.2f;_accel(0)%6.2f\n", _gyro(1), _accel(0));
				_data_good = true;
			}
        	//printf("imu_update!\n");

			if (att_update(dt)) {
				_theta = theta;
				_psi = psi;
        	    //if(i % 10 == 0)
				//printf("att_update!theta%6.2f°;psi%6.2f°\n", _theta/PAI*180, _psi/PAI*180);
				float dt_now=1.0e-6f * (now - _time_att_origin);
				
			}
			
        	i ++;
        }

    }

    bool Estimator::att_init(){
        theta = atan2f(sqrtf(_accel(0) * _accel(0) + _accel(1) * _accel(1)) , _accel(2));
        //att.y() = 0;
        psi = atan2f(_accel(1) , _accel(0));
        //att.normalize();
        _q = Eulerf(0.0f, theta, psi);
	    _q.normalize();

	    if (NX_ISFINITE(_q(0)) && NX_ISFINITE(_q(1)) &&
	        NX_ISFINITE(_q(2)) && NX_ISFINITE(_q(3)) &&
	        _q.length() > 0.95f && _q.length() < 1.05f) {
	    	att_inited = true;

	    } else {
	    	att_inited = false;
	    }

	    return att_inited;

    }
    bool Estimator::att_update(float dt){
        if (!att_inited) {

			if (!_data_good) {
				return false;
			}

			return att_init();
	    }

	    Quatf q_last = _q;

        float theta_am = 0.0f;
        float psi_am = 0.0f;
    
        theta -= _gyro(1) * dt;
        theta_am = atan2f(sqrtf(_accel(0) * _accel(0) + _accel(1) * _accel(1)) , _accel(2));
        theta = theta * _w_accel / (_w_accel + dt) + theta_am * dt / (_w_accel + dt);

        psi += _gyro(2) * dt;
        psi_am = atan2f(_accel(1) , _accel(0));
        psi = psi * _w_gyro_bias / (_w_gyro_bias + dt) + psi_am * dt / (_w_gyro_bias + dt);

		//printf("!theta_am%6.2f°;psi_am%6.2f°;theta%6.2f°;psi%6.2f°\n", theta_am/PAI*180, 
		//	psi_am/PAI*180,_theta/PAI*180, _psi/PAI*180);

        _q = Eulerf(0.0f, theta, psi);

		_theta = theta;
		_psi = psi;

	    // Normalize quaternion
	    _q.normalize();

	    if (!(NX_ISFINITE(_q(0)) && NX_ISFINITE(_q(1)) &&
	          NX_ISFINITE(_q(2)) && NX_ISFINITE(_q(3)))) {
	    	// Reset quaternion to last good state
	    	_q = q_last;
	    	_rates.zero();
	    	_gyro_bias.zero();
	    	return false;
	    }

	    return true;

    }

	/*int Estimator::getDelayPeriods(float delay, uint8_t *periods)
	{
		float t_delay = 0;
		uint8_t i_hist = 0;

		for (i_hist = 1; i_hist < HIST_LEN; i_hist++) {
			t_delay = 1.0e-6f * (_timeStamp - _tDelay.get(i_hist)(0, 0));

			if (t_delay > delay) {
				break;
			}
		}

		*periods = i_hist;

		if (t_delay > DELAY_MAX) {
			mavlink_and_console_log_info(&mavlink_log_pub, "%sdelayed data old: %8.4f", msg_label, double(t_delay));
			return -1;
		}

		return OK;
	}*/

    int Estimator::sign(double a){
        if(a > 0){  return 1;}
        else if(a = 0){ return 0;}
        else {  return -1;}
    }
 

}