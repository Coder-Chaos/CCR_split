#include "tcp_pos.hpp"
#include <string.h>
namespace ccr_split {
    /*int send_can(can_frame &send_frame, common::Net &tcp_server)
    {
        uint8_t can_buff[CAN_BUFFER_SIZE];
        can_buff[0] = send_frame.can_dlc;   
        can_buff[1] = (send_frame.can_id >> 24) & 0xff;
        can_buff[2] = (send_frame.can_id >> 16) & 0xff;
        can_buff[3] = (send_frame.can_id >> 8) & 0xff;
        can_buff[4] = send_frame.can_id & 0xff;
        for(int i = 0; i < 8; i ++){
            can_buff[i + 5] = send_frame.data[i];
        }
        //printf("can_id:0x%x!\n", send_frame.can_id);
        int nb = tcp_server.TcpSendtoClient(can_buff, CAN_BUFFER_SIZE);
        return nb;

    }*/

    int recv_imu(struct IMU_DATA &recv_frame, common::Net &imu_server)
    {
        char imu_buff[IMU_BUFFER_SIZE];
        //printf("recv_can start\n");
        int nb = imu_server.TcpRecvfromClient(imu_buff, sizeof(imu_buff));
        memcpy(&recv_frame.acc_x,imu_buff + 1, 4);
        memcpy(&recv_frame.acc_y,imu_buff + 6, 4);
        memcpy(&recv_frame.acc_z,imu_buff + 11, 4);
        //printf("acc:%f; %f; %f\n", recv_frame.acc_x, recv_frame.acc_y, recv_frame.acc_z);
        return nb;

    }

    int recv_rtk(struct RTK_DATA &recv_frame, common::Net &rtk_server)
    {
        char rtk_buff[RTK_BUFFER_SIZE];
        int ret,i;
        //float fX,fY,fH;
        //char buf[BUF];
        //printf("recv_can start\n");
        int nb = rtk_server.TcpRecvfromClient(rtk_buff, sizeof(rtk_buff));
        if(nb > 1)
        {  
            if (strstr(rtk_buff, "$GNGGA")!= NULL) //查找buf中是否有"$GPGGA"字符串，并将其在buf中的位置返回
            {
                if((i=strlen(strstr(rtk_buff, "$GNGGA"))) > GNGGA_SIZE)    //保证buf数组中有一个完整的 "$GNGGA,——"字符串
                {
                    ret=sscanf(rtk_buff,"$GNGGA,%*f,%f,%c,%f,%c,%d,%*d,%*f,%f", &recv_frame.fX, &recv_frame.cX, 
                        &recv_frame.fY, &recv_frame.cY, &recv_frame.nQ, &recv_frame.fH);
                    if(ret==6&&(recv_frame.nQ==1||recv_frame.nQ==2)){
                        printf("cX:fX=%c:%f\ncY:fY=%c:%f\nfH=%f\n",recv_frame.cX, recv_frame.fX, 
                        recv_frame.cY,recv_frame.fY,recv_frame.fH);//此处可将解析出的数据发送给其他进程
                    } else{
                        printf("strlen= %d; rtk.nQ = %d\n", i, recv_frame.nQ);
                    }
                }
            }
            bzero(rtk_buff,RTK_BUFFER_SIZE);
        }
        return nb;
    }

    void LonLat2UTM(double longitude, double latitude, double& UTME, double& UTMN)
    {
    	double lat = latitude;
    	double lon = longitude;
    
    	double kD2R = PI / 180.0;
    	double ZoneNumber = floor((lon - 1.5) / 3.0) + 1;
    	double L0 = ZoneNumber * 3.0;
    
    	double a = 6378137.0;
    	double F = 298.257223563;
    	double f = 1 / F;
    	double b = a * (1 - f);
    	double ee = (a * a - b * b) / (a * a);
    	double e2 = (a * a - b * b) / (b * b);
    	double n = (a - b) / (a + b); 
    	double n2 = (n * n); 
    	double n3 = (n2 * n); 
    	double n4 = (n2 * n2); 
    	double n5 = (n4 * n);
    	double al = (a + b) * (1 + n2 / 4 + n4 / 64) / 2.0;
    	double bt = -3 * n / 2 + 9 * n3 / 16 - 3 * n5 / 32.0;
    	double gm = 15 * n2 / 16 - 15 * n4 / 32;
    	double dt = -35 * n3 / 48 + 105 * n5 / 256;
    	double ep = 315 * n4 / 512;
    	double B = lat * kD2R;
    	double L = lon * kD2R;
    	L0 = L0 * kD2R;
    	double l = L - L0; 
    	double cl = (cos(B) * l); 
    	double cl2 = (cl * cl); 
    	double cl3 = (cl2 * cl); 
    	double cl4 = (cl2 * cl2); 
    	double cl5 = (cl4 * cl); 
    	double cl6 = (cl5 * cl); 
    	double cl7 = (cl6 * cl); 
    	double cl8 = (cl4 * cl4);
    	double lB = al * (B + bt * sin(2 * B) + gm * sin(4 * B) + dt * sin(6 * B) + ep * sin(8 * B));
    	double t = tan(B); 
    	double t2 = (t * t); 
    	double t4 = (t2 * t2); 
    	double t6 = (t4 * t2);
    	double Nn = a / sqrt(1 - ee * sin(B) * sin(B));
    	double yt = e2 * cos(B) * cos(B);
    	double N = lB;
    	N = N + t * Nn * cl2 / 2;
    	N = N + t * Nn * cl4 * (5 - t2 + 9 * yt + 4 * yt * yt) / 24;
    	N = N + t * Nn * cl6 * (61 - 58 * t2 + t4 + 270 * yt - 330 * t2 * yt) / 720;
    	N = N + t * Nn * cl8 * (1385 - 3111 * t2 + 543 * t4 - t6) / 40320;
    	double E = Nn * cl;
    	E = E + Nn * cl3 * (1 - t2 + yt) / 6;
    	E = E + Nn * cl5 * (5 - 18 * t2 + t4 + 14 * yt - 58 * t2 * yt) / 120;
    	E = E + Nn * cl7 * (61 - 479 * t2 + 179 * t4 - t6) / 5040;
    	E = E + 500000;
    	N = 0.9996 * N;
    	E = 0.9996 * (E - 500000.0) + 500000.0;
    
    	UTME = E;
    	UTMN = N;
    }

    void PositionEstimator::initP()
    {
    	_P.setZero();
    	// initialize to twice valid condition
    	_P(X_x, X_x) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
    	_P(X_y, X_y) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
    	_P(X_z, X_z) = 2 * EST_STDDEV_Z_VALID * EST_STDDEV_Z_VALID;
    	_P(X_vx, X_vx) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
    	_P(X_vy, X_vy) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
    	// use vxy thresh for vz init as well
    	_P(X_vz, X_vz) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
    	// initialize bias uncertainty to small values to keep them stable
    	_P(X_bx, X_bx) = 1e-6;
    	_P(X_by, X_by) = 1e-6;
    	_P(X_bz, X_bz) = 1e-6;
    	_P(X_tz, X_tz) = 2 * EST_STDDEV_TZ_VALID * EST_STDDEV_TZ_VALID;
    }

    void PositionEstimator::initSS()
    {
    	initP();

    	// dynamics matrix
    	_A.setZero();
    	// derivative of position is velocity
    	_A(X_x, X_vx) = 1;
    	_A(X_y, X_vy) = 1;
    	_A(X_z, X_vz) = 1;

    	// input matrix
    	_B.setZero();
    	_B(X_vx, U_ax) = 1;
    	_B(X_vy, U_ay) = 1;
    	_B(X_vz, U_az) = 1;

    	// update components that depend on current state
    	updateSSStates();
    	updateSSParams();
    }

    void PositionEstimator::updateSSStates()
    {
    	// derivative of velocity is accelerometer acceleration
    	// (in input matrix) - bias (in body frame)
    	_A(X_vx, X_bx) = -_R_att(0, 0);
    	_A(X_vx, X_by) = -_R_att(0, 1);
    	_A(X_vx, X_bz) = -_R_att(0, 2);

    	_A(X_vy, X_bx) = -_R_att(1, 0);
    	_A(X_vy, X_by) = -_R_att(1, 1);
    	_A(X_vy, X_bz) = -_R_att(1, 2);

    	_A(X_vz, X_bx) = -_R_att(2, 0);
    	_A(X_vz, X_by) = -_R_att(2, 1);
    	_A(X_vz, X_bz) = -_R_att(2, 2);
    }

    void PositionEstimator::updateSSParams()
    {
    	// input noise covariance matrix
    	_R.setZero();
    	_R(U_ax, U_ax) = _accel_xy_stddev.get() * _accel_xy_stddev.get();
    	_R(U_ay, U_ay) = _accel_xy_stddev.get() * _accel_xy_stddev.get();
    	_R(U_az, U_az) = _accel_z_stddev.get() * _accel_z_stddev.get();

    	// process noise power matrix
    	_Q.setZero();
    	float pn_p_sq = _pn_p_noise_density.get() * _pn_p_noise_density.get();
    	float pn_v_sq = _pn_v_noise_density.get() * _pn_v_noise_density.get();
    	_Q(X_x, X_x) = pn_p_sq;
    	_Q(X_y, X_y) = pn_p_sq;
    	_Q(X_z, X_z) = pn_p_sq;
    	_Q(X_vx, X_vx) = pn_v_sq;
    	_Q(X_vy, X_vy) = pn_v_sq;
    	_Q(X_vz, X_vz) = pn_v_sq;

    	// technically, the noise is in the body frame,
    	// but the components are all the same, so
    	// ignoring for now
    	float pn_b_sq = _pn_b_noise_density.get() * _pn_b_noise_density.get();
    	_Q(X_bx, X_bx) = pn_b_sq;
    	_Q(X_by, X_by) = pn_b_sq;
    	_Q(X_bz, X_bz) = pn_b_sq;

    	// terrain random walk noise ((m/s)/sqrt(hz)), scales with velocity
    	float pn_t_noise_density =
    		_pn_t_noise_density.get() +
    		(_t_max_grade.get() / 100.0f) * sqrtf(_x(X_vx) * _x(X_vx) + _x(X_vy) * _x(X_vy));
    	_Q(X_tz, X_tz) = pn_t_noise_density * pn_t_noise_density;
    }

    void PositionEstimator::update(){
        ;
        ;
    }

    double GetPos() 
    {
        double pos, pos_i, pos_r, UTME, UTMN, UTME_old, UTMN_old, fH_old;

        // wait for a sensor update, check for exit condition every 100 ms
	    if (recv_imu(imu_frame, imu_server) < 0 && recv_rtk(rtk_frame, rtk_server) < 0) {
	    	return;
	    }

        uint64_t newTimeStamp = hrt_absolute_time();
	    float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	    _timeStamp = newTimeStamp;
        
        // update parameters
        // is xy valid?
        // is z valid?


        
        if((recv_imu(imu_frame, imu_server) > 0) && (recv_rtk(rtk_frame, rtk_server)) > 0){
            LonLat2UTM(rtk_frame.fX, rtk_frame.fY, UTME, UTMN);
            pos_r = sign(UTME - UTME_old)*sqrt((UTME_old - UTME)^2 + (UTMN_old - UTMN)^2 + 
                (rtk_frame.fH - fH_old)^2);
            pos_i = ;
        } else{
            printf("IMU or RTK error\n");
        }
        UTME_old = UTME;
        UTMN_old = UTMN;
        fH_old = rtk_frame.fH;
        //return motion_.odometer_; 
    }

}