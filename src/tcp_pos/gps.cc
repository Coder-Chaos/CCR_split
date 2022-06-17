#include "tcp_pos.hpp"
#include <matrix/math.hpp>


// required number of samples for sensor
// to initialize
static const uint32_t		REQ_GPS_INIT_COUNT = 5;
static const uint32_t		GPS_TIMEOUT = 1000000;	// 1.0 s

namespace ccr_split {

/*void Estimator::gpsInit()
{
	// check for good gps signal
	
	printf("[l29] gps init ");
	// measure
	Vector<double, 3> y_org;
	Vector<double, 3> temp;
	Vector<double, n_y_gps> y;
	printf("[l34] gps init ");
	for(int i=0; i<5; i++){
		if(gpsMeasure(y_org))
		{	
			temp += y_org;
			i ++;
		}
	}
	double gpsLat = temp(0)/5;
	double gpsLon = temp(1)/5;
	float gpsAlt = temp(2)/5;
	printf("[l45] gps init ");
	_sensorTimeout &= ~SENSOR_GPS;
	_sensorFault &= ~SENSOR_GPS;
	//_gpsStats.reset();
	if (!_receivedGps) {
		// this is the first time we have received gps
		_receivedGps = true;
		// note we subtract X_z which is in down directon so it is
		// an addition
		_gpsAltOrigin = gpsAlt + _x(X_z)*cosf(theta);
		// find lat, lon of current origin by subtracting x and y
		double UTME = 0;
		double UTMN = 0;
		LonLat2UTM(gpsLat, gpsLon, UTME, UTMN);
		printf("[l59] gps init ");
		_time_origin = _timeStamp;
		// always override alt origin on first GPS to fix
		// possible baro offset in global altitude at init
		_altOrigin = _gpsAltOrigin;
		_altOriginInitialized = true;
		_altOriginGlobal = true;
		printf("[lpe] UTM origin init (gps) : lat %6.2f lon %6.2f alt %5.1f m",
					     UTME, UTMN, double(_gpsAltOrigin));
		
		printf("[lpe] gps init "
			 "lat %6.2f lon %6.2f alt %5.1f m",
			 gpsLat,
			 gpsLon,
			 double(gpsAlt));
	}
	
}*/

int Estimator::gpsMeasure(Vector<double, 3> &y)
{
	// gps measurement
	y.setZero();
	char rtk_buff[RTK_BUFFER_SIZE];
    int ret,i;
	//double UTME, UTMN;
	int nb = rtk_server_.TcpRecvfromClient(rtk_buff, sizeof(rtk_buff));
    if(nb > 1){
		if((i=strlen(strstr(rtk_buff, "$GNGGA"))) > GNGGA_SIZE)    //保证buf数组中有一个完整的 "$GNGGA,——"字符串
                {
                    ret=sscanf(rtk_buff,"$GNGGA,%*f,%f,%c,%f,%c,%d,%*d,%*f,%f", &rtk.fX, &rtk.cX, 
                        &rtk.fY, &rtk.cY, &rtk.nQ, &rtk.fH);
                    if(ret==6&&(rtk.nQ==1||rtk.nQ==2)){
                        printf("cX:fX=%c:%f\ncY:fY=%c:%f\nfH=%f\n",rtk.cX, rtk.fX, 
                        rtk.cY,rtk.fY,rtk.fH);//此处可将解析出的数据发送给其他进程
                    } else{
                        //return -1;
                    }
                }
	}

	//LonLat2UTM(rtk.fX, rtk.fY, UTME, UTMN);
	y(0) = rtk.fX * 1e-7;
	y(1) = rtk.fY * 1e-7;
	y(2) = rtk.fH * 1e-3;
	
	return 1;
}

void Estimator::gpsCorrect()
{
	// measure
	Vector<double, 3> y_global;
	Vector<double, n_y_gps> y_lobal;

	if (gpsMeasure(y_global) < 0) { return; }

	// gps measurement in local frame
	double  lat = y_global(0);
	double  lon = y_global(1);
	float  alt = y_global(2);
	double px = 0;
	double py = 0;
	float pz = -(alt - _gpsAltOrigin);
	LonLat2UTM(lat, lon, px, py);
	Vector<float, n_y_gps> y;
	y.setZero();
	y(Y_gps_z) = pz/cosf(theta);

	// gps measurement matrix, measures position and velocity
	// C = [1 0 0 0]
	Matrix<float, n_y_gps, n_x> C;
	C.setZero();
	C(Y_gps_z, X_z) = 1;

	// gps covariance matrix
	SquareMatrix<float, n_y_gps> R;
	R.setZero();

	// default to parameter, use gps cov if provided
	// R = [var_z]
	float var_z = _gps_z_stddev * _gps_z_stddev;
	R(0, 0) = var_z;

	// get delayed x
	uint8_t i_hist = 0;

	//if (getDelayPeriods(_gps_delay.get(), &i_hist)  < 0) { return; }

	//Vector<float, n_x> x0 = _xDelay.get(i_hist);

	// residual
	Vector<float, n_y_gps> r = y - C * _x0;

	// residual covariance
	// S = [_P(X_z, X_z)+var_z]
	Matrix<float, n_y_gps, n_y_gps> S = C * _P * C.transpose() + R;

	// residual covariance, (inverse)
	Matrix<float, n_y_gps, n_y_gps> S_I = inv<float, n_y_gps>(S);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	// artifically increase beta threshhold to prevent fault during landing
	float beta_thresh = 1e2f;

	// kalman filter correction always for GPS 
	// K = [_P(X_z, X_z)/(_P(X_z, X_z)+var_z), 0, 0, 0]T
	Matrix<float, n_x, n_y_gps> K = _P * C.transpose() * S_I;
	Vector<float, n_x> dx = K * r;
	_x += dx;
	_P -= K * C * _P;
}

/*
void Estimator::gpsCheckTimeout()
{
	if (_timeStamp - _time_last_gps > GPS_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_GPS)) {
			_sensorTimeout |= SENSOR_GPS;
			printf("[lpe] GPS timeout\n");
		}
	}
}
*/

void Estimator::LonLat2UTM(double longitude, double latitude, double& UTME, double& UTMN)
{
    
	double lat = latitude;
	double lon = longitude;

	double kD2R = PAI / 180.0;
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
}
