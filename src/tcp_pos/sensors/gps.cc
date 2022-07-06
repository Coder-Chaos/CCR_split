#include "../tcp_pos.hpp"
#include <matrix/math.hpp>


// required number of samples for sensor
// to initialize
static const uint32_t		REQ_GPS_INIT_COUNT = 5;
static const uint32_t		GPS_TIMEOUT = 1000000;	// 1.0 s

namespace ccr_split {

void Estimator::gpsInit()
{
	// check for good gps signal
	/*uint8_t nSat = _sub_gps.get().satellites_used;
	float eph = _sub_gps.get().eph;
	float epv = _sub_gps.get().epv;
	uint8_t fix_type = _sub_gps.get().fix_type;

	if (
		nSat < 6 ||
		eph > _gps_eph_max.get() ||
		epv > _gps_epv_max.get() ||
		fix_type < 3
	) {
		_gpsStats.reset();
		return;
	}*/

	// measure
	Vector<double, 3> y_org;
	Vector<double, 3> temp;
	Vector<double, n_y_gps> y;
	int i = 0;
	double gpsLat = 0;
	double gpsLon = 0;
	float gpsAlt = 0;
	while(1){
		//break;
		int i1 = gpsMeasure(y_org);
		if( i1 > 0)
		{	
			temp += y_org;
			i ++;
			if(i > 4)
				break;
			gpsLat = temp(0);
			gpsLon = temp(1);
			gpsAlt = temp(2);
			break;

			//printf("i=%d; Lat=%f;Lon=%f;temp2=%f;\n", i, temp(0), temp(1), temp(2));		
		} else{
			//printf("Error gpsMeasure%d\n", i1);
		}
	}
	/*double gpsLat = temp(0)/5;
	double gpsLon = temp(1)/5;
	float gpsAlt = temp(2)/5;
	*/
	//printf("gpsLat=%f; gpsLon=%f; gpsAlt=%f;\n", gpsLat, gpsLon, gpsAlt);
	_sensorTimeout &= ~SENSOR_GPS;
	_sensorFault &= ~SENSOR_GPS;
	//_gpsStats.reset();
	if (!_receivedGps) {
		// this is the first time we have received gps
		_receivedGps = true;
		// note we subtract X_z which is in down directon so it is
		// an addition
		_gpsAltOrigin = gpsAlt;
		// find lat, lon of current origin by subtracting x and y
		//gpsLon = 120.75819277;
    	//gpsLat = 31.34534646;
		LonLat2UTM(gpsLon, gpsLat, _UTME, _UTMN);
		//printf("UTME=%f; UTMN=%f;\n", _UTME, _UTMN);
		/*
		double lon = 120.75819277;
    	double lat = 31.34534646;
		double x;
    	double y;
    	LonLat2UTM(lon, lat, x, y);
		printf("UTME=%f; UTMN=%f;\n", x, y);
		*/
		// always override alt origin on first GPS to fix
		// possible baro offset in global altitude at init
		_altOrigin = _gpsAltOrigin;
		_altOriginInitialized = true;
		_altOriginGlobal = true;
		printf("[lpe] UTM origin init (gps) : UTME %6.2f UTMN %6.2f alt %5.1f m\n",
					     _UTME, _UTMN, double(_gpsAltOrigin));
		
		/*printf("[lpe] gps init "
			 "lat %6.2f lon %6.2f alt %5.1f m\n",
			 gpsLat,
			 gpsLon,
			 double(gpsAlt));
		*/
	}
	
}

int Estimator::Parse_GPS(char *data)
{
    int ret,temp;
    char* start, *tempStr;
    char *field;
	char longtitude[50]={0};
	char latitude[50]={0};
	char alt[50]={0};
	char nQ[50]={0};
   	int index = 0;
    start = strstr(data,"GNGGA");
 
    if (start)
    {
        tempStr = strstr(start,"E");
        if (tempStr == NULL)
        {
            //return -1;
        }
        //*(tempStr - 1) = 0;
 
        field = strtok(start,",");
 
        while (field)
        {
            index++;
            //printf("%s\n", field);
            field = strtok(NULL,",");
            if (index == 2)
            {
                strcpy(longtitude,field);
				rtk.fX = atof(longtitude);
				//printf("fX%f", rtk.fX);
            }
            if (index == 3)
            {
                strcpy(&rtk.cX, field);
				//printf("%c\n", rtk.cX);
            }
			if (index == 4)
            {
				strcpy(latitude,field);
				rtk.fY = atof(latitude);
				//printf("fY%f", rtk.fY);
            }
			if (index == 5)
            {
                strcpy(&rtk.cY, field);
				//printf("%c\n", rtk.cY);
            }
			if (index == 6)
            {
                strcpy(nQ, field);
				rtk.nQ = atof(nQ);;
				//printf("nQ%d", rtk.nQ);
            }
			/*if (index == 7)
            {
                strcpy(alt, field);
				rtk.fH = atof(alt);;
				printf("alt7=%lf", rtk.fH);
            }
			if (index == 8)
            {
                strcpy(alt, field);
				rtk.fH = atof(alt);;
				printf("alt8=%lf", rtk.fH);
            }
			if (index == 10)
            {
                strcpy(alt, field);
				rtk.fH = atof(alt);;
				printf("alt10=%lf", rtk.fH);
            }
			if (index == 11)
            {
                strcpy(alt, field);
				rtk.fH = atof(alt);;
				printf("alt11=%lf", rtk.fH);
            }*/
			if (index == 9)
            {
				strcpy(alt,field);
                rtk.fH = atof(alt);
				//printf("Alt=%lf\n", rtk.fH);
            }
        }
    }
 
    return 0;
}

int Estimator::gpsMeasure(Vector<double, 3> &y)
{
	// gps measurement
	y.setZero();
	char rtk_buff[RTK_BUFFER_SIZE];
    int ret,i;
	//double UTME, UTMN;
	int nb = rtk_server_.TcpRecvfromClient(rtk_buff, sizeof(rtk_buff));
	
    if(nb > 1){
		if (strstr(rtk_buff, "$GNGGA")!= NULL) //查找buf中是否有"$GPGGA"字符串，并将其在buf中的位置返回
        {
			i = strlen(strstr(rtk_buff, "$GNGGA"));
			if(i > GNGGA_SIZE)    //保证buf数组中有一个完整的 "$GNGGA,——"字符串
            {
				ret = Parse_GPS(rtk_buff);
                if(ret==0 && (rtk.nQ > 0) && (rtk.fX * rtk.fY * rtk.fH !=0)){
					y(0) = (int)(rtk.fX * 1e-2) +(rtk.fX * 1e-2 - (int)(rtk.fX * 1e-2))/0.6;
					y(1) = (int)(rtk.fY * 1e-2) +(rtk.fY * 1e-2 - (int)(rtk.fY * 1e-2))/0.6;
					y(2) = rtk.fH - 3.135;
					//printf("cX:fX=%c:%f; cY:fY=%c:%f; fH=%f\n", rtk.cX, rtk.fX, 
                    //   rtk.cY,rtk.fY,rtk.fH);//此处可将解析出的数据发送给其他进程
					printf("y(0)=%f; y(1)=%f; y(2)=%f\n", y(0), y(1), y(2));
					return 1;
                } else{
					//printf("Error:cX:fX=%c:%f; cY:fY=%c:%f; fH=%f; Q=%d\n", rtk.cX, rtk.fX, 
                    //    rtk.cY,rtk.fY,rtk.fH, rtk.nQ);
                    return -1;
                }
            }
		}
	}
	return -1;
	//return 0;
	//printf("y(0)=%f; y(1)=%f; y(2)=%f;\n", y(0), y(1), y(2));
	//usleep(1000*200);
	
}

void Estimator::gpsCorrect()
{
	// measure
	/*
	Vector<double, 3> y_global;
	Vector<double, n_y_gps> y_lobal;

	int i = gpsMeasure(y_global);
	if (i != 1) { return; }
	*/

	// gps measurement in local frame
	double  lat = y_global(0);
	double  lon = y_global(1);
	float  alt = y_global(2);
	//printf("gpsM%d:gpsLat=%f; gpsLon=%f; gpsAlt=%f;\n", i, lat, lon, alt);
	double px = 0;
	double py = 0;
	float pz = -(alt - _gpsAltOrigin);
	LonLat2UTM(lon, lat, px, py);
	Vector<float, n_y_gps> y;
	y.setZero();

	if(_theta < 0.78f){
		//// theta < 45°
		y(Y_gps_z) = pz/cosf(theta);
		printf("atl_y0=%4.3f", y(0));
	} else{
		// theta >= 45°
		int k = ((px-_UTME) > 0) ? 1 : (-1);
		y(Y_gps_z) = k * sqrt((px-_UTME)*(px-_UTME)+(py-_UTMN)*(py-_UTMN))/sinf(_theta);
		printf("UTM_y0=%4.3f;px=%lfUTME=%lf;py=%lfUTMN=%lf\n", y(0), px, _UTME, py, _UTMN);
	}
	

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
	printf("r=%4.3f;", r(0));
	// residual covariance
	// S = [_P(X_z, X_z)+var_z]
	Matrix<float, n_y_gps, n_y_gps> S = C * _P * C.transpose() + R;
	printf("S=%4.3f;", S(0, 0));
	// residual covariance, (inverse)
	Matrix<float, n_y_gps, n_y_gps> S_I = inv<float, n_y_gps>(S);
	printf("S_I=%4.3f;", S_I(0,0));
	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	// artifically increase beta threshhold to prevent fault during landing
	float beta_thresh = 1e2f;

	// kalman filter correction always for GPS 
	// K = [_P(X_z, X_z)/(_P(X_z, X_z)+var_z), 0, 0, 0]T
	Matrix<float, n_x, n_y_gps> K = _P * C.transpose() * S_I;
	printf("K=%4.3f;%4.3f;%4.3f;", K(0,0), K(0,1),K(0,2));
	Vector<float, n_x> dx = K * r;
	printf("dx=%4.3f;%4.3f;%4.3f\n", dx(0), dx(1), dx(2));
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
    double lon = longitude;
    double lat = latitude;
    // unit: km
    // variable
    double a = 6378.137;
    double e = 0.0818192;
    double k0 = 0.9996;
    double E0 = 500;
    double N0 = 0;
    //calc zoneNumber
    double zoneNumber = floor(lon/6) + 31;
    //calc lambda0
    double lambda0 = (zoneNumber - 1) * 6 - 180 + 3; //deg
    lambda0 = lambda0 * DEG_TO_RAD_LOCAL; //radian
    //calc phi and lambda (lat and lon)
    double phi = lat * DEG_TO_RAD_LOCAL;
    double lambda = lon * DEG_TO_RAD_LOCAL;

    // Formula START
    double v = 1 / sqrt(1 - pow(e*sin(phi), 2));
    double A = (lambda - lambda0) * cos(phi);
    double T = pow(tan(phi), 2);
    double C = pow(e, 2) / (1 - pow(e, 2)) * pow(cos(phi), 2);
    double s = (1 - pow(e, 2)/4 - 3*pow(e, 4)/64 - 5*pow(e, 6)/256)*phi - (3*pow(e, 2)/8 + 3*pow(e, 4)/32 + 45*pow(e, 6)/1024)*sin(2*phi) + (15*pow(e, 4)/256 + 45*pow(e, 6)/1024)*sin(4*phi) - 35*pow(e, 6)/3072*sin(6*phi);

    UTME = E0 + k0*a*v * (A + (1-T+C)*pow(A, 3)/6 + (5-18*T+T*T)*pow(A, 5)/120);
    UTMN = N0 + k0*a * (s + v*tan(phi) * (pow(A, 2)/2 + (5-T+9*C+4*C*C)*pow(A, 4)/24 + (61-58*T+T*T)*pow(A, 6)/720));

    UTME *= 1000;
    UTMN *= 1000;

	/*
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
	*/
}
}
