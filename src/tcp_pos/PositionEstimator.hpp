#pragma once
// #include "common.hpp"
#include "net.hpp"
//#include <Eigen/Dense>
#include <matrix/Matrix.hpp>

#define IMU_BUFFER_SIZE 15
#define RTK_BUFFER_SIZE 512
#define GNGGA_SIZE 100

// required standard deviation of estimate for estimator to publish data
static const uint32_t		EST_STDDEV_XY_VALID = 2.0;	// 2.0 m
static const uint32_t		EST_STDDEV_Z_VALID = 2.0;	// 2.0 m
static const uint32_t		EST_STDDEV_TZ_VALID = 2.0;	// 2.0 m

//using namespace matrix;

namespace ccr_split {

        /* imu data */
    struct IMU_DATA
    {
        /*uint64_t timestamp;
        bool fusionPoseValid;
        RTVector3 fusionPose;
        bool fusionQPoseValid;
        RTQuaternion fusionQPose;
        bool gyroValid;
        RTVector3 gyro;
        bool accelValid;
        RTVector3 accel;
        bool compassValid;
        RTVector3 compass;
        bool pressureValid;
        RTFLOAT pressure;
        bool temperatureValid;
        RTFLOAT temperature;
        bool humidityValid;
        RTFLOAT humidity;*/

        float acc_x;
        float acc_y;
        float acc_z;

    };

    /* rtk data */
    struct RTK_DATA
    {
        float fX,fY,fH;             //Latitude; Longitude;Altitude
        int nQ;                     //GPS Quality
        char cX,cY;                 // N/S E/W
    } ;

class PositionEstimator {

public:

	// constants
	enum {X_x = 0, X_y, X_z, X_vx, X_vy, X_vz, X_bx, X_by, X_bz, X_tz, n_x};
	enum {U_ax = 0, U_ay, U_az, n_u};
	enum {Y_baro_z = 0, n_y_baro};
	enum {Y_lidar_z = 0, n_y_lidar};
	enum {Y_flow_vx = 0, Y_flow_vy, n_y_flow};
	enum {Y_sonar_z = 0, n_y_sonar};
	enum {Y_gps_x = 0, Y_gps_y, Y_gps_z, n_y_gps};
	enum {Y_vision_x = 0, Y_vision_y, Y_vision_z, n_y_vision};
	enum {Y_mocap_x = 0, Y_mocap_y, Y_mocap_z, n_y_mocap};
	enum {Y_land_vx = 0, Y_land_vy, Y_land_agl, n_y_land};
	enum {Y_target_x = 0, Y_target_y, n_y_target};
	enum {POLL_FLOW = 0, POLL_SENSORS, POLL_PARAM, n_poll};
	enum {
		FUSE_GPS = 1 << 0,
		FUSE_FLOW = 1 << 1,
		FUSE_VIS_POS = 1 << 2,
		FUSE_LAND_TARGET = 1 << 3,
		FUSE_LAND = 1 << 4,
		FUSE_PUB_AGL_Z = 1 << 5,
		FUSE_FLOW_GYRO_COMP = 1 << 6,
		FUSE_BARO = 1 << 7
	};

	enum sensor_t {
		SENSOR_BARO = 1 << 0,
		SENSOR_GPS = 1 << 1,
		SENSOR_LIDAR = 1 << 2,
		SENSOR_FLOW = 1 << 3,
		SENSOR_SONAR = 1 << 4,
		SENSOR_VISION = 1 << 5,
		SENSOR_MOCAP = 1 << 6,
		SENSOR_LAND = 1 << 7,
		SENSOR_LAND_TARGET = 1 << 8,
	};

	enum estimate_t {
		EST_XY = 1 << 0,
		EST_Z = 1 << 1,
		EST_TZ = 1 << 2,
	};

    PositionEstimator()
    {
      
      // initialize A, B,  P, x, u
	  _x.setZero();
	  _u.setZero();
	  initSS();

    }
    ~PositionEstimator() {}
    void update();

private:




    uint64_t _timeStamp;

    //int send_imu(struct can_frame &send_frame, common::Net &tcp_server);  
    int recv_imu(struct IMU_DATA &recv_frame, common::Net &imu_server);
    int recv_rtk(struct RTK_DATA &recv_frame, common::Net &rtk_server);
    void LonLat2UTM(double longitude, double latitude, double& UTME, double& UTMN)
    double GetPos(struct IMU_DATA &imu_frame, common::Net &imu_server, struct RTK_DATA &rtk_frame, common::Net &rtk_server);

    // state space
	Vector<float, n_x>  _x;	// state vector
	Vector<float, n_u>  _u;	// input vector
	Matrix<float, n_x, n_x>  _P;	// state covariance matrix

	matrix::Dcm<float> _R_att;
	Vector3f _eul;

	Matrix<float, n_x, n_x>  _A;	// dynamics matrix
	Matrix<float, n_x, n_u>  _B;	// input matrix
	Matrix<float, n_u, n_u>  _R;	// input covariance
	Matrix<float, n_x, n_x>  _Q;	// process noise covariance
}
}
