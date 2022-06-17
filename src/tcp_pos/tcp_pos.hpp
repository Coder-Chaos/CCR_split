#pragma once
#include "common.hpp"
#include "net.hpp"
#include <eigen3/Eigen/Dense>
#include <signal.h>
#include <math.h>
#include <iostream>
#include "matrix/math.hpp"
#include <sys/time.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

//#define IMU_BUFFER_SIZE 15
#define PARAM_OFFSET(Strct, Field)    ((unsigned long)&(((Strct *)0)->Field))
#define NX_ISFINITE(x) std::isfinite(x)
#define RTK_BUFFER_SIZE 512
#define GNGGA_SIZE 100

using namespace matrix;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::Vector;

static const float          BIAS_MAX = 1e-1f;
static const float          HIST_STEP = 0.2f;	// 20 hz
static const uint32_t		EST_STDDEV_Z_VALID = 2.0;	// 2.0 m
static const uint32_t		EST_STDDEV_TZ_VALID = 2.0;	// 2.0 m
static const float          Ps_MAX = 1.0e6f;	// max allowed value in state covariance
static const size_t         HIST_LEN = 10;	// DELAY_MAX / HIST_STEP;
static const double			PAI = 3.1415926535897932;
static const double			DEG_TO_RAD_LOCAL = 3.1415926535897932 / 180.0;

//using namespace matrix;

namespace ccr_split {

    static const float CONSTANTS_ONE_G = 9.80665f;						// m/s^2
    
    /* imu data */
    struct IMU_DATA
    {
        uint64_t timestamp;
        /*bool fusionPoseValid;
        Eigen::Vector3f fusionPose;
        bool fusionQPoseValid;
        Eigen::Vector4f fusionQPose;*/
        bool gyroValid;
        Vector3f gyro;
        bool accelValid;
        Vector3f accel;
        /*bool compassValid;
        Eigen::Vector3f compass;
        bool pressureValid;
        float pressure;
        bool temperatureValid;
        float temperature;
        bool humidityValid;
        float humidity;*/


    };

    /* rtk data */
    struct RTK_DATA
    {
        double fX,fY,fH;             //Latitude; Longitude;Altitude
        int nQ;                     //GPS Quality
        char cX,cY;                 // N/S E/W
    } ;

	class Estimator 
  {
	  friend class Robot;
    private:

    float		_w_accel = 0.5f;
	float		_w_mag = 0.0f;
	float		_w_ext_hdg = 0.0f;
	float		_w_gyro_bias = 0.5f;
	float		_mag_decl = 0.0f;
	bool		_mag_decl_auto = false;
	bool		_acc_comp = false;
	float		_bias_max = 0.0f;
	float		alpha = 0.0f;
	float		theta = 0.0f;
	
    float		phi = 0.0f;
    float       psi = 0.0f;

    const float _dt_min = 0.00001f;
	const float _dt_max = 0.02f;

    Vector3f	_gyro;
	Vector3f	_accel;

    Quatf		_q;
	Vector3f	_rates;
	Vector3f	_gyro_bias;

    bool		att_inited = false;
	bool		_data_good = false;
	bool		_ext_hdg_good = false;

    float       _accel_z_stddev = 0.02f;
    float       _gps_z_stddev = 0.1f;

    float       _pn_p_noise_density = 0.1f;
    float       _pn_v_noise_density = 0.1f;
    float       _pn_b_noise_density = 1e-3f;
    float       _pn_t_noise_density = 0.001f;
    float       _vz_pub_thresh = 0.3f;
    
    float       sample_freq = 800.0f;
    float       cutoff_freq = 30.0f;
    
    math::LowPassFilter2p	_filter{sample_freq, cutoff_freq};
    //math::LowPassFilter2p	_filter;
    common::Net imu_server_{common::protocal_type::TCP, common::type::SERVER,
                            "192.168.10.100", 6001};                        
    common::Net rtk_server_{common::protocal_type::TCP, common::type::SERVER,
                            "192.168.10.100", 8001};

    IMU_DATA imu;
    RTK_DATA rtk;

    // read imu parameters thread
    std::thread read_imu_data_thread_;
    // read rtk parameters thread
    std::thread read_rtk_data_thread_;

    //uint64_t _timeStamp;
    //bool first_imu = false;
	bool first_pos = false;
	bool first_att = false;

    public:
    // dynamics:
//
//	x(+) = A * x(-) + B * u(+)
//	y_i = C_i*x
//
// kalman filter
//
//	E[xx'] = P
//	E[uu'] = W
//	E[y_iy_i'] = R_i
//
//	prediction
//		x(+|-) = A*x(-|-) + B*u(+)
//		P(+|-) = A*P(-|-)*A' + B*W*B'
//
//	correction
//		x(+|+) =  x(+|-) + K_i * (y_i - H_i * x(+|-) )
//
//
// input:
//      ax, ay, az (acceleration Cable)
//
// states:
//      pz , ( position Cable, m)
//      vz ( vel Cable, m/s),
//      bz ( accel bias, m/s^2)
//      tz (terrain altitude, ASL, m)
//
// measurements:
//
//      sonar: pz (measured d*cos(phi)*cos(theta))
//
//      baro: pz
//
//      flow: vx, vy (flow is in body x, y frame)
//
//      gps: px, py, pz, vx, vy, vz (flow is in body x, y frame)
//
//      lidar: pz (actual measured d*cos(phi)*cos(theta))
//
//      vision: px, py, pz, vx, vy, vz
//
//      mocap: px, py, pz
//
//      land (detects when landed)): pz (always measures agl = 0)
//
public:

	// constants
	float		_theta = 0.0f;
	float		_psi = 0.0f;
	double 		_UTME = 0.0;
	double 		_UTMN = 0.0;

	enum {X_z = 0, X_vz, X_bz, X_tz, n_x};
	enum {U_az = 0, n_u};
	
	enum {Y_lidar_z = 0, Y_lidar_vz, n_y_lidar};
	
	enum {Y_sonar_z = 0, n_y_sonar};
	enum {Y_gps_z = 0, n_y_gps};
	enum {Y_vision_x = 0, Y_vision_y, Y_vision_z, n_y_vision};
	
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

    Estimator() : 
      // reference altitudes
	  _altOrigin(0),
	  _altOriginInitialized(false),
	  _altOriginGlobal(false),
	  _baroAltOrigin(0),
	  _gpsAltOrigin(0),
	  _LidarAltOrigin(0),
  
	  // status
	  _receivedGps(false),
	  _lastArmedState(false),
  
	  // masks
	  _sensorTimeout(UINT16_MAX),
	  _sensorFault(0),
	  _estimatorInitialized(0),
  
	  // sensor update flags
  
	  _gpsUpdated(false),
	  _lidarUpdated(false)
    {

      
      // read motor parameter thread
      read_imu_data_thread_ = std::thread([&]() { recv_imu(); });
      read_imu_data_thread_.detach();

      read_rtk_data_thread_ = std::thread([&]() { recv_rtk();});
      read_rtk_data_thread_.detach();
      
	  // assign distance subs to array  
	  // initialize A, B,  P, x, u
	  _x.setZero();
	  _u.setZero();
	  initSS();  
	  // print fusion settings to console

      }
    ~Estimator() {}

    Eigen::Vector3f att;
    
    Eigen::Vector3f pos;

    bool		_att_should_exit = false;		/**< if true, att should exit */
    bool		_pos_should_exit = false;

    //int send_imu(struct can_frame &send_frame, common::Net &tcp_server);  
    int recv_imu();
    int recv_rtk();
    void LonLat2UTM(double longitude, double latitude, double& UTME, double& UTMN);
    
    //void predict();
    Vector<float, n_x> dynamics(
		float t,
		const Vector<float, n_x> &x,
		const Vector<float, n_u> &u);
	void initP();
	void initSS();
    void updateSSStates();
    void updateSSParams();
    
    int sign(double a);
    void pos_predict(float dt);
    void pos_update();

    void gpsInit1();
	void gpsInit();
	int Parse_GPS(char *data);
    int gpsMeasure(Vector<double, 3> &y);
    void gpsCorrect();

    bool att_init();
    bool att_update(float dt);
    bool att_run();

    // misc
	uint64_t _time_att_origin;

	uint64_t _timeStamp;
	uint64_t _time_origin;
    uint64_t _time_last_hist;
	uint64_t _time_last_gps;

	float _dt_now;

    // reference altitudes
	float _altOrigin;
	bool _altOriginInitialized;
	bool _altOriginGlobal; // true when the altitude of the origin is defined wrt a global reference frame
	float _baroAltOrigin;
	float _gpsAltOrigin;
	float _LidarAltOrigin;


	// status
	bool _receivedGps;
	bool _lastArmedState;

	// masks
	uint16_t _sensorTimeout;
	uint16_t _sensorFault;
	uint8_t _estimatorInitialized;

	// sensor update flags
	//bool _flowUpdated;
	bool _gpsUpdated;
	//bool _visionUpdated;
	//bool _mocapUpdated;
	bool _lidarUpdated;
	//bool _sonarUpdated;
	//bool _landUpdated;
	//bool _baroUpdated;

    Vector<float, n_x>  _x0;	// state vector

	// state space
	Vector<float, n_x>  _x;	// state vector
	Vector<float, n_x>  _xf;	// state vector
	Vector<float, n_u>  _u;	// input vector
	Matrix<float, n_x, n_x>  _P;	// state covariance matrix

	matrix::Dcm<float> _R_att;
	Vector3f _eul;

	Matrix<float, n_x, n_x>  _A;	// dynamics matrix
	Matrix<float, n_x, n_u>  _B;	// input matrix
	Matrix<float, n_u, n_u>  _R;	// input covariance
	Matrix<float, n_x, n_x>  _Q;	// process noise covariance

  };
}
