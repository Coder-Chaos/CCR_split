#pragma once
#include <ctime>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include "ccr_common.hpp"
#include "freemodbus_tcp.h"
#include "motion.hpp"
#include "net.hpp"
#include "shared_mem.h"
#include "udp_send.hpp"
#include "uvc.hpp"
#include "tfminip.hpp"
#include "tcp_pos.hpp"
#include "can.hpp"
#include "cuavpm.hpp"

#define DEBUG_ON 0
// modbus holding registor
extern USHORT usRegHoldingBuf[REG_HOLDING_NREGS];
namespace ccr_split
{
  

  struct RobotParam
  {
    // odometer
    int odometer;

    // ps core and pl core temperature
    unsigned short ps_temp;
    unsigned short pl_temp;

    // speed, mm/s
    short speed;

    // image saving state
    unsigned short img_saving_state;

    // heart beat
    // std::time_t sys_time_;
    // std::tm *tm_ = nullptr;

    // // uvc pic folder name
    // std::string uvc_folder_name_;
  };

  //Range
  struct Range
  {
    //id,type...
    unsigned short radiation_type;
    float field_of_view;
    float min_range;
    float max_range;
    float range;
    std::string frame_id;
    //std::time_t stamp;
    // std::tm *tm_ = nullptr;
  };
  //class Robot : public Motion
  class Robot 
  {

  private:
    /* data */

    double odom_last_time_ = 0;
    double odom_this_time_;
    // control period, ms
    static const int kControlPeriod = 100;
    //static const int kControlPeriod = 100;

    //
    double odometer_down = 0;
    double odometer_up = 0;

    cuav::PMhv *pmhv_obj;
    float pm_vlot_[2];
    float pm_curr_;

    benewake::TFmini *tfmini_obj;
    Range *Range_Up = (Range *)malloc(sizeof(Range));
    benewake::TFmini *tf03_obj;
    Range *Range_ = (Range *)malloc(sizeof(Range));

    Range *Range_Down = (Range *)malloc(sizeof(Range));
    static const USHORT ULTRASOUND = 0;
    static const USHORT INFRARED = 1;

    // can device
    can can0{(char *)"can0"};

    /* vision */
    // uvc camera
    uvc uvc_cam_{640, 480, "/dev/video0"};

    // udp client
    common::Net udp_client_{common::protocal_type::UDP, common::type::CLIENT,
                            "192.168.10.8", 7000};
    //common::Net udp_client_{,
    //                        "10.60.2.101", 7000};                        
    // AP Hotspot
    common::Net tcp_server_{common::protocal_type::TCP, common::type::SERVER,
                            "192.168.10.100", 4001};
    
    common::Net tcp_tf02_{common::protocal_type::TCP, common::type::SERVER,
                            "192.168.10.100", 9001};

    Estimator est_;

    /* motion */
    //   tmotor pointer
    std::vector<TMotor> tmotors = {
        {&can0, 1, (TMotorParam *)&usRegHoldingBuf[150]},
        {&can0, 2, (TMotorParam *)&usRegHoldingBuf[200]},
        {&can0, 3, (TMotorParam *)&usRegHoldingBuf[250]},
        {&can0, 4, (TMotorParam *)&usRegHoldingBuf[300]}
        };

    // maxon
    std::vector<maxon> maxons = {
        {&can0, &tcp_server_, 5, (maxon_type *)&usRegHoldingBuf[350]},
        {&can0, &tcp_server_, 6, (maxon_type *)&usRegHoldingBuf[400]},
        {&can0, &tcp_server_, 7, (maxon_type *)&usRegHoldingBuf[450]},
        {&can0, &tcp_server_, 8, (maxon_type *)&usRegHoldingBuf[500]},
        {&can0, &tcp_server_, 9, (maxon_type *)&usRegHoldingBuf[550]}
        };

    
    // motion
    Motion motion_ = {(MotionCmd *)&usRegHoldingBuf[0],
                      (MotionState *)&usRegHoldingBuf[100], &tmotors, 
                      &maxons};

    // shared memory for inter-process comunication
    char *shared_mem_ = nullptr;

    FILE * fp         = nullptr;
    

    // modbus thread
    std::thread modbus_thread_;

    // pm thread
    std::thread pmhv_thread_;

    // tfmini thread
    std::thread tf03_thread_;
    std::thread tf02_thread_;
    std::thread tfmini_thread_;

    // motion thread
    std::thread motion_thread_;

    // param thread
    std::thread param_thread_;
    std::thread param1_thread_;

    // uvc vision thread
    std::thread uvc_vision_thread_;

    //  guard thread
    std::thread guard_thread_;

    // save uvc thread
    std::thread save_uvc_pic_thread_;

    // get maxon parameter thread
    std::thread read_maxon_param_thread;

    // read att parameters thread
    std::thread att_thread_;
    //
    std::thread pos_thread_;

    std::thread log_thread_;

    // robot parameter
    RobotParam *param_ = nullptr;

    // message to vision process
    VisionMessage *vision_msg_ = nullptr;

    // heartbeat flag
    std::atomic_bool heart_beat_flag_;

    // control signal
    bool stop_uvc_signal_ = false;

    bool disconnect_host_signal_ = false;

    bool motion_stop_signal_ = false;

    bool stop_att_signal_ = false;
    bool stop_pos_signal_ = false;

    bool stop_param_signal_ = false;
    bool stop_param1_signal_ = false;

    bool stop_tfmini_signal_ =false;
    bool stop_tf03_signal_ =false;
    bool stop_tf02_signal_ =false;

    bool stop_pmhv_signal_ =false;
    bool stop_log_signal_ =false;

  public:
    // motion
    //Motion motion_ = {(MotionCmd *)&usRegHoldingBuf[0],
    //                  (MotionState *)&usRegHoldingBuf[100], &tmotors, 
    //                  &maxons};
    Robot()
    {
      // get shared memory
      shared_mem_ = (char *)get_shared_memory(
          kShareMemName.c_str(), kSemaphoreName.c_str(), kSharedMemLength);

      /* perform some init operation */
      param_ = (RobotParam *)&usRegHoldingBuf[50];
      int ret = 0;
      // init uvc camera
      ret = uvc_cam_.Init();
      if (!ret)
      {
        log_info("uvc camera init success.");
      }

      // point to shared memory
      vision_msg_ = (VisionMessage *)shared_mem_;

      /*IMU_DATA acc;
      RTK_DATA rtk;

      // read motor parameter thread
      read_imu_data_thread_ = std::thread([&]() { recv_imu(acc, imu_server_); });
      read_imu_data_thread_.detach();

      read_rtk_data_thread_ = std::thread([&]() { recv_rtk(rtk, rtk_server_);});
      read_rtk_data_thread_.detach();
      */

      }
    ~Robot() {}


    // set 
    int flag_pause;

    
    
    // robot safety guard
    void Guard();

    // connect host
    void ConnectHost();

    // disconnect host
    void DisConnectHost();

    //pm
    void PMhvRecv();
    void PMhvStop();

    //tfmini range
    void TF03Recv();
    void TF03Stop();
    void TFminiRecv();
    void TFminiStop();
    void TF02Recv();
    void TF02Stop();

    // read maxon parameter
    //void MaxonParamRead();
    //void MaxonParamReadStop();

    void StartVisionDetection() { vision_msg_->vision_system_control = 1; }
    void StopVisionDetection() { vision_msg_->vision_system_control = 0; }

    // start uvc
    void StartUVC();

    void StopUVC();

    // robot run
    void Run();

    // robot stop
    void Stop();

    // shut down robot
    void ShutDown();

    void Param();
    void ParamStop();
    void Param1();
    void Param1Stop();

    void Getatt();
    void GetattStop();

    void Getpos();
    void GetposStop();

    void Logdata();
    void LogdataStop();

  private:
    // get robot parameter
    void GetParam();

    // get odometer
    double GetOdometer() const;

    // get robot speed
    unsigned int GetSpeed();
  };
} // namespace ccr_split
