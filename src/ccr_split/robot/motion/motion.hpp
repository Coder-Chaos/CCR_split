#pragma once
#include "ccr_common.hpp"
#include "common.hpp"
#include "freemodbus_tcp.h"
#include "tmotor.hpp"
#include "maxon.hpp"
#include <vector>
#include <atomic>

#define LEAK_ROB 0

namespace ccr_split {
  // total motor number
  static const int kTMotorNum = 4;

  // wheel perimeter 370mm
  static const USHORT kWheelPerimeter = 370;

  /* robot state */
  struct MotionState {
    //   motion mode
    MotionModeState motion_mode_state;

    // //   motion
    USHORT current_cmd;

    // //   FSM state
    USHORT current_state;
  };

  class Motion {
    friend class Robot;

  private:
    int test_dir_ = 0;
    int test_pm_  = 0;
    int test_p_  = 0;
    int test_m_  = 0;
    int loop_num_ = 0;
 
    // host command
    MotionCmd* motion_cmd_ = nullptr;

    //   motion state
    MotionState* motion_state_ = nullptr;

    //  tmotors
    std::vector<TMotor>* tmotors_ = nullptr;

    //  maxon
    std::vector<maxon>* maxons_ = nullptr;

    // moving odometer
    double odometer_;
    double odometer_0;

    double odometer_p1  = 0;
    double odometer_p2 = 0;
    /* motor parameters */
    // kd, 0-5
    static constexpr float Kd = 5;

    // kp, 0-500
    static constexpr float Kp = 5;

    // read motor parameters thread
    std::thread read_tmotors_param_thread_;

    // read maxon parameters thread
    std::thread read_maxon_param_thread_;
    std::thread read_maxon_param1_thread_;

  public:
    // host command

    Motion(MotionCmd* motion_cmd, MotionState* motion_state,
      std::vector<TMotor>* tmotors, std::vector<maxon>* maxons)
      : motion_cmd_(motion_cmd), motion_state_(motion_state),
      tmotors_(tmotors), maxons_(maxons) {
      // init motor
      for (auto tmotor : *tmotors_) {
        tmotor.Init();
      }
      for (auto maxon : *maxons_) {
        maxon.Init();
      }

      // read motor parameter thread
      read_tmotors_param_thread_ = std::thread([&]() { GetTMotorsParam(); });
      read_tmotors_param_thread_.detach();
      
      // read maxon parameter thread
      read_maxon_param_thread_ = std::thread([&]() { GetmaxonsParam(); });
      read_maxon_param_thread_.detach();

      read_maxon_param1_thread_ = std::thread([&]() { GetmaxonsParam1(); });
      read_maxon_param1_thread_.detach();
      
      // set default param
      motion_cmd_->sample_interval = kDefaultSampleInterval;
      motion_cmd_->speed_gear = kNormal;
      //motion_cmd_->speed_gear = kNormal;
    }

    Motion(/* args */) {}
    ~Motion() {
      // exit motor mode
      for (auto tmotor : *tmotors_) {
        tmotor.ExitMotorMode();
      }
    }

    // moving odometer
    double odom_Up;
    double odom_Down;
    double odom_Mid;
    double odom_Up0;
    double odom_Down0;
    double odom_Mid0;

    bool flag_first_test = false;
    bool flag_first_return =false;
    bool flag_first_auto = false;

    // set 
    int flag_pause;
    int flag_Pup;
    int flag_Pdown;
    int flag_Mup;
    int flag_MDown;
    
    /* Motor node id List */
    static const __u8 kUpClaw = 5;
    static const __u8 kDownClaw1 = 6;
    static const __u8 kDownClaw2 = 7;
    static const __u8 kPulley1 = 8;
    static const __u8 kPulley2 = 9;

    // maxon motion state
    static const __u8 kIdle = 0;
    static const __u8 kHold = 1;
    static const __u8 kLoose = 2;
    static const __u8 kTighten = 3;
    static const __u8 kPullingUp = 4;
    static const __u8 kPullingDown = 5;
    static const __u8 kStop = 6;

    /* -------------------debug parameters------------------------------------ */
    /* upclaw debug parameters */
    // upclaw hold torque, 55%
    static const __s16 kUpClawHoldTorque = 750;
    // static const __s16 kUpClawHoldTorque = 600;

    // upclaw loose distance
    static const __s32 kUpClawLooseDistance = -30 * 10000;

    /* downclaw debug parameters */
    // down hold torque, 40%
    static const __u16 kDownClawHoldTorque = 600;
    //static ; 

    // loose distance
    static const __s32 kDownClawLooseDistance = -40 * 10000;
    // ;

    // pulleys para
    static const __s16 kPulleysHoldTorque = -50; //
    static const __s32 kPulleysLooseSpeed = 54000;   //GeraBox()*180/pi*59/35*6/0.01 52000  Vel7.5上54000
    static const __s32 kPulleysTightenSpeed = -56000; // 35mm-57950;40mm-50706 Vel          Vel6.0上56000
    static const __s32 kPulleysUpSpeed = -50000 * 0.75f; //0.8-OK
    static const __s32 kPulleysDownSpeed = 50000 * 0.75f; //0.8-OK

    // homing torque
    static const __u32 kPulleysHomingTorque = 400;

    // delay_time wait for epos 1000us
    static const __u32 kDelayEpos = 1000; 

    // System motion FSM
    void SystemMotionFSM();
    void SystemMotionFSM(const std::atomic_bool &heartbeat_flag);

    // Manual motion FSM
    void ManualMotonFSM();

    // return motion
    int Return();

    // motors reset
    void MotorReset();

    //Claw Hold/Loose function
	  void UpClawHold();
	  void UpClawLoose();
	  void DownClawHold();
	  void DownClawLoose();

    //Pulleys function
    void PulleysLoose();
    void PulleysTighten();
    void PulleysTorque();
    void PulleysStop();
    void PulleysPullingup();
    void PulleysPullingdown();

    // move up function
    void MoveUp();
    void PMoveUp();
    void MMoveUp();
    void TMoveUp();

    // move down function
    void MoveDown();
    void PMoveDown();
    void MMoveDown();
    void TMoveDown();

    // Stop function
    void Stop();
    void PStop();
    void MStop();
    void TStop();

    void Idle();

    // get motor parameter
    void GetTMotorsParam();
    
    void GetmaxonsParam();
    void GetmaxonsParam1();

    //  speed gear
    float GearBox();

    // check heartbeat
    bool IsHostAlive();

  };

} // namespace ccr_split
