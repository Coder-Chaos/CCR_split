#include "motion.hpp"
#include "freemodbus_tcp.h"
#include "log.h"
#include "tmotor.hpp"
#include "maxon.hpp"
#include <cmath>
#include <thread>
#include <chrono>
#include <iostream>

using namespace std;
#define MIMIC_MOTION 0


namespace ccr_split
{

  void Motion::ManualMotonFSM()
  {
    //if(motion_cmd_->command == 0){motion_cmd_->command = 1;}
    // select mannual motion command
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
    cout << "0/1/2/13 for Robot Stop, MoveUp, MoveDown, reset; "<< endl;
    cout << "3/4/5 for Precursor Stop, MoveUp, MoveDown;"<< endl;
    cout << "6/7/8 for MainFrame Stop, MoveUp, MoveDown;"<< endl;
    cout << "9/10/11/12 for UpClawHold, UpClawLoose, DownClawHold, DownClawLoose" << endl;

    int flag_command;
    cout << "Please input command: ";
    cin >> flag_command;
    //printf("cin num%c finished!\n", flag_command);
    for(int i = 0; i < 20; i++){
      if(i == flag_command){
        motion_cmd_->command = i;
      }
    }
    flag_pause = flag_command;
    printf("motion_cmd_->command=%d!\n", motion_cmd_->command);
    // select mannual motion command
    switch (motion_cmd_->command)
    {
    case kMoveUpCmd:
#if MIMIC_MOTION
      log_debug("move up.");
      log_debug("gearbox:%f", GearBox());
#else
      MoveUp();
#endif
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kMoveUpState;
      //motion_cmd_->command = 2;
      break;

    case kMoveDownCmd:
#if MIMIC_MOTION
      log_debug("move down.");
      log_debug("gearbox:%f", GearBox());
#else
      MoveDown();
#endif
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kMoveDownState;
      //motion_cmd_->command = 1;
      break;

    case kStopCmd:
      if (motion_state_->current_state != kStopState){
        Stop();
      } else{
        printf("kStop finished!\n");
      }

      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kStopState;
      break;

    case pMoveUpCmd:
      if (motion_state_->current_state != pMoveUpState){
        PMoveUp();
      } else{
        printf("pMoveUp finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = pMoveUpState;
      break;

    case pMoveDownCmd:
      if (motion_state_->current_state != pMoveDownState){
        PMoveDown();
      } else{
        printf("pMoveDown finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = pMoveDownState;
      break;

    case pStopCmd:
      if (motion_state_->current_state != pStopState){
        PStop();
      } else{
        printf("pStop finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = pStopState;
      break;

    case mMoveUpCmd:
      if (motion_state_->current_state != mMoveUpState){
        MMoveUp();
      } else{
        printf("mMoveUp finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = mMoveUpState;
      break;

    case mMoveDownCmd:
      if (motion_state_->current_state != mMoveDownState){
        MMoveDown();
      } else{
        printf("mMoveDown finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = mMoveDownState;
      break;

    case mStopCmd:
      if (motion_state_->current_state != mStopState){
        MStop();
      } else{
        printf("mStop finished!\n");
      } 
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = mStopState;
      break;

    case kUpClawHoldCmd:
      if (motion_state_->current_state != kUpClawHoldState){
        UpClawHold();
      } else{
        printf("kUpClawHold finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kUpClawHoldState;
      break;

    case kUpClawLooseCmd:
      if (motion_state_->current_state != kUpClawLooseState){
        UpClawLoose();
      } else{
        printf("kUpClawLoose finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kUpClawLooseState;
      break;      

    case kDownClawHoldCmd:
      if (motion_state_->current_state != kDownClawHoldState){
        DownClawHold();
      } else{
        printf("kDownClawHold finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kDownClawHoldState;
      break;

    case kDownClawLooseCmd:
      if (motion_state_->current_state != kDownClawLooseState){
        DownClawLoose();
      } else{
        printf("kDownClawLoose finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kDownClawLooseState;
      break;

    case kMotorResetCmd:
      MotorReset();
      printf("kMotorReset finished!\n");
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kMotorResetState;
      break;

    // defualt stop
    default:
#if MIMIC_MOTION
      log_debug("stop state.");
      log_debug("gearbox:%f", GearBox());
#else
      Stop();
#endif
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kStopState;
      break;
    }
  }

  int Motion::Return()
  {
    int ret;

    // odometer > 100mm, move back
    if (odometer_ > 100)
    {
      log_debug("odom %f mm", odometer_);
      MoveDown();
      ret = -1;
    }
    else
    {
      log_debug("stop!");
      Stop();
      ret = 0;
    }
    return ret;
  }
  static int stop_cnt = 0;
  /* motion FSM */
  void Motion::SystemMotionFSM(const std::atomic_bool &heartbeat_flag)
  {
    maxons_->begin()->NMTstart();
    // check heartbeat
    if (heartbeat_flag == false)
    {
      if (stop_cnt == 0)
      {
        stop_cnt++;
        Stop();
        sleep(1);
      }
      motion_cmd_->motion_mode = kReturnMode;
    }

    // check quick stop command
    if (motion_cmd_->quick_stop != kQuickStopOn)
    {
      
      //motion_cmd_->motion_mode = 4;
      //   select motion mode
      switch (motion_cmd_->motion_mode)
      {
        //     mannual mode
      case kManualMode:
        // update state
        //log_info("ManualMode：odometer_ = %f mm", odometer_);
        motion_state_->motion_mode_state = MotionModeState::ManualMode;
        ManualMotonFSM();
        break;

        // auto mode
      case kAutoMode:
        motion_state_->motion_mode_state = MotionModeState::AutoMode;
        log_debug("auto mode.");
        MoveUp();
        MoveDown();
        break;

        // return mode
      case kReturnMode:
        int ret;
        log_debug("return mode.");
        if (motion_state_->motion_mode_state != MotionModeState::ReturnMode)
        {
          do
          {
            // check if quick stop
            if (motion_cmd_->quick_stop == kQuickStopOn)
            {
              //   quick stop
              Stop();
              break;
            }
            ret = Return();
            // usleep(200000);
            std::this_thread::sleep_for(std::chrono::microseconds(200));
          } while (ret != 0);
          motion_state_->motion_mode_state = MotionModeState::ReturnMode;
        }
        // change to mannual mode
        motion_cmd_->motion_mode = kManualMode;
        break;

        // set zero
      case kSetZero:
        if (motion_state_->motion_mode_state != MotionModeState::SetZero)
        {
          // clear motor parameters
          for (auto tmotor : *tmotors_)
          {
            odometer_ = 0;
            tmotor.ClearParam();
          }
          motion_state_->motion_mode_state = MotionModeState::SetZero;
        }
        // change to mannual mode
        log_debug("finish set zero.");
        motion_cmd_->motion_mode = kManualMode;
        break;

        // test
      case kTestMode:
        motion_state_->motion_mode_state = MotionModeState::TestMode;
        log_info("odometer_ = %f mm.", odometer_);
        if (test_dir_ == 0)
        {
          // move up 1000mm
          if (odometer_ < 2000)
          {
            MoveUp();
          }
          else
          {
            // change to move down
            test_dir_ = -1;
            // stop motor
            Stop();
            sleep(1);
          }
        }
        else if (test_dir_ == -1)
        {
          // move up 1000mm
          if (odometer_ > 600)
          {
            MoveDown();
          }
          else
          {
            // change to move up
            test_dir_ = 0;
            // stop motor
            Stop();
            sleep(1);
          }
        }
        break;

        // default stop
      default:
        Stop();
        break;
      }
    }
    else
    {
      //   quick stop
      Stop();
      motion_cmd_->motion_mode = kManualMode;
      motion_cmd_->quick_stop = kQuickStopOff;
    }
  }

  void Motion::MotorReset()
  {
    for(int i = 0; i< 5; i++)
    {
      maxons_->at(i).MotorReset();
      maxons_->at(i).param_->motion_state = kIdle;
      usleep(kDelayEpos);
    }
    printf("motors reset done!\n");
  }

  //UpClaw Hold
	void Motion::UpClawHold()
  {
    //if((maxons_->at(0).param_->StatusWord & 0x00ff) != 0x37){
      maxons_->at(0).MotorEnable();
    //}
    maxons_->at(0).SetTargetTorque(kUpClawHoldTorque);
	  // wait hold complete
	  while (abs(maxons_->at(0).param_->actual_average_vel) > 20) {
	  	usleep(10);
      maxons_->at(0).CanDisPatch1();
	  } 
	  printf("upclaw hold done!\n");
	  // change upclaw1 state to hold
	  maxons_->at(0).param_->motion_state = kHold;
  }
  //UpClaw Loose
	void Motion::UpClawLoose()
  {
    //if(maxons_->at(0).param_->StatusWord != 0x0240){
      maxons_->at(0).MotorDisable();
    //}
    maxons_->at(0).MotorEnable();
    maxons_->at(0).SetMotorAbsPos(maxons_->at(0).param_->PosPV 
      + kUpClawLooseDistance);
    // change upclaw1 state to loose
	  maxons_->at(0).param_->motion_state = kLoose;
    printf("UpClaw loose Done!\n");
  }
  //DownClaw Hold
	void Motion::DownClawHold()
  {
    int i = 0;
    //if((maxons_->at(1).param_->StatusWord & 0x00ff) != 0x37){
      maxons_->at(1).MotorEnable();
    //}
    //if((maxons_->at(2).param_->StatusWord & 0x00ff) != 0x37){
      maxons_->at(2).MotorEnable();
    //}    
    maxons_->at(1).SetTargetTorque(maxons_->at(1).param_, 
    maxons_->at(2).param_, kDownClawHoldTorque, kDownClawHoldTorque);
    
    printf("DownClawHold Done.\n");
    maxons_->at(1).param_->motion_state = kHold;
	  maxons_->at(2).param_->motion_state = kHold;
  }
  //DownClaw Loose
	void Motion::DownClawLoose()
  {
    __s32 pos1 = kDownClawLooseDistance;
    __s32 pos2 = kDownClawLooseDistance;
    //if(maxons_->at(1).param_->StatusWord != 0x0240){
      maxons_->at(1).MotorDisable();
    //}
    //if(maxons_->at(2).param_->StatusWord != 0x0240){
      maxons_->at(2).MotorDisable();;
    //}
    maxons_->at(1).MotorEnable();
    maxons_->at(2).MotorEnable();
	  
	  maxons_->at(1).SetMotorAbsPos(maxons_->at(1).param_, maxons_->at(2).param_, 
      maxons_->at(1).param_->PosPV + pos1, maxons_->at(2).param_->PosPV + pos2);

	  printf("down claw loose Done!\n");
	  // change downclaw1 state to
	  maxons_->at(1).param_->motion_state = kLoose;
	  maxons_->at(2).param_->motion_state = kLoose;    
  }
  // Tmotors move up
  void Motion::TMoveUp()
  {
    while(1){
    // Tmotors move up
    for (int i = 0; i < 2; i++)
    {
#if LEAK_ROB
      tmotors_->at(2*i).param_->dir = 1;
#else
      tmotors_->at(2*i).param_->dir = -1;
#endif
      tmotors_->at(2*i).param_->tmtSetVel = GearBox();
      tmotors_->at(2*i).param_->tmtSetKD = Kd;
      tmotors_->at(2*i).SetVel(tmotors_->at(2*i).param_->dir, tmotors_->at(2*i).param_->tmtSetVel,
                                 tmotors_->at(2*i).param_->tmtSetKD);
      tmotors_->at(2*i + 1).param_->dir = -1;
      tmotors_->at(2*i + 1).param_->tmtSetVel = GearBox();
      tmotors_->at(2*i + 1).param_->tmtSetKD = Kd;
      tmotors_->at(2*i + 1).SetVel(tmotors_->at(2*i + 1).param_->dir, 
        tmotors_->at(2*i + 1).param_->tmtSetVel, tmotors_->at(2*i + 1).param_->tmtSetKD);
      /*printf("Tmotors%d Vel_sp/Vel=%f, %f!\n", 2*i+1, tmotors_->at(2*i).param_->tmtSetVel, 
      tmotors_->at(2*i).param_->tmtGetVel);
      printf("Tmotors%d Vel_sp/Vel=%f, %f!\n", 2*i+2, tmotors_->at(2*i+1).param_->tmtSetVel, 
      tmotors_->at(2*i+1).param_->tmtGetVel);*/
    }
    if(fabs(fabs(tmotors_->at(0).param_->tmtGetVel) - fabs(tmotors_->at(0).param_->tmtSetVel)) < 0.1f && 
      fabs(fabs(tmotors_->at(1).param_->tmtGetVel) - fabs(tmotors_->at(1).param_->tmtSetVel)) < 0.1f && 
      fabs(fabs(tmotors_->at(2).param_->tmtGetVel) - fabs(tmotors_->at(2).param_->tmtSetVel)) < 0.1f && 
      fabs(fabs(tmotors_->at(3).param_->tmtGetVel) - fabs(tmotors_->at(3).param_->tmtSetVel)) < 0.1f){
      tmotors_->at(0).param_->motion_state = kPullingUp;
      printf("Tmotors up done!\n");
      break;
    }
    }
  }
  // Tmotors move down
  void Motion::TMoveDown()
  {
    while(1){
    // Tmotor move down 
    for (int i = 0; i < 2; i++)
    {
#if LEAK_ROB
      tmotors_->at(2*i).param_->dir = -1;
#else
      tmotors_->at(2*i).param_->dir = 1;
#endif
      tmotors_->at(2*i).param_->tmtSetVel = GearBox();
      tmotors_->at(2*i).param_->tmtSetKD = Kd;
      tmotors_->at(2*i).SetVel(tmotors_->at(2*i).param_->dir, tmotors_->at(2*i).param_->tmtSetVel,
                                 tmotors_->at(2*i).param_->tmtSetKD);
      tmotors_->at(2*i + 1).param_->dir = 1;
      tmotors_->at(2*i + 1).param_->tmtSetVel = GearBox();
      tmotors_->at(2*i + 1).param_->tmtSetKD = Kd;
      tmotors_->at(2*i + 1).SetVel(tmotors_->at(2*i + 1).param_->dir, 
        tmotors_->at(2*i + 1).param_->tmtSetVel, tmotors_->at(2*i + 1).param_->tmtSetKD);
    }
    if(fabs(fabs(tmotors_->at(0).param_->tmtGetVel) - fabs(tmotors_->at(0).param_->tmtSetVel)) < 0.1f && 
      fabs(fabs(tmotors_->at(1).param_->tmtGetVel) - fabs(tmotors_->at(1).param_->tmtSetVel)) < 0.1f && 
      fabs(fabs(tmotors_->at(2).param_->tmtGetVel) - fabs(tmotors_->at(2).param_->tmtSetVel)) < 0.1f && 
      fabs(fabs(tmotors_->at(3).param_->tmtGetVel) - fabs(tmotors_->at(3).param_->tmtSetVel)) < 0.1f){
      tmotors_->at(0).param_->motion_state = kPullingDown;
      printf("Tmotors down done!\n");
      break;
    }
    }      
  }
  // Tmotors stop
  void Motion::TStop()
  {
    bool stop_flag = false;

    for (auto tmotor : *tmotors_)
    {
      if (fabs(tmotor.param_->tmtGetVel) < 0.01f)
      {      
        tmotor.EnterMotorMode();
        stop_flag = true;
      }
    }
    printf("Tmotors stop.\n");
    while (stop_flag == false)
    {
      // motors vel_up stop
      for (int i = 0; i < 2; i++)
      {
#if LEAK_ROB
      tmotors_->at(2*i).param_->dir = -1;
#else
      tmotors_->at(2*i).param_->dir = 1;
#endif
      tmotors_->at(2*i).param_->tmtSetVel = 0;
      tmotors_->at(2*i).param_->tmtSetKD = Kd;
      tmotors_->at(2*i).SetVel(tmotors_->at(2*i).param_->dir, tmotors_->at(2*i).param_->tmtSetVel,
                                 tmotors_->at(2*i).param_->tmtSetKD);
      tmotors_->at(2*i + 1).param_->dir = 1;
      tmotors_->at(2*i + 1).param_->tmtSetVel = 0;
      tmotors_->at(2*i + 1).param_->tmtSetKD = Kd;
      tmotors_->at(2*i + 1).SetVel(tmotors_->at(2*i + 1).param_->dir, 
        tmotors_->at(2*i + 1).param_->tmtSetVel, tmotors_->at(2*i + 1).param_->tmtSetKD);
      
      }
      if(fabs(fabs(tmotors_->at(0).param_->tmtGetVel) - fabs(tmotors_->at(0).param_->tmtSetVel)) < 0.1f && 
      fabs(fabs(tmotors_->at(1).param_->tmtGetVel) - fabs(tmotors_->at(1).param_->tmtSetVel)) < 0.1f && 
      fabs(fabs(tmotors_->at(2).param_->tmtGetVel) - fabs(tmotors_->at(2).param_->tmtSetVel)) < 0.1f && 
      fabs(fabs(tmotors_->at(3).param_->tmtGetVel) - fabs(tmotors_->at(3).param_->tmtSetVel)) < 0.1f){
        tmotors_->at(0).param_->motion_state = kStop;
        printf("Tmotors stop done!\n");
        break;
       }
    }
    //odometer_0 = odometer_;
  }
  //Pulleys speed mode
  void Motion::PulleysLoose()
  {
    __s32 speed1 = kPulleysLooseSpeed;
    __s32 speed2 = kPulleysTightenSpeed;

    maxons_->at(3).SetRMDSpeed(maxons_->at(3).param_, maxons_->at(4).param_, speed1, speed2);
    printf("PulleysLoose:Vel1=%d Vel2=%d!\n", maxons_->at(3).param_->actual_average_vel, 
      maxons_->at(4).param_->actual_average_vel);

    maxons_->at(3).param_->motion_state = kLoose;
    maxons_->at(4).param_->motion_state = kLoose;

  }
  //Pulleys speed tighten
  void Motion::PulleysTighten()
  {
    __s32 speed1 = kPulleysTightenSpeed;
    __s32 speed2 = kPulleysLooseSpeed;

    maxons_->at(3).SetRMDSpeed(maxons_->at(3).param_, maxons_->at(4).param_, speed1, speed2);
    printf("PulleysLoose:Vel1=%d Vel2=%d!\n",
    maxons_->at(3).param_->actual_average_vel, maxons_->at(4).param_->actual_average_vel);

    maxons_->at(3).param_->motion_state = kTighten;
    maxons_->at(4).param_->motion_state = kTighten;
  }
  //Pulleys torque ctl
  void Motion::PulleysTorque()
  {
    //torque
    __s16 torque1 = kPulleysHoldTorque;
    __s16 torque2 = -kPulleysHoldTorque;
    
    maxons_->at(3).SetRMDTorque(maxons_->at(3).param_, maxons_->at(4).param_, torque1, torque2);
    
    if(abs(maxons_->at(3).param_->TrqPV - torque1) < 11 && 
        abs(maxons_->at(4).param_->TrqPV - torque2) < 11){
      maxons_->at(3).param_->motion_state = kHold;
      maxons_->at(4).param_->motion_state = kHold;
      log_info("pulleysTorque :Vel1=%d Vel2=%d!\n", maxons_->at(3).param_->actual_average_vel, 
        maxons_->at(4).param_->actual_average_vel);
    }
  }
  //Pulleys stop
  void Motion::PulleysStop(){
    maxons_->at(3).TxRMD2(maxons_->at(3).param_->motor_id, 0);
    maxons_->at(4).TxRMD2(maxons_->at(4).param_->motor_id, 0);
    //maxons_->at(3).SetRMDSpeed(maxons_->at(3).param_, maxons_->at(4).param_, 0, 0);
    maxons_->at(3).SetRMDPos(maxons_->at(3).param_, maxons_->at(4).param_, 0, 0);
    printf("PulleysStop Loop:Vel1=%d Vel2=%d!\n",
    maxons_->at(3).param_->actual_average_vel, maxons_->at(4).param_->actual_average_vel);

    maxons_->at(3).param_->motion_state = kStop;
    maxons_->at(4).param_->motion_state = kStop;    
  }
  //Pulleys up
  void Motion::PulleysPullingup(){
    __s32 speed1 = kPulleysUpSpeed;
    __s32 speed2 = kPulleysDownSpeed;

    maxons_->at(3).SetRMDSpeed(maxons_->at(3).param_, maxons_->at(4).param_, speed1, speed2);
    printf("PulleysLoose:Vel1=%d Vel2=%d!\n",
    maxons_->at(3).param_->actual_average_vel, maxons_->at(4).param_->actual_average_vel);

    maxons_->at(3).param_->motion_state = kPullingUp;
    maxons_->at(4).param_->motion_state = kPullingUp;
  }
  //Pulleys down
  void Motion::PulleysPullingdown(){
    __s32 speed1 = kPulleysDownSpeed;
    __s32 speed2 = kPulleysUpSpeed;

    maxons_->at(3).SetRMDSpeed(maxons_->at(3).param_, maxons_->at(4).param_, speed1, speed2);
    printf("PulleysLoose:Vel1=%d Vel2=%d!\n",
    maxons_->at(3).param_->actual_average_vel, maxons_->at(4).param_->actual_average_vel);

    maxons_->at(3).param_->motion_state = kPullingDown;
    maxons_->at(4).param_->motion_state = kPullingDown;

  }

  // Precursor move up
  void Motion::PMoveUp()
  {
    if(maxons_->at(3).param_->motion_state == kStop &&  maxons_->at(4).param_->motion_state 
    == kStop && maxons_->at(1).param_->motion_state == kHold && 
    maxons_->at(2).param_->motion_state == kHold){
      // UpClaw loose
      UpClawLoose();
      // Pulley loose (Torque?)
      PulleysLoose();
      // Tmotors move up
      TMoveUp();
    } else{
      printf("PMoveUp-Warning: DownClaws hold or Pulleys stop!!\n");
    }
  }
  // Precursor move down
  void Motion::PMoveDown()
  {
    if(maxons_->at(3).param_->motion_state == kStop &&  maxons_->at(4).param_->motion_state 
    == kStop && maxons_->at(1).param_->motion_state == kHold && 
    maxons_->at(2).param_->motion_state == kHold){
      // UpClaw loose
      UpClawLoose();
      // Tmotors move down
      TMoveDown();
      // Pulley tighten (Torque?)
      PulleysTighten();
    } else{
      printf("PMoveDown-Warning: DownClaws hold or Pulleys stop!!\n");
    }

  }
  // Precursor stop
  void Motion::PStop()
  {
    if(maxons_->at(1).param_->motion_state == kHold && 
    maxons_->at(2).param_->motion_state == kHold){
      // Tmotors stop
      TStop();
      //UpClaw hold
      UpClawHold();
      //Pulley line tighten
      PulleysTorque();
      //Pulleys stop
      PulleysStop();
    } else{
      printf("PStop-Warning: DownClaws hold!!\n");
    }
  }

  // MainFrame move up
  void Motion::MMoveUp(){
    if(maxons_->at(0).param_->motion_state == kHold && maxons_->at(3).param_->motion_state == kStop
     && maxons_->at(4).param_->motion_state == kStop){
      //Pulleys stop
      //PulleysStop();
      //DownClaw loose
      DownClawLoose();
      //Pulleys up
      PulleysPullingup();
    } else{
      printf("MMoveUp-Warning: UpClaw hold or Pulleys stop!!\n");
    }
  }
  // MainFrame move down
  void Motion::MMoveDown(){
    if(maxons_->at(0).param_->motion_state == kHold && maxons_->at(3).param_->motion_state == kStop
     && maxons_->at(4).param_->motion_state == kStop){
      //DownClaw loose
      DownClawLoose();
      //Pulleys up
      PulleysPullingdown();
    } else{
      printf("MMoveDown-Warning: UpClaw hold or Pulleys stop!!\n");
    }
  }
  // MainFrame stop
  void Motion::MStop(){
    if(maxons_->at(0).param_->motion_state == kHold){
      //Pulleys stop
      PulleysStop();
      //DownClaw hold
      DownClawHold();
      //Pulley line tighten
      PulleysTorque();
      //Pulleys stop
      PulleysStop();      
    } else{
      printf("MStop-Warning: UpClaw hold!!\n");
    }
  }

  // Robot move up
  void Motion::MoveUp()
  {
    PMoveUp();
    usleep(kDelayEpos * 1000);
    PStop();
    MMoveUp();
    usleep(kDelayEpos * 1000);
    MStop();
  }
  // Robot stop
  void Motion::Stop()
  {
    //DownClaw hold
    DownClawHold();
    //UpClaw hold
    UpClawHold();
    //Pulley line tighten
    PulleysTorque();
    //Pulleys stop
    PulleysStop();
  }
  // Robot move down
  void Motion::MoveDown()
  {
    MMoveDown();
    usleep(kDelayEpos * 1000);
    MStop();
    PMoveDown();
    usleep(kDelayEpos * 1000);
    PStop();
  }

#define USE_VECTOR 0

#if USE_VECTOR
  static std::vector<double> odom_turns;
  static std::vector<double>::iterator max_turn;
#endif

  void Motion::GetTMotorsParam()
  {
    // get received can frame
    can_frame recv_frame;
    double odm_sum = 0;
    double max_turn = 0;
    //   poll to get frame, change to epoll?
    for (;;)
    {
      // all motors share one CAN bus
      tmotors_->begin()->can_dev_->receive(&recv_frame);

      //   assign parameters
      for (auto tmotor : *tmotors_)
      {
        tmotor.GetParam(recv_frame);
      }
      for (auto maxon : *maxons_)
      {
        maxon.CanDisPatch1();
      }

#if USE_VECTOR
      // calc odometer
      for (auto tmotor : *tmotors_)
      {
        // odm_sum += tmotor.param_->turns;
        odom_turns.emplace_back(std::move(tmotor.param_->turns));
      }

#if 1
      // sort odom vector
      std::sort(odom_turns.begin(), odom_turns.end());

      // clear the max element
      odom_turns.pop_back();

      odometer_ =
          (double)(std::accumulate(odom_turns.begin(), odom_turns.end(), 0) /
                   odom_turns.size() * 370.52);
#else

      max_turn = std::max_element(odom_turns.begin(), odom_turns.end());

      odometer_ =
          (double)((std::accumulate(odom_turns.begin(), odom_turns.end(), 0) -
                    *max_turn) /
                   (odom_turns.size() - 1) * 370.52);
#endif
      // clear vector
      odom_turns.clear();

#else

#if LEAK_ROB
      for (int i = 0; i < 2; i++)
      {
        odm_sum -= tmotors_->at(i).param_->turns;
        //odm_sum -= tmotors_->at(i+4).param_->turns;
      }

      for (int i = 2; i < 4; i++)
      {
        odm_sum += tmotors_->at(i).param_->turns;
        //odm_sum += tmotors_->at(i+4).param_->turns;
      }

#else
      // calc odometer
      for (auto tmotor : *tmotors_)
      {
        odm_sum += tmotor.param_->turns;
      }

#endif
      log_debug("turn:%f", (double)(odm_sum / tmotors_->size()));

      odometer_ = (double)(odm_sum / (tmotors_->size()) * 370.52); // mm

      odm_sum = 0; // clear
      max_turn = 0;
#endif
      log_debug("odometer:%f mm", odometer_);
      // refresh 10ms
      usleep(10000);
    }
  }

  // read the can frame except tmotors
  void Motion::CanDisPatch(void) {  
  }

  void Motion::GetmaxonsParam()
  {
    for (;;) {
    // delay 10us
    usleep(10);
    CanDisPatch();
    }
  }

  // select speed
  float Motion::GearBox()
  {
    switch (motion_cmd_->speed_gear)
    {
    case kNormal:
#if MIMIC_MOTION
      log_info("normal speed");
#else
      return kNormalSpeed;
#endif
      break;

    case kHigh:
#if MIMIC_MOTION
      log_info("high speed");
#else
      return 2 * kNormalSpeed;
#endif
      break;

    case kLow:
#if MIMIC_MOTION
      log_info("low speed");
#else
      return 0.5 * kNormalSpeed;
#endif
      break;

      // defualt normal speed
    default:
#if MIMIC_MOTION
      log_info("normal speed");
#else
      return kNormalSpeed;
#endif
      break;
    }

#if MIMIC_MOTION
    return 0;
#endif
  }

  bool IsHostAlive()
  {
    //check every 30s
  }
} // namespace ccr_split
