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
    
    // select mannual motion command
    // select mannual motion command
    /*cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
    cout << "0/1/2/13 for Robot Stop, MoveUp, MoveDown, reset; "<< endl;
    cout << "3/4/5 for Precursor Stop, MoveUp, MoveDown;"<< endl;
    cout << "6/7/8 for MainFrame Stop, MoveUp, MoveDown;"<< endl;
    cout << "9/10/11/12 for UpClawHold, UpClawLoose, DownClawHold, DownClawLoose" << endl;
    cout << "14/15/16 for TcpClose, MotorsIdle, MaxonsIdle" << endl;
    
    int flag_command = 0;
    cout << "Please input command: ";
    cin >> flag_command;
    for(int i = 0; i < 20; i++){
      if(i == flag_command){
        motion_cmd_->command = i;
      }
    }
    */

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
      break;

    case kStopCmd:
      if (motion_state_->current_state != kStopState){
        MotorReset();
        Stop();
        printf("Robot stop done!\n");
      } else{      
        //printf("Robot stop done!\n");
      }

      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kStopState;
      break;

    case pMoveUpCmd:
      if (Range_Up->range > 2.0 && motion_state_->current_state 
        != pMoveUpState){
        PMoveUp();
        motion_state_->current_state = pMoveUpState;
        printf("pMoveUp start!\n");
      } else if(Range_Up->range <= 2.0){
        PStop();
        motion_state_->current_state = pStopState;
        //printf("pMoveUp finished!\n");
      }
      
      break;

    case pMoveDownCmd:
      if (motion_state_->current_state != pMoveDownState){
        PMoveDown();
        printf("pMoveDown finished!\n");
      } else{
        //printf("pMoveDown finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = pMoveDownState;
      break;

    case pStopCmd:
      if (motion_state_->current_state != pStopState){
        PStop();
        printf("pStop finished!\n");
      } else{
        //printf("pStop finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = pStopState;
      break;

    case mMoveUpCmd:
      if (motion_state_->current_state != mMoveUpState){
        MMoveUp();
        printf("mMoveUp finished!\n");
      } else{
        //printf("mMoveUp finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = mMoveUpState;
      break;

    case mMoveDownCmd:
      if (motion_state_->current_state != mMoveDownState){
        MMoveDown();
        printf("mMoveDown finished!\n");
      } else{
        //printf("mMoveDown finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = mMoveDownState;
      break;

    case mStopCmd:
      if (motion_state_->current_state != mStopState){
        MStop();
        printf("mStop finished!\n");
      } else{
        //printf("mStop finished!\n");
      } 
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = mStopState;
      break;

    case kUpClawHoldCmd:
      if (motion_state_->current_state != kUpClawHoldState){
        UpClawHold();
        printf("kUpClawHold finished!\n");
      } else{
        //printf("kUpClawHold finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kUpClawHoldState;
      break;

    case kUpClawLooseCmd:
      if (motion_state_->current_state != kUpClawLooseState){
        UpClawLoose();
        printf("kUpClawLoose finished!\n");
      } else{
        //printf("kUpClawLoose finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kUpClawLooseState;
      break;      

    case kDownClawHoldCmd:
      if (motion_state_->current_state != kDownClawHoldState){
        DownClawHold();
        printf("kDownClawHold finished!\n");
      } else{
        //printf("kDownClawHold finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kDownClawHoldState;
      break;

    case kDownClawLooseCmd:
      if(motion_state_->current_state == kMotorsIdleState){
        Stop();
        motion_state_->current_state = kStopState;
      } else if(motion_state_->current_state != kDownClawLooseState){
        DownClawLoose();
        printf("kDownClawLoose finished!\n");
        motion_state_->current_cmd = motion_cmd_->command;
        motion_state_->current_state = kDownClawLooseState;
      } else{
        //printf("kDownClawLoose finished!\n");
      }
      
      break;

    case kMotorResetCmd:
      if (motion_state_->current_state != kMotorResetState){
        MotorReset();
        printf("kMotorReset finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kMotorResetState;
      break;

    case kTcpCloseCmd:
      if (motion_state_->current_state != kTcpCloseState){
        maxons_->begin()->tcp_->closePort();
        printf("kTcpClose finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kTcpCloseState;
      break;

    case kMotorsIdleCmd:
      if (motion_state_->current_state != kMotorsIdleState){
        //maxons_->begin()->NMTstop(0);
       
        Idle();

        printf("MotorsIdle finished!\n");
      }
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kMotorsIdleState;
      break;

    case kMaxonsIdleCmd:

      MoveUp();

      MoveDown();

      //if (motion_state_->current_state != kMaxonsIdleState){
      //  printf("kMaxonsIdle finished!\n");
        
      //}
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kMaxonsIdleState;
      break;

    case kMotionStopCmd:
      printf("kMotionStop finished!\n");
      //motion_stop_signal_ = true;
      break;

    // defualt stop
    default:
#if MIMIC_MOTION
      log_debug("stop state.");
      log_debug("gearbox:%f", GearBox());
#else
      if (motion_state_->current_state != kStopState){
        MotorReset();
        Stop();}
#endif
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kStopState;
      break;
    }
  }

  void Motion::Idle(){
    for (auto tmotor : *tmotors_) {
      tmotor.ExitMotorMode();
    }
    for (int i = 0; i < 3; i ++) {
      maxons_->at(i).MotorDisable();
    }
    maxons_->at(3).TxRMD0(8, 0x80);
    maxons_->at(4).TxRMD0(9, 0x80);
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
    // check heartbeat
    /*if (heartbeat_flag == false)
    {
      if (stop_cnt == 0)
      {
        stop_cnt++;
        Stop();
        sleep(1);
      }
      motion_cmd_->motion_mode = kReturnMode;
      //motion_cmd_->motion_mode = kReturnMode;
    }*/

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
            //ret = Return();
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
	  } 
	  printf("Upclaw hold done!\n");
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
    maxons_->at(2).MotorEnable();
  
    maxons_->at(1).SetTargetTorque(maxons_->at(1).param_, 
    maxons_->at(2).param_, kDownClawHoldTorque, kDownClawHoldTorque);
    
    printf("DownClawHold Done: Toq %d, %d.\n", maxons_->at(1).param_->actual_average_torque, 
      maxons_->at(2).param_->actual_average_torque);
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
	  
	  //__s8 flag = maxons_->at(1).SetMotorAbsPos(maxons_->at(1).param_, maxons_->at(2).param_, 
    //  maxons_->at(1).param_->PosPV + pos1, maxons_->at(2).param_->PosPV + pos2);
    __s8 flag = maxons_->begin()->SetMotorAbsPos(maxons_->at(1).param_, maxons_->at(2).param_, 
      maxons_->at(1).param_->PosPV + pos1, maxons_->at(2).param_->PosPV + pos2);

	  //printf("down claw loose Done!\n");
    if(flag > 0){
	    // change downclaw1 state to
	    maxons_->at(1).param_->motion_state = kLoose;
	    maxons_->at(2).param_->motion_state = kLoose;
    } else{
      //maxons_->at(1).SetMotorAbsPos(maxons_->at(1).param_, maxons_->at(2).param_, 
      //maxons_->at(1).param_->PosPV + pos1, maxons_->at(2).param_->PosPV + pos2);
    }   
  }
  // Tmotors move up
  void Motion::TMoveUp()
  {
    int j = 0;
    float vel_average;
    __s32 speed1 = __s32(kPulleysLooseSpeed * GearBox());
    __s32 speed2 = __s32(-kPulleysLooseSpeed * GearBox());
    while(1){
    // Tmotors move up    
    for (int i = 0; i < 2; i++)
    {
#if LEAK_ROB
      tmotors_->at(2*i).param_->dir = 1;
#else
      tmotors_->at(2*i).param_->dir = -1;
#endif
      tmotors_->at(2*i).param_->tmtSetVel = GearBox() * 1.5;
      tmotors_->at(2*i).param_->tmtSetKD = Kd;
      tmotors_->at(2*i).SetVel(tmotors_->at(2*i).param_->dir, tmotors_->at(2*i).param_->tmtSetVel,
                                 tmotors_->at(2*i).param_->tmtSetKD);
      
      tmotors_->at(2*i + 1).param_->dir = -1;
      tmotors_->at(2*i + 1).param_->tmtSetVel = GearBox() * 1.5;
      tmotors_->at(2*i + 1).param_->tmtSetKD = Kd;
      tmotors_->at(2*i + 1).SetVel(tmotors_->at(2*i + 1).param_->dir, 
        tmotors_->at(2*i + 1).param_->tmtSetVel, tmotors_->at(2*i + 1).param_->tmtSetKD);
      
    }
    for (int k = 0; k < 4; k++)
    {
      vel_average += fabs(tmotors_->at(k).param_->tmtGetVel);
    }
    vel_average = vel_average / 4;
    if((abs(maxons_->at(3).param_->TrqPV) > 10 || abs(maxons_->at(4).param_->TrqPV) > 10) && 
      (vel_average > 0.1*tmotors_->at(0).param_->tmtSetVel))
    {
      // PulleysLoose or?
      PulleysLoose();
      //printf("Tup-vel%f &PulleysLoose-torq%d!\n", vel_average, j);
      //maxons_->at(3).TxRMD2(maxons_->at(3).param_->motor_id, speed1);
      //maxons_->at(4).TxRMD2(maxons_->at(4).param_->motor_id, speed2);

    } else{
      maxons_->at(3).TxRMD2(maxons_->at(3).param_->motor_id, 0);
      maxons_->at(4).TxRMD2(maxons_->at(4).param_->motor_id, 0);
    }

    if(vel_average > 0.2*tmotors_->at(0).param_->tmtSetVel && 
      tmotors_->at(0).param_->motion_state != kPullingUp){
      tmotors_->at(0).param_->motion_state = kPullingUp;
      printf("Tmotors up start loop-%d!\n", j);
      break;
    }
    j ++;
    if(j > 502){
      printf("Tmotors up-%4.2f break!\n");
      // 
      break;
    }
    usleep(200);
    }
    //Stop();
    //PulleysStop();
    //PStop();
    //printf("Tmotors up done-%4.2f!\n", Range_Up->range);
  }
  // Tmotors move down
  void Motion::TMoveDown()
  {
    int j = 0;
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
      //usleep(110);
      tmotors_->at(2*i + 1).param_->dir = 1;
      tmotors_->at(2*i + 1).param_->tmtSetVel = GearBox();
      tmotors_->at(2*i + 1).param_->tmtSetKD = Kd;
      tmotors_->at(2*i + 1).SetVel(tmotors_->at(2*i + 1).param_->dir, 
        tmotors_->at(2*i + 1).param_->tmtSetVel, tmotors_->at(2*i + 1).param_->tmtSetKD);
    }

    if(fabs(tmotors_->at(0).param_->tmtGetVel) > 0.2*fabs(tmotors_->at(0).param_->tmtSetVel) && 
      fabs(tmotors_->at(1).param_->tmtGetVel) > 0.2*fabs(tmotors_->at(1).param_->tmtSetVel)&& 
      fabs(tmotors_->at(2).param_->tmtGetVel) > 0.2*fabs(tmotors_->at(2).param_->tmtSetVel)&& 
      fabs(tmotors_->at(3).param_->tmtGetVel) > 0.2*fabs(tmotors_->at(3).param_->tmtSetVel)){
      tmotors_->at(0).param_->motion_state = kPullingDown;
      printf("Tmotors down start  loop-%d!\n", j);
      break;
    }
    j ++;
    if(j > 501){
      printf("Tmotors down break!\n");
      break;
      //printf;
    }
    usleep(202);
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
    
    //printf("Tmotors stop odometer_ = %f mm.\n", odometer_);  
    stop_flag = false;
    int j = 0;
    while (stop_flag == false)
    {
      // motors vel_up stop
      for (int i = 0; i < 2; i++)
      {
      tmotors_->at(2*i).param_->tmtSetVel = 0.02f;
      tmotors_->at(2*i).param_->tmtSetKD = 0.3f;
      tmotors_->at(2*i).SetVel(tmotors_->at(2*i).param_->tmtSetVel,
                              tmotors_->at(2*i).param_->tmtSetKD);
      tmotors_->at(2*i + 1).param_->tmtSetVel = -0.02f;
      tmotors_->at(2*i + 1).param_->tmtSetKD = 0.3f;
      tmotors_->at(2*i + 1).SetVel(tmotors_->at(2*i + 1).param_->tmtSetVel, 
        tmotors_->at(2*i + 1).param_->tmtSetKD);
      
      }

      if(fabs(tmotors_->at(0).param_->tmtGetVel) < 0.03f && fabs(tmotors_->at(1).param_->tmtGetVel)< 0.03f && 
        fabs(tmotors_->at(2).param_->tmtGetVel) < 0.03f && fabs(tmotors_->at(3).param_->tmtGetVel) < 0.03f){
        tmotors_->at(0).param_->motion_state = kStop;
        printf("Tmotors stop done loop-%d!\n", j);
        break;
      }
      if(j % 5 == 0){
        maxons_->at(3).TxRMD3(maxons_->at(3).param_->motor_id, 0);
        maxons_->at(4).TxRMD3(maxons_->at(4).param_->motor_id, 0);
        //
      }
      if(j > 50){
        printf("Tmotors stop break!\n");
        //printf
        break;
      }
      j ++;
      usleep(100);
    }
    for (int i = 0; i < 2; i++)
      {
      tmotors_->at(2*i).param_->tmtSetVel = 0.02;
      tmotors_->at(2*i).param_->tmtSetKD = 4.0;
      tmotors_->at(2*i).SetVel(tmotors_->at(2*i).param_->tmtSetVel,
                              tmotors_->at(2*i).param_->tmtSetKD);
      tmotors_->at(2*i + 1).param_->tmtSetVel = -0.02;
      tmotors_->at(2*i + 1).param_->tmtSetKD = 4.0;
      tmotors_->at(2*i + 1).SetVel(tmotors_->at(2*i + 1).param_->tmtSetVel, 
        tmotors_->at(2*i + 1).param_->tmtSetKD);
      
      }
    //printf;
  }
  //Pulleys speed mode
  void Motion::PulleysLoose()
  {
    __s32 speed1 = __s32(kPulleysLooseSpeed * GearBox());
    __s32 speed2 = __s32(-kPulleysLooseSpeed * GearBox());

    maxons_->at(3).SetRMDSpeed(maxons_->at(3).param_, maxons_->at(4).param_, speed1, speed2);
    //printf("PulleysLoose:Vel1=%d Vel2=%d!\n", maxons_->at(3).param_->actual_average_vel, 
    //  maxons_->at(4).param_->actual_average_vel);

    maxons_->at(3).param_->motion_state = kLoose;
    maxons_->at(4).param_->motion_state = kLoose;

  }
  //Pulleys speed tighten
  void Motion::PulleysTighten()
  {
    __s32 speed1 = __s32(kPulleysTightenSpeed * GearBox());
    __s32 speed2 = __s32(-kPulleysTightenSpeed * GearBox());

    maxons_->at(3).SetRMDSpeed(maxons_->at(3).param_, maxons_->at(4).param_, speed1, speed2);
    //printf("PulleysTighten:Vel1=%d Vel2=%d!\n",
    //maxons_->at(3).param_->actual_average_vel, maxons_->at(4).param_->actual_average_vel);

    maxons_->at(3).param_->motion_state = kTighten;
    maxons_->at(4).param_->motion_state = kTighten;
  }
  //Pulleys torque ctl
  void Motion::PulleysTorque()
  {
    //torque
    __s16 torque1 = kPulleysHoldTorque;
    __s16 torque2 = -kPulleysHoldTorque;
    if(maxons_->at(0).param_->motion_state == kHold && maxons_->at(1).param_->motion_state 
      == kHold && maxons_->at(2).param_->motion_state == kHold){
      maxons_->at(3).SetRMDTorque(maxons_->at(3).param_, maxons_->at(4).param_, torque1, torque2);

      if(abs(maxons_->at(3).param_->TrqPV - torque1) < 11 && 
          abs(maxons_->at(4).param_->TrqPV - torque2) < 11){
        maxons_->at(3).param_->motion_state = kHold;
        maxons_->at(4).param_->motion_state = kHold;
        printf("pulleysTorque :Vel1=%d Vel2=%d!\n", maxons_->at(3).param_->actual_average_vel, 
          maxons_->at(4).param_->actual_average_vel);
      }
    } else{
      printf("PulleysTorque need ClawHold: %d, %d, %d!\n", maxons_->at(0).param_->motion_state, 
        maxons_->at(1).param_->motion_state, maxons_->at(2).param_->motion_state);
    }
  }
  //Pulleys stop
  void Motion::PulleysStop(){
    float f1 = maxons_->at(3).param_->pos_sum;
    float f2 = maxons_->at(4).param_->pos_sum;
    printf("PulleysStop odometer_ = %f %f mm.\n", f1, f2);
    //
    maxons_->at(3).SetRMDPos(maxons_->at(3).param_, maxons_->at(4).param_, 0, 0);
    printf("PulleysStop Loop:Vel1=%d Vel2=%d!\n",
    maxons_->at(3).param_->actual_average_vel, maxons_->at(4).param_->actual_average_vel);

    maxons_->at(3).param_->motion_state = kStop;
    maxons_->at(4).param_->motion_state = kStop;    
  }
  //Pulleys up
  void Motion::PulleysPullingup(){
    __s32 speed1 = kPulleysUpSpeed * GearBox();
    __s32 speed2 = kPulleysDownSpeed * GearBox();

    maxons_->at(3).param_->motion_state = kPullingUp;
    maxons_->at(4).param_->motion_state = kPullingUp;

    maxons_->at(3).SetRMDSpeed(maxons_->at(3).param_, maxons_->at(4).param_, speed1, speed2);
    printf("PulleysLoose:Vel1=%d Vel2=%d!\n",
    maxons_->at(3).param_->actual_average_vel, maxons_->at(4).param_->actual_average_vel);

  }
  //Pulleys down
  void Motion::PulleysPullingdown(){
    __s32 speed1 = kPulleysDownSpeed * GearBox();
    __s32 speed2 = kPulleysUpSpeed * GearBox();

    maxons_->at(3).param_->motion_state = kPullingDown;
    maxons_->at(4).param_->motion_state = kPullingDown;

    maxons_->at(3).SetRMDSpeed(maxons_->at(3).param_, maxons_->at(4).param_, speed1, speed2);
    printf("PulleysLoose:Vel1=%d Vel2=%d!\n",
    maxons_->at(3).param_->actual_average_vel, maxons_->at(4).param_->actual_average_vel);

  }

  // Precursor move up
  void Motion::PMoveUp()
  {
    if(maxons_->at(3).param_->motion_state == kStop &&  maxons_->at(4).param_->motion_state 
    == kStop && maxons_->at(1).param_->motion_state == kHold && 
    maxons_->at(2).param_->motion_state == kHold){
      //if(Range_Up->range > 2.0){
        // UpClaw loose
        UpClawLoose();
        // Pulley loose (Torque?)
        //PulleysLoose();
        // Tmotors move up
        TMoveUp();
      /*} else{
        printf("Up-%4.2f limit: stop!!\n", Range_Up->range);
        if(motion_state_->current_state != pStopCmd){
          PStop();
          motion_state_->current_state == pStopCmd;
        }
      }*/
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
      //Pulleys stop
      PulleysStop();
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
    
    Range_Up->range_ = Range_Up->range;
    PMoveUp();
    //sleep(2);
    //PStop();
    while(Range_Up->range > 2.0 && (Range_Up->range_ - 
      Range_Up->range) < 2.5 ){
      usleep(1000*100);
    }
    PStop();

    MMoveUp();
    //sleep(2);
    //MStop();
    motion_state_->current_state = mMoveUpState;
    //while(motion_state_->current_state != kStopState){
    //  usleep(1000*100);
    //}
  }
  // Robot stop
  void Motion::Stop()
  {
    //Pulley stop
    PulleysStop();
    //maxons_->at(3).TxRMD3(maxons_->at(3).param_->motor_id, 0);
    //maxons_->at(4).TxRMD3(maxons_->at(4).param_->motor_id, 0);
    //DownClaw hold
    DownClawHold();
    // TStop or pStop
    //TStop();
    for (int i = 0; i < 2; i++)
    {
      tmotors_->at(2*i).param_->tmtSetVel = 0.02f;
      tmotors_->at(2*i).param_->tmtSetKD = 0.3f;
      tmotors_->at(2*i).SetVel(tmotors_->at(2*i).param_->tmtSetVel,
                              tmotors_->at(2*i).param_->tmtSetKD);
      tmotors_->at(2*i + 1).param_->tmtSetVel = -0.02f;
      tmotors_->at(2*i + 1).param_->tmtSetKD = 0.3f;
      tmotors_->at(2*i + 1).SetVel(tmotors_->at(2*i + 1).param_->tmtSetVel, 
        tmotors_->at(2*i + 1).param_->tmtSetKD);
    }

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
    motion_state_->current_state = mMoveDownState;
    //sleep(2);
    //MStop();
    while(motion_state_->current_state != kStopState){
      usleep(1000*100);
    }
    PMoveDown();
    motion_state_->current_state = pMoveDownState;
    //sleep(2);
    //PStop();
    while(motion_state_->current_state != kStopState){
      usleep(1000*100);
    }
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
    int j = 0;
    //   poll to get frame, change to epoll?
    for (;;)
    {
      // all motors share one CAN bus
      tmotors_->begin()->can_dev_->receive(&recv_frame);
      //printf("GetTMotorsParam\n");
      //   assign parameters
      if(recv_frame.can_id == 0)
      {
      for (auto tmotor : *tmotors_)
      {
        tmotor.GetParam(recv_frame);
      }
      //printf("endGetTMotorsParam\n");
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
        odm_sum -= tmotors_->at(2*i + 1).param_->turns;
      }
      //
      for (int i = 0; i < 2; i++)
      {
        odm_sum += tmotors_->at(2*i).param_->turns;
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
      //odometer_ = (double)(odm_sum / 3 * 370.52); //3电机
      odm_sum = 0; // clear
      max_turn = 0;
#endif
      log_debug("odometer:%f mm", odometer_);
    }
    // refresh ms
    usleep(5000);
    if(j%1000 == 0){
      printf("turns:%f, %f, %f, %f\n", tmotors_->at(0).param_->turns, tmotors_->at(1).param_->turns, 
      tmotors_->at(2).param_->turns, tmotors_->at(3).param_->turns);
      printf("odometer_%f\n", odometer_);
      //printf("odometer_%f\n", odometer_);
    }
    j++;
    
    }
  }

  void Motion::GetmaxonsParam()
  {
    int i, j1, j2;
    float p;
    for (;;) {
      
      can_frame recv_frame;
      __u16 cob_id, SlaveId, t;
      
      recv_can(recv_frame, *(maxons_->begin()->tcp_));
      SlaveId = (recv_frame.can_id & 0x0F);
      cob_id = recv_frame.can_id & (~0x000F);
      //printf("recv_can865\n");     
      for (auto maxon : *maxons_){
        if(maxon.id_ == SlaveId){
          maxon.MotorParaRead(cob_id, maxon.param_, &recv_frame);
          // calc turns
          if(maxon.param_->motion_state == kPullingUp || 
            maxon.param_->motion_state == kPullingDown){
            if(maxon.id_ == kPulley1){
              if(j1 <1){ p= maxon.param_->PosPV;}
              maxon.param_->pos_sum +=maxon.param_->PosPV - p;
              p = maxon.param_->PosPV;
              j1 ++;
            } else if(maxon.id_ == kPulley2){
              if(j2 <1){ p= maxon.param_->PosPV;}
              maxon.param_->pos_sum +=maxon.param_->PosPV - p;
              p = maxon.param_->PosPV;
              j2 ++;
            } else{
              j1 = 0; j2 = 0;
            }
          }
        }
      }
      // delay 10ms
      usleep(100);
      i ++;
    }
  }

  void Motion::GetmaxonsParam1()
  {
    int i;
    for (;;) {
      
      can_frame recv_frame;
      __u16 cob_id, SlaveId;

      maxons_->begin()->can_dev_->receive(&recv_frame);
      SlaveId = (recv_frame.can_id & 0x7F);
      cob_id = recv_frame.can_id & (~0x007F);
      maxons_->begin()->MotorParaRead(cob_id, maxons_->begin()->param_, &recv_frame);
      // delay 10ms
      usleep(100);
      
      i ++;
    }
  }

  void Motion::TFminiRecv()
  {
    //TFmini Up
    float dist1 = 0;
    float dist1_old = 0;
    float dist1_stop = 0;
    std::string id1 = "TFmini";
    std::string portName1 = "/dev/TFmini";
    int baud_rate1 = 115200;
    tfmini_obj = new benewake::TFmini(portName1, baud_rate1);
    Range_Up->radiation_type = INFRARED;
    Range_Up->field_of_view = 0.04f;
    Range_Up->min_range = 0.1f;
    Range_Up->max_range = 12.0f;   
    Range_Up->frame_id = id1;

    int i;
    while (false == stop_tfmini_signal_ )
    {
      //printf("tfmini_thread_ start\n");
      dist1 = tfmini_obj->getDist();
      dist1_stop = 3.5f;
      if(i % 300 == 0)
       printf("dist_up=%4.2fm.\n", dist1);
      if(dist1 > 0 && dist1 < Range_Up->max_range)
      {
        Range_Up->range = dist1;                       
        /*if((dist1 < 1.5f) && (dist1 < dist1_old) && (motion_.motion_cmd_->command 
        == kMoveUpCmd || motion_.motion_cmd_->command == pMoveUpCmd)){
  
          motion_.motion_cmd_->command = kStopCmd;
         
        }*/
        dist1_old = dist1;
      }
      else if(dist1 == -1.0)
      {
        log_error("TFmini failed to read data!");
        break;
      }
      else if(dist1 == 0.0)
      {
        log_error("TFmini data 0.0 error!");
      }

      usleep(5000);
      i++;
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
