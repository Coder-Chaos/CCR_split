#pragma once
#include "common.hpp"
#include "freemodbus_tcp.h"
#include "shared_mem.h"
namespace ccr_split {

  static const int kSharedMemLength = 65536;
  static const std::string kShareMemName = "/command_shared_mem";
  static const std::string kSemaphoreName = "/command_semaphore";
  // sem_t** command_semaphore;

  /* motion mode */
  static const USHORT kManualMode = 0;
  static const USHORT kAutoMode = 1;
  static const USHORT kReturnMode = 2;
  static const USHORT kSetZero = 3;
  static const USHORT kTestMode = 4;
  enum class MotionModeCommand { ManualMode = 0, AutoMode, ReturnMode, SetZero };
  enum class MotionModeState {
    ManualMode = 0,
    AutoMode,
    ReturnMode,
    SetZero,
    TestMode
  };

  /* motion manual command */
  static const USHORT kStopCmd          = 0;
  static const USHORT kMoveUpCmd        = 1;
  static const USHORT kMoveDownCmd      = 2;
  static const USHORT pStopCmd          = 3;
  static const USHORT pMoveUpCmd        = 4;
  static const USHORT pMoveDownCmd      = 5;
  static const USHORT mStopCmd          = 6;
  static const USHORT mMoveUpCmd        = 7;
  static const USHORT mMoveDownCmd      = 8;
  static const USHORT kUpClawHoldCmd    = 9;
  static const USHORT kUpClawLooseCmd   = 10;
  static const USHORT kDownClawHoldCmd  = 11;
  static const USHORT kDownClawLooseCmd = 12;
  static const USHORT kMotorResetCmd    = 13;
  static const USHORT kTcpCloseCmd      = 14;
  static const USHORT kMotorsIdleCmd    = 15;
  static const USHORT kMaxonsIdleCmd    = 16;
  static const USHORT kMotionStopCmd    = 77;
  /*enum class MotionCommand { 
    StopCommand = 0, MoveUpCommand, MoveDownCommand, 
    PrecursorStopCommand, PrecursorMoveUpCommand, PrecursorMoveDownCommand, 
    MainStopCommand, MainMoveUpCommand, MainMoveDownCommand,
    UpClawHoldCommand, UpClawLooseCommand, DownClawHoldCommand, 
    DownClawLooseCommand
  };*/

  /* FSM motion manual state */
  
  static const USHORT kMoveUpState        = 1;
  static const USHORT kMoveDownState      = 2;
  static const USHORT pStopState          = 3;
  static const USHORT pMoveUpState        = 4;
  static const USHORT pMoveDownState      = 5;
  static const USHORT mStopState          = 6;
  static const USHORT mMoveUpState        = 7;
  static const USHORT mMoveDownState      = 8;
  static const USHORT kUpClawHoldState    = 9;
  static const USHORT kUpClawLooseState   = 10;
  static const USHORT kDownClawHoldState  = 11;
  static const USHORT kDownClawLooseState = 12;
  static const USHORT kStopState          = 13;
  static const USHORT kErrorState         = 14;
  static const USHORT kMotorResetState    = 15;
  static const USHORT kTcpCloseState      = 16;
  static const USHORT kMotorsIdleState   = 17;
  static const USHORT kMaxonsIdleState   = 18;

  /* speed gear, high normal low */
  static const USHORT kNormal = 0;
  static const USHORT kHigh = 1;
  static const USHORT kLow = 2;
  enum class SpeedGear { Normal = 0, High, Low };

  // image saving state
  enum class ImageSavingState { NotStarted = 0, Saving, Done };

  // default image sample interval 120 mm
  static const int kDefaultSampleInterval = 150;

  static const float kNormalSpeed = 5;

  /* quick stop */
  static const USHORT kQuickStopOff = 0;
  static const USHORT kQuickStopOn = 1;

  // bridge information
  struct BridgeInfo {
    // bridge name
    unsigned short name;
    //   cable no.
    unsigned short cable_no;
    // cable diameter
    unsigned short cable_diameter;
    // cable length
    unsigned short cable_len;
  };

  // time stamp
  struct TimeStamp {
    unsigned short year;
    unsigned short month;
    unsigned short day;
    unsigned short hour;
    unsigned short minute;
    unsigned short second;
  };

  /* motion command */
  struct MotionCmd {
    //   mode select
    USHORT motion_mode;

    //   motion command
    USHORT command;

    // speed gear, low, normal, high
    USHORT speed_gear;

    // quick stop command
    USHORT quick_stop;

    // bridge information
    BridgeInfo bridge_info;

    // bridge info set, 0->unset, 1->set
    USHORT bridge_info_set;

    // video stream control, 0->off, 1->on
    USHORT video_stream_control;

    // take image control, 1->save
    USHORT take_image_control;

    // take image interval, mm
    USHORT sample_interval;

    // time stamp
    struct TimeStamp time_stamp;
  };

  // image saving state
  enum class PicSaveState { NotStarted = 0, Saving, Done };

  // message to vision process
  struct VisionMessage {
    // host motion command
    MotionCmd host_motion_cmd;

    // moving odometer
    double odometer;

    // vision system control ,0->off,1->on
    unsigned short vision_system_control;
  };

  /* host command */
  static const unsigned short KBridgeInfoSet = 1;
  static const unsigned short KBridgeInfoUnset = 0;
  struct HostCmd {
    //   bridge info command
    BridgeInfo bridge_info_cmd;
    // bridge info set flag, 1->set
    unsigned short bridge_info_set_flag;

    // save image control, 1->start
    unsigned short save_image_control;
  };

} // namespace ccr_split