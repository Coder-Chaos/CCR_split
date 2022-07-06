#include "maxon.hpp"

#define PRINT_INF 0

namespace ccr_split {
//maxon::maxon() {}
//maxon::~maxon() {}
/* -------------------------------NMT
 * control------------------------------------ */
void maxon::NMTstart(void) {
  usleep(kDelayEpos);
  can_frame nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kNMT;
  nmt_frame.can_dlc = 2;
  nmt_frame.data[0] = kNMT_Start_Node;
  nmt_frame.data[1] = 0;
  can_dev_->send(&nmt_frame);
  send_can(nmt_frame, *tcp_);
}

void maxon::NMTstart(__u8 slave_id) {
  usleep(kDelayEpos);
  can_frame nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kNMT;
  nmt_frame.can_dlc = 2;
  nmt_frame.data[0] = kNMT_Start_Node;
  nmt_frame.data[1] = slave_id;
  switch (slave_id) {
    case kUpClaw:
      can_dev_->send(&nmt_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      send_can(nmt_frame, *tcp_);
      break;

    case kMaxon:
      can_dev_->send(&nmt_frame);
      send_can(nmt_frame, *tcp_);
      break;

    default:
      break;
  }
}

void maxon::NMTPreOperation(__u8 slave_id) {
  usleep(kDelayEpos);
  can_frame nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kNMT;
  nmt_frame.can_dlc = 2;
  nmt_frame.data[0] = kNMT_Enter_PreOperational;
  nmt_frame.data[1] = 0;
  
  switch (slave_id) {
    case kUpClaw:
      can_dev_->send(&nmt_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      send_can(nmt_frame, *tcp_);
      break;

    case kMaxon:
      can_dev_->send(&nmt_frame);
      send_can(nmt_frame, *tcp_);
      break;

    default:
      break;
  }  
}

void maxon::NMTstop(__u8 slave_id) {
  can_frame nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kNMT;
  nmt_frame.can_dlc = 2;
  nmt_frame.data[0] = kNMT_Stop_Node;
  nmt_frame.data[1] = slave_id;
  
  switch (slave_id) {
    case kUpClaw:
      can_dev_->send(&nmt_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      send_can(nmt_frame, *tcp_);
      break;

    case kMaxon:
      can_dev_->send(&nmt_frame);
      send_can(nmt_frame, *tcp_);
      break;

    default:
      can_dev_->send(&nmt_frame);
      send_can(nmt_frame, *tcp_);
      break;
  }  
}

// cmd sync
void maxon::CmdSync() {
  can_frame nmt_frame;
  // nmt frame init
  nmt_frame.can_id = kSYNC;
  nmt_frame.can_dlc = 0;

  can_dev_->send(&nmt_frame);
}

/* -------------------TxPDO mapping------------------ */
// TxPDO4 mapping
void maxon::TxPDO4Remap(__u8 slave_id, __u32 object_value) {
  // enter preoperation state
  NMTPreOperation(slave_id);
  // clear past PDO mapping
  SdoWrU8(slave_id, 0x1603, 0x00, 0);
  // first new mapped object in RxPDO4, mode of operation
  SdoWrU32(slave_id, 0x1603, 0x01, object_value);

  // reset mapping object number, 1
  SdoWrU8(slave_id, 0x1603, 0x00, 1);

  // sleep(1);
}

// TxPDO1
ssize_t maxon::TxPdo1(__u8 slave_id, __u16 ctrl_wrd) {
  can_frame tx_pdo1_frame;
  // tx_pdo1 frame init
  tx_pdo1_frame.can_id = kPDO1rx + slave_id;
  tx_pdo1_frame.can_dlc = 2;
  tx_pdo1_frame.data[0] = ctrl_wrd & 0xff;
  tx_pdo1_frame.data[1] = (ctrl_wrd >> 8) & 0xff;
  if(slave_id == kUpClaw){
    can_dev_->send(&tx_pdo1_frame);
  } else if(slave_id > kUpClaw){
    send_can(tx_pdo1_frame, *tcp_);
  }
  return 0;
}

// TxPDO2
ssize_t maxon::TxPdo2(__u8 slave_id, __u16 ctrl_wrd, __s32 pos_sv) {
  can_frame tx_pdo2_frame;

  // tx_pdo2 frame init
  tx_pdo2_frame.can_id = kPDO2rx + slave_id;
  tx_pdo2_frame.can_dlc = 6;
  tx_pdo2_frame.data[0] = ctrl_wrd & 0xff;
  tx_pdo2_frame.data[1] = (ctrl_wrd >> 8) & 0xff;
  tx_pdo2_frame.data[2] = pos_sv & 0xff;
  tx_pdo2_frame.data[3] = (pos_sv >> 8) & 0xff;
  tx_pdo2_frame.data[4] = (pos_sv >> 16) & 0xff;
  tx_pdo2_frame.data[5] = (pos_sv >> 24) & 0xff;

  switch (slave_id) {
    case kUpClaw:
      return can_dev_->send(&tx_pdo2_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      return send_can(tx_pdo2_frame, *tcp_);
      break;

    default:
      break;
  }
}

ssize_t maxon::TxPdo2(__u8 slave_id, __u16 ctrl_wrd, __s32 pos_sv,
                      __u16 mode_of_operation) {
  can_frame tx_pdo2_frame;

  // tx_pdo2 frame init
  tx_pdo2_frame.can_id = kPDO2rx + slave_id;
  tx_pdo2_frame.can_dlc = 7;
  tx_pdo2_frame.data[0] = ctrl_wrd & 0xff;
  tx_pdo2_frame.data[1] = (ctrl_wrd >> 8) & 0xff;
  tx_pdo2_frame.data[2] = pos_sv & 0xff;
  tx_pdo2_frame.data[3] = (pos_sv >> 8) & 0xff;
  tx_pdo2_frame.data[4] = (pos_sv >> 16) & 0xff;
  tx_pdo2_frame.data[5] = (pos_sv >> 24) & 0xff;
  tx_pdo2_frame.data[6] = mode_of_operation;

  switch (slave_id) {
    case kUpClaw:
      return can_dev_->send(&tx_pdo2_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      return send_can(tx_pdo2_frame, *tcp_);
      break;

    default:
      break;
  }
}

// TxPDO3
ssize_t maxon::TxPdo3(__u8 slave_id, __s32 speed_set) {
  can_frame tx_pdo3_frame;

  // tx_pdo3 frame init
  tx_pdo3_frame.can_id = kPDO3rx + slave_id;
  tx_pdo3_frame.can_dlc = 4;
  tx_pdo3_frame.data[0] = speed_set & 0xff;
  tx_pdo3_frame.data[1] = (speed_set >> 8) & 0xff;
  tx_pdo3_frame.data[2] = (speed_set >> 16) & 0xff;
  tx_pdo3_frame.data[3] = (speed_set >> 24) & 0xff;
  
  switch (slave_id) {
    case kUpClaw:
      return can_dev_->send(&tx_pdo3_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      return send_can(tx_pdo3_frame, *tcp_);
      break;

    default:
      break;
  }
}

// TxPDO3 CST mode
ssize_t maxon::TxPdo3(__u8 slave_id, __s16 target_torque,
                      __u16 mode_of_operation) {
  can_frame tx_pdo3_frame;

  // tx_pdo4 frame init
  tx_pdo3_frame.can_id = kPDO3rx + slave_id;

  tx_pdo3_frame.can_dlc = 3;
  tx_pdo3_frame.data[0] = target_torque & 0xff;
  tx_pdo3_frame.data[1] = (target_torque >> 8) & 0xff;
  tx_pdo3_frame.data[2] = mode_of_operation;

  switch (slave_id) {
    case kUpClaw:
      return can_dev_->send(&tx_pdo3_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      return send_can(tx_pdo3_frame, *tcp_);
      break;

    default:
      break;
  }
}

// TxPDO4 operation mode set
ssize_t maxon::TxPdo4(__u8 slave_id, __s32 speed_set, __u16 mode_of_operation) {
  can_frame tx_pdo4_frame;

  // tx_pdo4 frame init
  tx_pdo4_frame.can_id = kPDO4rx + slave_id;
  tx_pdo4_frame.can_dlc = 5;
  tx_pdo4_frame.data[0] = speed_set & 0xff;
  tx_pdo4_frame.data[1] = (speed_set >> 8) & 0xff;
  tx_pdo4_frame.data[2] = (speed_set >> 16) & 0xff;
  tx_pdo4_frame.data[3] = (speed_set >> 24) & 0xff;
  tx_pdo4_frame.data[4] = mode_of_operation;

  switch (slave_id) {
    case kUpClaw:
      return can_dev_->send(&tx_pdo4_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      return send_can(tx_pdo4_frame, *tcp_);
      break;

    default:
      break;
  }
}

ssize_t maxon::TxPdo4(__u8 slave_id, __u16 mode_of_operation) {
  can_frame tx_pdo4_frame;

  // tx_pdo4 frame init
  tx_pdo4_frame.can_id = kPDO4rx + slave_id;
  tx_pdo4_frame.can_dlc = 1;
  tx_pdo4_frame.data[0] = mode_of_operation;

  switch (slave_id) {
    case kUpClaw:
      return can_dev_->send(&tx_pdo4_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      return send_can(tx_pdo4_frame, *tcp_);
      break;

    default:
      break;
  }
}

// TxPDO4 CST mode
ssize_t maxon::TxPdo4CST(__u8 slave_id, __u16 target_torque) {
  can_frame tx_pdo4_frame;

  // tx_pdo4 frame init
  tx_pdo4_frame.can_id = kPDO4rx + slave_id;

  tx_pdo4_frame.can_dlc = 2;
  tx_pdo4_frame.data[0] = target_torque & 0xff;
  tx_pdo4_frame.data[1] = (target_torque >> 8) & 0xff;

  switch (slave_id) {
    case kUpClaw:
      return can_dev_->send(&tx_pdo4_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      return send_can(tx_pdo4_frame, *tcp_);
      break;

    default:
      break;
  }
}

// TxRMD0读取、写入电机状态
ssize_t maxon::TxRMD0(__u8 slave_id, __u8 mode_of_operation) {
  can_frame tx_rmd0_frame;
  // tx_pdo4 frame init
  tx_rmd0_frame.can_id = RMD1rx + slave_id;
  tx_rmd0_frame.can_dlc = 8;
  tx_rmd0_frame.data[0] = mode_of_operation;
  tx_rmd0_frame.data[1] = 0x00;
  tx_rmd0_frame.data[2] = 0x00;
  tx_rmd0_frame.data[3] = 0x00;
  tx_rmd0_frame.data[4] = 0x00;
  tx_rmd0_frame.data[5] = 0x00;
  tx_rmd0_frame.data[6] = 0x00;
  tx_rmd0_frame.data[7] = 0x00;
  
  return send_can(tx_rmd0_frame, *tcp_);
}

// TxRMD1 torque mode
ssize_t maxon::TxRMD1(__u8 slave_id, __s16 target_torque) {
  can_frame tx_rmd1_frame;
  // tx_rmd1 frame init
  tx_rmd1_frame.can_id = RMD1rx + slave_id;
  tx_rmd1_frame.can_dlc = 8;
  tx_rmd1_frame.data[0] = 0xA1;
  tx_rmd1_frame.data[1] = 0x00;
  tx_rmd1_frame.data[2] = 0x00;
  tx_rmd1_frame.data[3] = 0x00;
  tx_rmd1_frame.data[4] = target_torque & 0xff;
  tx_rmd1_frame.data[5] = (target_torque >> 8) & 0xff;
  tx_rmd1_frame.data[6] = 0x00;
  tx_rmd1_frame.data[7] = 0x00;
  
  return send_can(tx_rmd1_frame, *tcp_);
}

// set rmd torque
__s8 maxon::SetRMDTorque(maxon_type *motor1, maxon_type *motor2,
                           __s16 torque1, __s16 torque2) {
  int i = 0;
  TxRMD1(motor1->motor_id, torque1);
  TxRMD1(motor2->motor_id, torque2);
  motor1->TrqPV = 0;
  motor2->TrqPV = 0;
  do{
    
    usleep(kDelayEpos);
    if(i%50 == 0){ 
      //printf("RMD%d torque%d;%d!\n", motor1->motor_id, motor1->TrqPV, motor2->TrqPV);
      TxRMD1(motor1->motor_id, torque1);
      TxRMD1(motor2->motor_id, torque2);}
    i ++;
   
    if(i > 10000){
      break;
#if PRINT_INF
      printf("loop%d-RMD torque break torque %d; %d!\n", i, motor1->TrqPV, motor2->TrqPV);
#endif
    }
  } while(abs(motor1->TrqPV - torque1) > 4 || abs(motor2->TrqPV - torque2) > 4 || 
  abs(motor1->actual_average_vel) > 100 || abs(motor2->actual_average_vel) > 100 );
#if PRINT_INF
  printf("loop%d-RMD%d&%d torque done!toq:%d,%d; %d,%d\n", i, motor1->motor_id, 
    motor2->motor_id, motor1->TrqPV, torque1, motor2->TrqPV, torque2); 
#endif 
  return kCfgSuccess;
}                

// TxRMD2 speed mode
ssize_t maxon::TxRMD2(__u8 slave_id, __s32 speed_set) {
  can_frame tx_rmd2_frame;
  // tx_rmd2 frame init
  tx_rmd2_frame.can_id = RMD1rx + slave_id;
  tx_rmd2_frame.can_dlc = 8;
  tx_rmd2_frame.data[0] = 0xA2;
  tx_rmd2_frame.data[1] = 0x00;
  tx_rmd2_frame.data[2] = 0x00;
  tx_rmd2_frame.data[3] = 0x00;
  tx_rmd2_frame.data[4] = speed_set & 0xff;
  tx_rmd2_frame.data[5] = (speed_set >> 8) & 0xff;
  tx_rmd2_frame.data[6] = (speed_set >> 16) & 0xff;
  tx_rmd2_frame.data[7] = (speed_set >> 24) & 0xff;
  
  return send_can(tx_rmd2_frame, *tcp_);
}

// set rmd speed
__s8 maxon::SetRMDSpeed(maxon_type *motor1, maxon_type *motor2,
                           __s32 speed1, __s32 speed2) {
  int i = 0;
  TxRMD2(motor1->motor_id, speed1);
  TxRMD2(motor2->motor_id, speed2);
  //TxRMD0(i, 0x92);
  do{
    usleep(kDelayEpos);
    i ++;
    if(i%50 == 0){
      TxRMD2(motor1->motor_id, speed1);
      TxRMD2(motor2->motor_id, speed2);}
    //TxRMD0(i, 0x92);
    //if(i % 1000 == 0){
      //
    //}
    if(i % 2000 == 0){
      printf("loop%d-RMD speed break%d, %d: %d, %d!\n", i, speed1, speed2, 
      motor1->actual_average_vel, motor2->actual_average_vel);
      break;
    }
  } while(abs(motor1->actual_average_vel - speed1/100) > 100
     || abs(motor2->actual_average_vel - speed2/100) > 100 );
#if PRINT_INF
  printf("loop%d-RMD%d speed%d: %d;%d!\n", i, motor1->motor_id, speed1/100, 
    motor1->actual_average_vel, motor2->actual_average_vel);
#endif
  return kCfgSuccess;
}                           

// TxRMD3 relative position mode
ssize_t maxon::TxRMD3(__u8 slave_id, __s32 pos_sv) {
  can_frame tx_rmd3_frame;
  // tx_pdo4 frame init
  tx_rmd3_frame.can_id = RMD1rx + slave_id;
  tx_rmd3_frame.can_dlc = 8;
  tx_rmd3_frame.data[0] = 0xA7;
  tx_rmd3_frame.data[1] = 0x00;
  tx_rmd3_frame.data[2] = 0x00;
  tx_rmd3_frame.data[3] = 0x00;
  tx_rmd3_frame.data[4] = pos_sv & 0xff;
  tx_rmd3_frame.data[5] = (pos_sv >> 8) & 0xff;
  tx_rmd3_frame.data[6] = (pos_sv >> 16) & 0xff;
  tx_rmd3_frame.data[7] = (pos_sv >> 24) & 0xff;

  return send_can(tx_rmd3_frame, *tcp_);;
}

// set rmd position
__s8 maxon::SetRMDPos(maxon_type *motor1, maxon_type *motor2,
                           __s32 pos1, __s32 pos2) {
  int i = 0;
  __s32 pos1_sp = pos1;
  __s32 pos2_sp = pos2;
  TxRMD3(motor1->motor_id, pos1);
  TxRMD3(motor2->motor_id, pos2);
  
  do{
    usleep(kDelayEpos);
    if(i%20 == 0){
      TxRMD3(motor1->motor_id, pos1);
      TxRMD3(motor2->motor_id, pos2);}
    
    i ++;
    if(i % 1000 == 0){
      //printf("loop%d-RMD position\n", i);
    }
    if(i % 2000 == 0){
      printf("SetPos break!Vel=%d\n", motor1->actual_average_vel);
      break;
    }
  } while(abs(motor1->actual_average_vel) > 100 || abs(motor2->actual_average_vel) > 100);
#if PRINT_INF
  printf("RMD%d&%d pos done;pos %d, %d: %d, %d!\n", motor1->motor_id, motor2->motor_id, 
    pos1_sp, pos2_sp, motor1->PosPV, motor2->PosPV);
#endif
  return kCfgSuccess;
} 

// sdo write 8bit
ssize_t maxon::SdoWrU8(__u8 slave_id, __u16 index, __u8 subindex, __u32 data) {
  usleep(kDelayEpos);
  can_frame sdo_rx_frame;
  sdo_rx_frame.can_id = kSDOrx + slave_id;
  sdo_rx_frame.can_dlc = 8;
  sdo_rx_frame.data[0] = 0x2F;
  sdo_rx_frame.data[1] = index & 0xff;
  sdo_rx_frame.data[2] = (index >> 8) & 0xff;
  sdo_rx_frame.data[3] = subindex;
  sdo_rx_frame.data[4] = data & 0xff;
  sdo_rx_frame.data[5] = 0;
  sdo_rx_frame.data[6] = 0;
  sdo_rx_frame.data[7] = 0;

  switch (slave_id) {
    case kUpClaw:
      return can_dev_->send(&sdo_rx_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      return send_can(sdo_rx_frame, *tcp_);
      break;

    default:
      break;
  }
}

// sdo write 16bit
ssize_t maxon::SdoWrU16(__u8 slave_id, __u16 index, __u8 subindex, __u32 data) {
  usleep(kDelayEpos);
  can_frame sdo_rx_frame;
  sdo_rx_frame.can_id = kSDOrx + slave_id;
  sdo_rx_frame.can_dlc = 8;
  sdo_rx_frame.data[0] = 0x2B;
  sdo_rx_frame.data[1] = index & 0xff;
  sdo_rx_frame.data[2] = (index >> 8) & 0xff;
  sdo_rx_frame.data[3] = subindex;
  sdo_rx_frame.data[4] = data & 0xff;
  sdo_rx_frame.data[5] = (data >> 8) & 0xff;
  sdo_rx_frame.data[6] = 0;
  sdo_rx_frame.data[7] = 0;
  //sdo_rx_frame.data[7] = 0;

  switch (slave_id) {
    case kUpClaw:
      return can_dev_->send(&sdo_rx_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      return send_can(sdo_rx_frame, *tcp_);
      break;

    default:
      break;
  }
}

// sdo write 32bit
ssize_t maxon::SdoWrU32(__u8 slave_id, __u16 index, __u8 subindex, __u32 data) {
  usleep(kDelayEpos);
  can_frame sdo_rx_frame;
  sdo_rx_frame.can_id = kSDOrx + slave_id;
  sdo_rx_frame.can_dlc = 8;
  sdo_rx_frame.data[0] = 0x23;
  sdo_rx_frame.data[1] = index & 0xff;
  sdo_rx_frame.data[2] = (index >> 8) & 0xff;
  sdo_rx_frame.data[3] = subindex;
  sdo_rx_frame.data[4] = data & 0xff;
  sdo_rx_frame.data[5] = (data >> 8) & 0xff;
  sdo_rx_frame.data[6] = (data >> 16) & 0xff;
  sdo_rx_frame.data[7] = (data >> 24) & 0xff;

  switch (slave_id) {
    case kUpClaw:
      return can_dev_->send(&sdo_rx_frame);
      break;

    case kPulley1:
    case kPulley2:
    case kDownClaw1:
    case kDownClaw2:
      return send_can(sdo_rx_frame, *tcp_);
      break;

    default:
      break;
  }
}

/*
 *Motor control
 */

ssize_t maxon::SetCtrlWrd(__u8 slave_id, __u16 ctrl_wrd) {
  return TxPdo1(slave_id, ctrl_wrd);
}

// enable motor
void maxon::MotorEnable(__u8 slave_id) {
  SetCtrlWrd(slave_id, 0x0006);
  usleep(kDelayEpos);
  SetCtrlWrd(slave_id, 0x000F);
  usleep(kDelayEpos);
}

// enable motor
__s8 maxon::MotorEnable() {
  int i;
  SetCtrlWrd(param_->motor_id, 0x0006);

  while ((param_->StatusWord >> 5 & 1) != 1) {
    i ++;
    // must delay!
    if(i % 30 == 0){
      SetCtrlWrd(param_->motor_id, 0x0006);
    }
    if(i % 300 == 0){
      MotorReset();
    }
    
    if(i % 2002 == 0){
      printf("motor%d 06break StatusWord: 0x%x\n", param_->motor_id, 
        param_->StatusWord);
      //MotorReset();
      break;
    }
    usleep(1000);
  }
  //printf("loop%d-motor%d 06StatusWord: 0x%x\n", i, param_->motor_id,
  //   param_->StatusWord);
  
  i = 0;
  SetCtrlWrd(param_->motor_id, 0x000F);
  usleep(kDelayEpos);
  // wait enable cmd success
  while ((param_->StatusWord & 0x00ff) != 0x37) {
    
    if(i % 60 == 0){
      SetCtrlWrd(param_->motor_id, 0x000F);
    }
    i ++;
    // must delay!
    usleep(1000);
    if(i % 2002 == 0){
      printf("motor%d 0fbreak\n", param_->motor_id);
      MotorReset();
      //break;
    }
    
  }
#if PRINT_INF
  printf("loop%d-motor%d enable done!\n", i, param_->motor_id);
#endif
  return kCfgSuccess;
}

void maxon::MotorDisable(__u8 slave_id) { SetCtrlWrd(slave_id, 0x0000); }

__s8 maxon::MotorDisable() {
  int i;
  SetCtrlWrd(param_->motor_id, 0x0000);
  // wait disable cmd success
  while (param_->StatusWord != 0x0240) {
    i ++;
    if(i % 20 == 0){
      SetCtrlWrd(param_->motor_id, 0x0000);
    }
    usleep(1000);
    
    if(i % 4048 == 0){
      printf("motor%d 00break20000\n", param_->motor_id);
      MotorReset();
      break;
      //break;
    }
  }
  //printf("loop%d-motor%d disable done!\n", i, param_->motor_id);
  return kCfgSuccess;
}

// reset motor
void maxon::MotorReset(){
  
  switch (param_->motor_id) {
    case kUpClaw:
    case kDownClaw1:
    case kDownClaw2:
      SdoWrU16(param_->motor_id, 0x6040, 0x00, 0x0080);
      break;

    case kPulley1:
    case kPulley2:
      TxRMD0(param_->motor_id, 0x9B);
      break;

    default:
      break;
  }
  
}

// init maxon
void maxon::Init(){
  
  NMTstart(param_->motor_id);
  MotorEnable(param_->motor_id);
  MotorDisable(param_->motor_id);

}

ssize_t maxon::SetMotorAbsPos(__u8 slave_id, __s32 abs_pos) {
  usleep(kDelayEpos);
  return TxPdo2(slave_id, kServAbsPosSet, abs_pos);
}

__s8 maxon::SetMotorAbsPos(__s32 abs_pos) {
  __s32 init_cfg_pos;
  // save the init pos before configuration
  init_cfg_pos = param_->PosPV;

  int i = 0;
  do {
    if(i % 20 == 0){
      TxPdo2(param_->motor_id, kServAbsPosSet, abs_pos, 0x01);
      SetCtrlWrd(param_->motor_id, 0x000F);
    }
    usleep(kDelayEpos);
    i ++;
    if( i % 10000 == 0){
      printf("motor%d pos break.\n", param_->motor_id);
      //break;
    }
  } while(abs(param_->PosPV - abs_pos) > 200); 

  //printf("loop%d-motor%d current pos:%d,target pos:%d\n", i,
  //  param_->motor_id, param_->PosPV, abs_pos);
  if(i > 10000){
    return kCfgFail;
  } else{
    return kCfgSuccess;
  }
}

__s8 maxon::SetMotorAbsPos(maxon_type *motor1, maxon_type *motor2,
                           __s32 abs_pos1, __s32 abs_pos2) {
  __s32 init_cfg_pos1, init_cfg_pos2;
  // save the init pos before configuration
  init_cfg_pos1 = motor1->PosPV;
  init_cfg_pos2 = motor2->PosPV;
  
  TxPdo2(motor1->motor_id, kServAbsPosSet, abs_pos1, 0x01);
  TxPdo2(motor2->motor_id, kServAbsPosSet, abs_pos2, 0x01);
  int i = 0;
  while (motor1->mode_display != 0x01 || motor2->mode_display != 0x01 
    || abs(motor1->PosPV - init_cfg_pos1) < 10000 || 
    abs(motor2->PosPV - init_cfg_pos2) < 10000)
		{
			usleep(kDelayEpos*2);
			TxPdo2(motor1->motor_id, kServAbsPosSet, abs_pos1, 0x01);
      TxPdo2(motor2->motor_id, kServAbsPosSet, abs_pos2, 0x01);
      SetCtrlWrd(motor1->motor_id, 0x000F);
      SetCtrlWrd(motor2->motor_id, 0x000F);
      //printf("motor pos init\n");
      i++;
      /*if(i % 100000){
        SdoWrU16(motor1->motor_id, 0x6040, 0x00, 0x0080);
        SdoWrU16(motor2->motor_id, 0x6040, 0x00, 0x0080);
        printf("motor pos init reset\n");
        
        MotorEnable(motor1->motor_id);
        MotorEnable(motor2->motor_id);
        break;
      }*/
		}
  //printf("motor%d&%d  pos init: %d, %d\n", motor1->motor_id, 
  //  motor2->motor_id, init_cfg_pos1, init_cfg_pos2);
  i = 0;
  do {
    i ++;
    
    usleep(kDelayEpos);
    i ++;
    if( i % 3000 == 0){
      printf("loop%d-motor%d&%d  pos done: %d, %d; %d, %d\n", i, 
      motor1->motor_id, motor2->motor_id, motor1->PosPV, abs_pos1, 
      motor2->PosPV, abs_pos2);
      
      }
  } while(abs(motor1->PosPV - abs_pos1) > 500 ||
      abs(motor2->PosPV - abs_pos2) > 500);
  
#if PRINT_INF
  printf("loop%d-motor%d&%d  pos done: %d, %d; %d, %d\n", i, 
    motor1->motor_id, motor2->motor_id, motor1->PosPV, abs_pos1, 
    motor2->PosPV, abs_pos2);
#endif
  
  //if(i > 3000){
  //  return kCfgFail;
  //} else{
    return kCfgSuccess;
  //}
}

__s8 maxon::SetMotorAbsPos(maxon_type *motor1, maxon_type *motor2,
                          maxon_type *motor3, __s32 abs_pos1,
                           __s32 abs_pos2, __s32 abs_pos3) {
  TxPdo2(motor1->motor_id, kServAbsPosSet, abs_pos1, 0x01);
  TxPdo2(motor2->motor_id, kServAbsPosSet, abs_pos2, 0x01);
  TxPdo2(motor3->motor_id, kServAbsPosSet, abs_pos2, 0x01);
  usleep(kDelayEpos);

  // wait the abs pos reach the target, 1000inc error
  while (motor1->mode_display != 0x01 || abs(motor1->PosPV - abs_pos1) > 1000 ||
         abs(motor2->PosPV - abs_pos2) > 1000 || motor2->mode_display != 0x01 ||
         motor3->mode_display != 0x01 || abs(motor3->PosPV - abs_pos3) > 1000) {
    printf("configuring!\n");

    usleep(kDelayEpos);
    TxPdo2(motor1->motor_id, kServAbsPosSet, abs_pos1, 0x01);
    SetCtrlWrd(motor1->motor_id, 0x000F);
    TxPdo2(motor2->motor_id, kServAbsPosSet, abs_pos2, 0x01);
    SetCtrlWrd(motor2->motor_id, 0x000F);
    TxPdo2(motor3->motor_id, kServAbsPosSet, abs_pos3, 0x01);
    SetCtrlWrd(motor3->motor_id, 0x000F);
  }
  return kCfgSuccess;
}

ssize_t maxon::SetMotorRelPos(__u8 slave_id, __s32 relative_pos) {
  return TxPdo2(slave_id, kServRelPosSet, relative_pos);
}

__s8 maxon::SetMotorRelPos( __s32 relative_pos) {
  // save init pos
  param_->init_pos = param_->PosPV;

  TxPdo2(param_->motor_id, kServRelPosSet, relative_pos, 0x01);
  usleep(kDelayEpos);
  // wait the rel pos reach the target, 1000inc error
  while (param_->mode_display != 0x01 ||
         abs(param_->PosPV - (param_->init_pos + relative_pos)) > 1000) {
    param_->delta_pos = param_->PosPV - param_->init_pos;
    printf("error:%d inc, motor %d relative pos not reached!\n",
           abs(param_->PosPV - (param_->init_pos + relative_pos)),
           param_->motor_id);
    usleep(kDelayEpos);
  }

  return kCfgSuccess;
}

ssize_t maxon::SetMotorSpeed(__u8 slave_id, __s32 speed_set) {
  return TxPdo3(slave_id, speed_set);
}

// set speed
__s8 maxon::SetMotorSpeed(__s32 speed) {
  // set
  TxPdo4(param_->motor_id, speed, 0x03);
  usleep(kDelayEpos);
  // wait the speed reach 5% of the target speed
  while (param_->mode_display != 0x03 ||
         abs(param_->actual_average_vel) < abs(speed * 0.05)) {
    // 
    // printf("current speed:%d\n", motor->SpdPV);
    // printf("target speed:%d\n", speed);
    TxPdo4(param_->motor_id, speed, 0x03);
    SetCtrlWrd(param_->motor_id, 0x000F);
    usleep(kDelayEpos);
  }

  return kCfgSuccess;
}

// set torque
ssize_t maxon::SetTargetTorque(__u8 slave_id, __s16 target_torque) {
  return TxPdo3(slave_id, target_torque, 0x0A);
}

__s8 maxon::SetTargetTorque(__s16 target_torque) {
  int i = 0;
  TxPdo3(param_->motor_id, target_torque, 0x0A);

  // wait the toruqe reach the target, 1% error
  while (param_->mode_display != 0x0A ||
    abs(param_->actual_average_torque - target_torque) > 20 
    || abs(param_->actual_average_vel) > 200) {
    i ++;
    usleep(kDelayEpos);
    if( i % 3 == 0){
      TxPdo3(param_->motor_id, target_torque, 0x0A);}
    
    if( i % 1024 == 0){
      printf("motor%d torque%d vel%d break&reset!\n", param_->motor_id, param_->actual_average_torque, 
        param_->actual_average_vel);
      //SdoWrU16(param_->motor_id, 0x6040, 0x00, 0x0080);
      break;
    }
  }
#if PRINT_INF
  printf("loop%d-motor%d torque configure done!\n", i, param_->motor_id);
#endif  
  return kCfgSuccess;  

}

__s8 maxon::SetTargetTorque(maxon_type *motor1, maxon_type *motor2,
                           __s16 torque1, __s16 torque2) {
  int i = 0;
  TxPdo3(motor1->motor_id, torque1, 0x0A);
  TxPdo3(motor2->motor_id, torque2, 0x0A);

  // wait the toruqe reach the target, 1% error
  while (motor1->mode_display != 0x0A || motor2->mode_display != 0x0A
        || abs(motor1->actual_average_torque - torque1) > 20 || abs(
        motor2->actual_average_torque - torque2) > 20 || abs(motor1->
        actual_average_vel) > 1000 || abs(motor2->actual_average_vel) > 1000) {
    if( i % 200 == 0){
      TxPdo3(motor1->motor_id, torque1, 0x0A);
      TxPdo3(motor2->motor_id, torque2, 0x0A);
    }
    usleep(kDelayEpos);
    i ++;
    if( i % 1000 == 0){  
      printf("motor%d&%d torque break Torq %d, %d; vel %d,%d!\n", motor1->motor_id, motor2->motor_id, 
        motor1->actual_average_torque, motor2->actual_average_torque, motor1->
        actual_average_vel, motor2->actual_average_vel);
      //break;
    }
  }
  //printf("loop%d-motor%d&%d torque configure done! Vel %d, %d\n", i, motor1->motor_id, 
  //     motor2->motor_id, motor1->actual_average_vel, motor2->actual_average_vel);
  return kCfgSuccess;
}

// set motor operation mode
ssize_t maxon::SetMotorMode(__u8 slave_id, __u16 operation_mode) {
  return TxPdo4(slave_id, operation_mode);
}

void maxon::MotorParaRead(__u16 cob_id, maxon_type *motor,
                          can_frame *recv_frame) {
  switch (cob_id) {
      // 0x180
    case kPDO1tx:
      motor->StatusWord =
          (__u16)(recv_frame->data[1] << 8) | recv_frame->data[0];
      motor->TrqPV = (__s16)((recv_frame->data[3] << 8) | recv_frame->data[2]);
      motor->PosPV =
          (__s32)((recv_frame->data[7] << 24) | (recv_frame->data[6] << 16) |
                  (recv_frame->data[5] << 8) | recv_frame->data[4]);
      break;

      // 0x280
    case kPDO2tx:
      motor->StatusWord = (recv_frame->data[1] << 8) | recv_frame->data[0];
      motor->TrqPV = (__s16)((recv_frame->data[3] << 8) | recv_frame->data[2]);
      motor->SpdPV =
          (__s32)((recv_frame->data[7] << 24) | (recv_frame->data[6] << 16) |
                  (recv_frame->data[5] << 8) | recv_frame->data[4]);
      break;

      // 0x380
    case kPDO3tx:
      motor->actual_average_vel =
          (__s32)((recv_frame->data[3] << 24) | recv_frame->data[2] << 16 |
                  (recv_frame->data[1] << 8) | recv_frame->data[0]);
      motor->actual_average_torque =
          (__s16)((recv_frame->data[5] << 8) | recv_frame->data[4]);
      break;

      // 0x480
    case kPDO4tx:
      motor->StatusWord =
          (__u16)(recv_frame->data[1] << 8) | recv_frame->data[0];
      motor->ServErr =
          (__u16)((recv_frame->data[3] << 8) | recv_frame->data[2]);
      motor->TrqPV = (__s16)((recv_frame->data[5] << 8) | recv_frame->data[4]);
      motor->mode_display = recv_frame->data[6];
      break;

      // 0x140
    case RMD1tx:
      //printf("RMD1tx%x\n", RMD1tx);
      motor->StatusWord = (int8_t)(recv_frame->data[0]);
      if(motor->StatusWord == 0x92){
        motor->PosPV_rmd = (__s64)((recv_frame->data[7] << 48) | (recv_frame->data[6] << 40) | (recv_frame->data[5] << 32) | 
          (recv_frame->data[4] << 24) | (recv_frame->data[3] << 16) | (recv_frame->data[2] << 8) | (recv_frame->data[1]));
      } else{
        motor->TrqPV = (int16_t)((recv_frame->data[3] << 8) | recv_frame->data[2]);
        motor->actual_average_vel = (int16_t)((recv_frame->data[5] << 8) | recv_frame->data[4]);
        motor->PosPV = (__u16)((recv_frame->data[7] << 8) | (recv_frame->data[6]));
      }
      
      //printf("actual_average_vel%d\n", motor->actual_average_vel);
      break;    

    default:
      break;
  }
}

// move to relative position
void maxon::MoveRelative(__u8 slave_id, __s32 relative_pos) {
  SetMotorRelPos(slave_id, relative_pos);
  usleep(kDelayEpos);
  SetCtrlWrd(slave_id, 0x000F);
  usleep(kDelayEpos);
}

// move to relative position, 2 motors
void maxon::MoveRelative(__u8 slave_id1, __u8 slave_id2, __s32 relative_pos) {
  // enable motor1
  MotorEnable(slave_id1);

  // enable motor2
  MotorEnable(slave_id2);

  SetMotorRelPos(slave_id1, relative_pos);

  SetMotorRelPos(slave_id2, relative_pos);

  SetCtrlWrd(slave_id1, 0x000F);

  SetCtrlWrd(slave_id2, 0x000F);
}

// move to absolute position
void maxon::MoveAbsolute(__u8 slave_id, __s32 absolute_pos) {
  // wait epos
  usleep(kDelayEpos);
  SetMotorRelPos(slave_id, absolute_pos);
  usleep(kDelayEpos);
  SetCtrlWrd(slave_id, 0x000F);
}

// quick stop motor
void maxon::MotorQuickStop(__u8 slave_id) {
  // wait epos
  usleep(kDelayEpos);
  SetCtrlWrd(slave_id, 0x000B);
}

// usleep
//void maxon::usleep(__u32 us) { usleep(us); }

// 2 motor
__s8 maxon::ChangeToTorqueMode(maxon_type *motor1,
                              maxon_type *motor2) {
  // disable pulleys
  MotorDisable(motor1->motor_id);
  MotorDisable(motor2->motor_id);

  // sleep(1);

  // change to CST mode
  SetMotorMode(motor1->motor_id, 0x0A);
  SetMotorMode(motor2->motor_id, 0x0A);

  // sleep(1);

  // remap pulley1 TxPDO4 to target torque
  TxPDO4Remap(motor2->motor_id, kOBJTargetTorque);
  TxPDO4Remap(motor1->motor_id, kOBJTargetTorque);

  // node enter normal mode
  NMTstart(motor1->motor_id);
  NMTstart(motor2->motor_id);

  if (motor1->mode_display == 0x0A && motor2->mode_display == 0x0A) {
    return kCfgSuccess;
  } else {
    return kCfgFail;
  }
}

// one motor
void maxon::ChangeToTorqueMode(__u8 slave_id) {
  // disable pulleys
  MotorDisable(slave_id);

  // change to CST mode
  SetMotorMode(slave_id, 0x0A);

  // remap TxPDO4 to target torque
  TxPDO4Remap(slave_id, kOBJTargetTorque);

  // node enter normal mode
  NMTstart(slave_id);
}

// change to torque mode
__s8 maxon::ChangeToTorqueMode(maxon_type *motor) {
  // change to CST mode
  SetMotorMode(motor->motor_id, 0x0A);
  // wait for epos
  // usleep(kDelayEpos);
  // if (motor->mode_display == 0x0A)
  // {
  //     return kCfgSuccess;
  // }
  // else
  // {
  //     return kCfgFail;
  // }
  while (motor->mode_display != 0x0A) {
    usleep(1000);
  }
  return kCfgSuccess;
}

// one motor
void maxon::ChangeToPositionMode(__u8 slave_id) {
  // disable motor
  MotorDisable(slave_id);

  // remap TxPdo4 to mode of operation
  TxPDO4Remap(slave_id, kOBJModeOfOperation);

  // restart node
  NMTstart(slave_id);

  // change to PPM mode;
  SetMotorMode(slave_id, 0x01);
}

// change to positionmode
__s8 maxon::ChangeToPositionMode(maxon_type *motor) {
  // change to PPM mode;
  SetMotorMode(motor->motor_id, 0x01);
  // wait for EPOS response
  // usleep(kDelayEpos);
  // if (motor->mode_display == 0x01)
  // {
  //     return kCfgSuccess;
  // }
  // else
  // {
  //     return kCfgFail;
  // }
  while (motor->mode_display != 0x01) {
    usleep(1000);
  }

  return kCfgSuccess;
}

// two motor
void maxon::ChangeToPositionMode(__u8 slave_id1, __u8 slave_id2) {
  // disable motor
  MotorDisable(slave_id1);
  MotorDisable(slave_id2);

  // remap TxPdo4 to mode of operation
  TxPDO4Remap(slave_id1, kOBJModeOfOperation);
  TxPDO4Remap(slave_id2, kOBJModeOfOperation);

  // restart node
  NMTstart(slave_id1);
  NMTstart(slave_id2);

  // change to PPM mode;
  SetMotorMode(slave_id1, 0x01);
  SetMotorMode(slave_id2, 0x01);
}



}
