#include "robot.hpp"
#include "log.hpp"
#include <chrono>
#define SAVE_UVC_PIC 0

namespace ccr_split
{
  // folder name
  static std::string uvc_folder_name;

  void Robot::Guard()
  {
    uint16_t heart_beat_last, heart_beat_this;
    log_info("start listen host.");
    // heart_beat_flag_ = true;
    heart_beat_last = utility::ChangeEndian(motion_.motion_cmd_->time_stamp.second);
    
    guard_thread_ = std::thread([&]()
                                {
                                  for (;;)
                                  {
                                    // listen every 30s
                                    std::this_thread::sleep_for(std::chrono::seconds(30));
                                    heart_beat_this = utility::ChangeEndian(motion_.motion_cmd_->time_stamp.second);
                                    // check heart beat
                                    if (heart_beat_last == heart_beat_this)
                                    {
                                      //log_error("I'm lost!!!!!");
                                      //log_error("I'm lost!!!!!");
                                      // set lost flag
                                      heart_beat_flag_ = false;
                                    }
                                    heart_beat_last = heart_beat_this;
                                  }
                                });
    guard_thread_.detach();
    
  }

  static void Piphandler(int sig)
  {
    if (SIGPIPE == sig)
    {
      log_error("pipe broken.");
    }
  }

  void
  Robot::ConnectHost()
  {
    log_info("try to connect host.");
    signal(SIGPIPE, Piphandler);
    // init modbus tcp
    if (eMBTCPInit(MODBUS_TCP_PORT) != MB_ENOERR)
    {
      log_error("Can't Initialize Modbus Stack!");
      log_error("Exit program!");
      // disable modbus stack
      (void)eMBDisable();

      // close modbus connection
      (void)eMBClose();
      exit(-1);
    }
    // enable protocal
    if (eMBEnable() != MB_ENOERR)
    {
      log_error("Can't enable modbus protocal!");
      exit(-1);
    }

    /* modbus poll thread */
    modbus_thread_ = std::thread([&]()
                                 {
                                   log_info("modbus start polling.");
                                   heart_beat_flag_ = true;
                                   while (false == disconnect_host_signal_)
                                   {
                                     if (eMBPoll() == MB_ENOERR)
                                     {
                                       // refresh rate 50ms
                                       usleep(50000);
                                     }
                                     else
                                     {
                                       log_error("Modbus Poll Error!");
                                       break;
                                     }
                                   }
                                   log_debug("exit modbus polling.");

#if 1
                                   // disable modbus stack
                                   (void)eMBDisable();

                                   // close modbus connection
                                   (void)eMBClose();
#endif
                                 });
  }

  void Robot::DisConnectHost()
  {
    log_info("disconnect host.");
    disconnect_host_signal_ = true;
    log_debug("wait modbus thread finish.");
    modbus_thread_.detach();

    log_info("host disconnected.");
  }

  void Robot::PMhvRecv()
  {
    //TFmini Up
    std::string id1 = "PMhv";
    std::string portName1 = "/dev/i2c-8";
    pmhv_obj = new cuav::PMhv(portName1);

    int i;
    
    /* pmhv recv thread */
    pmhv_thread_ = std::thread([&]()
                                 {
                                   //
                                   while (false == stop_pmhv_signal_ )
                                   {
                                     //printf("pmhv_thread_ start\n");
                                     
                                     if(pmhv_obj->getCurr() > 0)
                                      pm_curr_ = pmhv_obj->dataCurr[0];                                                                           
                                      
                                      if(pmhv_obj->getVolt() > 0)
                                      {
                                        pm_vlot_[0] = pmhv_obj->dataVolt[0];
                                        pm_vlot_[1] = pmhv_obj->dataVolt[1];                          
                                        if(pm_vlot_[1] < 0.2f)
                                          printf("Warning:volt=%2.2f.\n", pm_vlot_[1]);
                                      }
                                      else if(pmhv_obj->getCurr() == -1.0)
                                      {
                                        log_error("PMhv failed to read data!");
                                        break;
                                      }
                                      if(i % 1000 == 0)
                                       printf("volt=%2.2f/%2.2f%%;curr=%2.2f.\n", pm_vlot_[0], pm_vlot_[1], pm_curr_); 
                                                                        
                                      usleep(10000);
                                      i++;
                                    }
                                   
                                  });
    
  }

  void Robot::PMhvStop()
  {
    //log_info("TF02 Stop1.");
    stop_pmhv_signal_ = true;
    
    pmhv_thread_.join();
    //tf03_thread_.detach();
    pmhv_obj->closePort();
    log_info("PMhv Stop.");
  }


  void Robot::TFminiRecv()
  {
    //TFmini Up
    float dist1 = 0;
    float dist1_old = 0;
    float _dist1 = 0;
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
    bool first_tf_up = false;

    /* tfmini recv thread */
    tfmini_thread_ = std::thread([&]()
                                 {
                                   //
                                   while (false == stop_tfmini_signal_ )
                                   {
                                     //printf("tfmini_thread_ start\n");
                                      dist1 = tfmini_obj->getDist();
                                      dist1_stop = 3.5f;
                                      if(i % 1000 == 0)
                                       printf("odometer_up=%4.2fm; dist_up=%4.2fm.\n", odometer_up, dist1);
                                      if(dist1 > 0 && dist1 < Range_Up->max_range)
                                      {
                                        if(!first_tf_up){
                                         dist1_old = dist1;
                                         _dist1 = dist1;
                                         first_tf_up =true;
                                        }
                                        Range_Up->range = dist1; 
                                        motion_.odom_Up = dist1;
                                        if(i % 200 == 0){
                                          odometer_up += fabs(dist1 - _dist1);
                                          _dist1 = dist1;
                                        }
                                                           
                                        if((dist1 < 0.7f) && (dist1 < dist1_old) && (motion_.motion_cmd_->command 
                                          == kMoveUpCmd || motion_.motion_cmd_->command == pMoveUpCmd || 
                                          motion_.motion_cmd_->motion_mode == kTestMode  || 
                                        motion_.motion_cmd_->motion_mode == kAutoMode)){                                 
                                            //motion_.motion_cmd_->command = kStopCmd;
                                            motion_.motion_cmd_->quick_stop = kQuickStopOn;                                                                                                                         
                                        }
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
                                   
                                  });
    
  }

  void Robot::TFminiStop()
  {
    //log_info("TF02 Stop1.");
    stop_tfmini_signal_ = true;
    
    tfmini_thread_.join();
    //tf03_thread_.detach();
    tfmini_obj->closePort();
    log_info("TFmini Stop.");
  }


  void Robot::TF03Recv()
  {
    //TF03 Middle
    float dist = 0;
    float dist_old = 0;
    float dist_stop = 0;
    std::string id = "TF03";
    std::string portName = "/dev/TF03";
    int baud_rate = 115200;
    tf03_obj = new benewake::TFmini(portName, baud_rate);     
    Range_->radiation_type = INFRARED;
    Range_->field_of_view = 0.04f;
    Range_->min_range = 0.1f;
    Range_->max_range = 50.0f;   
    Range_->frame_id = id;

    int i;
    
    /* tfmini recv thread */
    tf03_thread_ = std::thread([&]()
                                 {
                                   //
                                   while (false == stop_tf03_signal_ )
                                   {
                                     //printf("tf03_thread_ start\n");
                                     dist = tf03_obj->getDist();                                    
                                     dist_stop = 3.5f;
                                     if(i % 1001 == 0)
                                      printf("\rdist_mid=%4.2fm.\n", dist);
                                     if(dist > 0 && dist < Range_->max_range)
                                     {
                                       Range_->range = dist;
                                       motion_.odom_Mid = dist;
                                       //Range_->stamp = ros::Time::now();                         
                                       if((dist < 0.6f) && (dist < dist_old) && (motion_.motion_cmd_->command 
                                        == pMoveDownCmd || motion_.motion_cmd_->command == mMoveUpCmd || 
                                        motion_.motion_cmd_->motion_mode == kTestMode  || 
                                        motion_.motion_cmd_->motion_mode == kAutoMode)){
                                  
                                         //motion_.motion_cmd_->command = kStopCmd;
                                         motion_.motion_cmd_->quick_stop = kQuickStopOn;
                                       }
                                       /*if((dist > (dist_stop-0.2)) && (dist > dist_old) && (motion_.motion_cmd_->command 
                                       == mMoveDownCmd)){
                                         motion_.motion_cmd_->command = kStopCmd;
                                         printf("dist=%4.2f;dist_stop=%4.2f",dist, dist_stop);
                                       }*/

                                        dist_old = dist;
                                     }
                                     else if(dist == -1.0)
                                     {
                                       log_error("TF03 failed to read data!");
                                       break;
                                     }
                                     else if(dist == 0.0)
                                     {
                                       log_error("TF03 data 0.0 error!");
                                     }

                                     usleep(5000);
                                     i++;
                                   }
                                   
                                  });
  }

  void Robot::TF03Stop()
  {
    //log_info("TF03 Stop1.");
    stop_tf03_signal_ = true;
    
    tf03_thread_.join();
    //tf03_thread_.detach();
    tf03_obj->closePort();
    log_info("TF03 Stop.");
    
  }

  void Robot::TF02Recv()
  {
    //TF02 Down
    float dist2 = 0;
    float dist2_old = 0;
    float _dist2 = 0;
    float dist2_stop = 0;
    std::string id = "TF02";
    can_frame recv_tf02;
    Range_Down->radiation_type = INFRARED;
    Range_Down->field_of_view = 0.04f;
    Range_Down->min_range = 0.1f;
    Range_Down->max_range = 22.0f;   
    Range_Down->frame_id = id;

    int i;
    bool first_tf_down = false;
    
    /* tfmini recv thread */
    tf02_thread_ = std::thread([&]()
                                 {
                                   //
                                   while (false == stop_tf02_signal_ )
                                   {
                                     if(recv_tf(recv_tf02, tcp_tf02_) > 0){
                                      dist2 = (float)(recv_tf02.data[0] << 8 | recv_tf02.data[1]) / 100.0;
                                      //printf("\rdist=%4.2f; max=%4.2f.\n", dist2, Range_->max_range);
                                     }
                                
                                     dist2_stop = 3.5f;
                                     if(i % 1000 == 0)
                                      //printf("can_id:0x%x\n", recv_tf02.can_id);
                                      printf("\rodometer_down=%4.2fm; dist_down=%4.2fm.\n", odometer_down, dist2);
                                     if(dist2 > 0 && dist2 < Range_Down->max_range)
                                     {
                                       if(!first_tf_down){
                                         dist2_old = dist2;
                                         _dist2 = dist2;
                                         first_tf_down =true;
                                       }
                                       Range_Down->range = dist2;
                                       motion_.odom_Down = dist2;
                                       if(i % 200 == 0){
                                         odometer_down += fabs(dist2 - _dist2);
                                         _dist2 = dist2;
                                       }
                                      
                                       //Range_->stamp = ros::Time::now();                         
                                       if((dist2 < 0.2f) && (dist2 < dist2_old) && (motion_.motion_cmd_->command 
                                       == kMoveDownCmd  || motion_.motion_cmd_->command == mMoveDownCmd || 
                                        motion_.motion_cmd_->motion_mode == kTestMode  || 
                                        motion_.motion_cmd_->motion_mode == kAutoMode)){
                                  
                                         //motion_.motion_cmd_->command = kStopCmd;
                                         motion_.motion_cmd_->quick_stop = kQuickStopOn;
                                       }

                                        dist2_old = dist2;
                                     }
                                     else if(dist2 == -1.0)
                                     {
                                       log_error("TF02 failed to read data!");
                                       break;
                                     }
                                     else if(dist2 == 0.0)
                                     {
                                       log_error("TF02 data 0.0 error!");
                                     }

                                     usleep(5000);
                                     i++;
                                   }
                                   
                                  });
    
  }

  void Robot::TF02Stop()
  {
    //log_info("TF02 Stop1.");
    stop_tf02_signal_ = true;
    
    tf02_thread_.join();
    //tf03_thread_.detach();
    tcp_tf02_.closePort();
    log_info("TF02 Stop.");
  }

  void Robot::Run()
  {
    /*  */

    /* create motion thread */
    motion_thread_ = std::thread([&]()
                                 {
                                   //   motion thread
                                   while (false == motion_stop_signal_)
                                   {
                                     // motion fsm
                                     motion_.SystemMotionFSM(heart_beat_flag_);
                                     if(motion_.motion_cmd_->command == 77)
                                     {
                                       motion_stop_signal_ = true;
                                     }

                                     GetParam();

                                     // refresh rate ms
                                     usleep(1000 * kControlPeriod);
                                     
                                   }
                                   
                                 });
  }

  void Robot::Param()
  {
    /*  */

    /* create motion thread */
    param_thread_ = std::thread([&]()
                                 {
                                   //   motion thread
                                   while (false == stop_param_signal_)
                                   {

                                     //est_.update();
                                     //recv_rtk(rtk, rtk_server_);
                                     
                                     // refresh rate ms
                                     usleep(1000);
                                   }
                                 });
  }

  void Robot::Getatt()
  {
    /*  */
    
    
    /* create Getatt thread */
    att_thread_ = std::thread([&]()
                                 {
                                   //log_info("att_thread_ Start.");
                                   //   Getatt thread
                                   while (false == stop_att_signal_)
                                   {

                                     est_.att_run();
                                     //recv_rtk(rtk, rtk_server_);
                                     
                                     // refresh rate ms
                                     usleep(1000);
                                   }
                                   
                                   
                                 });
  }

  void Robot::GetattStop()
  {
    stop_att_signal_ = true;
    est_._att_should_exit = true;
    att_thread_.join();
    
    log_info("att_thread_ Stop.");
  }

  void Robot::Logdata()
  {
		printf("fopen!\n");
    
    fp = fopen ("/home/chaos/ccr_nx/log/att.me", "w+");
    //printf("fprintf0!\n");
    fprintf(fp, "%10s %10s %10s %10s %10s %10s\n", "time", "theta", "psi", "pos", "vel", "accel");
    //fprintf(fp,"%10s %10s %10s %10s %10s %10s %10s %10s %10s %10s\n", "time", "theta", "psi", "pos", "vel", "accel", 
    //  "pos_f", "vel_f", "accel_f", "Volt");
    printf("fprintf1!\n");
    /* create Getdata thread */
    log_thread_ = std::thread([&]()
                                 {
                                   
                                   //   Getdata thread
                                   while (false == stop_log_signal_)
                                   {
                                                                                                                                                				                                                                 				                              
				                              fprintf(fp,"%10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n", est_._dt_now, 
                                        est_._theta, est_._psi, est_._x(0), est_._x(1), est_._u(0));
                                      			                              
                                      usleep(1000*100);                                      
                                      //printf("fprintf done!\n");
                                   }                                  
                                 });
  }

  void Robot::LogdataStop()
  {
    
    log_info("log_thread_ start.");
    stop_log_signal_ = true;
    log_thread_.join();
    log_info("fclose.");
    //log_thread_.detach();
    fclose(fp);
    log_info("log_thread_ Stop.");
  }

  void Robot::Getpos()
  {
    /*  */

    /* create Getatt thread */
    pos_thread_ = std::thread([&]()
                                 {
                                   //log_info("att_thread_ Start.");
                                   //   Getatt thread
                                   while (false == stop_pos_signal_)
                                   {

                                     est_.pos_update();
                                     
                                     // refresh rate ms
                                     usleep(1000*20);
                                   }
                                   
                                 });
  }

  void Robot::GetposStop()
  {
    stop_pos_signal_ = true;
    est_._pos_should_exit = true;
    pos_thread_.join();
    est_.imu_server_.closePort();
    est_.rtk_server_.closePort();
    log_info("pos_thread_ Stop.");
  }

  void Robot::Param1()
  {
    param1_thread_ = std::thread([&]()
                                 {
                                   //   motion thread
                                   while (false == stop_param1_signal_)
                                   {
                                     //printf("GetmaxonsParam start!\n");
                                     motion_.GetmaxonsParam1();
                                     
                                     // refresh rate ms
                                     usleep(100);
                                   }
                                 });
  }

  void Robot::ParamStop()
  {
    stop_param_signal_ = true;
    param_thread_.join();
    
    log_info("Param_thread Stop.");
  }

#define TIME_STAMP_DEBUG 0

  void Robot::GetParam()
  {
    // get pos
    //unsigned int tmp_odm = (unsigned int)GetOdometer();
    // get odometer
    unsigned int tmp_odm = (unsigned int)GetOdometer();

    param_->odometer = (int)((tmp_odm << 16) | (tmp_odm >> 16));

    // time stamp
    dlog_if(TIME_STAMP_DEBUG, "-----");
    // utility::ChangeEndian(motion_.motion_cmd_->time_stamp.year);
    // uint32_t time_stamp = (uint32_t)((((uint64_t)r15 << 48) + ((uint64_t)r14 << 32) + ((uint64_t)r13 << 16) + (uint64_t)r12) / (1000 * 10000));
    // struct tm *time = localtime(&time_stamp);

    // uint32_t time_stamp = utility::ChangeEndian((uint16_t)((motion_.motion_cmd_->time_stamp) << 48));

    dlog_if(TIME_STAMP_DEBUG, "year:%d", utility::ChangeEndian(motion_.motion_cmd_->time_stamp.year));
    dlog_if(TIME_STAMP_DEBUG, "month:%d", utility::ChangeEndian(motion_.motion_cmd_->time_stamp.month));
    dlog_if(TIME_STAMP_DEBUG, "day:%d", utility::ChangeEndian(motion_.motion_cmd_->time_stamp.day));
    dlog_if(TIME_STAMP_DEBUG, "hour:%d", utility::ChangeEndian(motion_.motion_cmd_->time_stamp.hour));
    dlog_if(TIME_STAMP_DEBUG, "minute:%d", utility::ChangeEndian(motion_.motion_cmd_->time_stamp.minute));
    dlog_if(TIME_STAMP_DEBUG, "second:%d", utility::ChangeEndian(motion_.motion_cmd_->time_stamp.second));

    // copy to shared memory
    // synchronize with semaphore
    // sem_wait(*command_semaphore);
    vision_msg_->odometer = (int)tmp_odm;
    memcpy((MotionCmd *)&(vision_msg_->host_motion_cmd),
           (MotionCmd *)motion_.motion_cmd_, sizeof(MotionCmd));
    // sem_post(*command_semaphore);

    // get speed, mm/s
    odom_this_time_ = GetOdometer();
    param_->speed =
        (short)((odom_this_time_ - odom_last_time_) * 1000 / kControlPeriod);
    // log_debug("delta odom:%f", odom_this_time_ - odom_last_time_);
    // log_debug("int size:%d", sizeof(int));
    odom_last_time_ = odom_this_time_;
    //if(odom_this_time_ > 1000.f &&(short)odom_this_time_%1000 < 50){printf("\r odom_this_time_-%f mm;speed-%d mm/s.\n", odom_this_time_, param_->speed);}
    // log_debug("speed %d mm/s", param_->speed);
  }

  void Robot::Stop()
  {
    log_info("Start stop motion_thread.\n");
    motion_stop_signal_ = true;
    motion_thread_.join();
    log_info("Start stop robot.\n");
    // force stop
    motion_.Stop();
    
    log_info("stop robot.\n");
    motion_.Idle();
    log_info("Idle robot.\n");
    tcp_server_.closePort();
    printf("tcp_server_ close done!\n");
  }

  void Robot::StartUVC()
  {
    visual_buffer_info_t visual_buf;
    // create uvc thread, use lambda
    printf("\rstart uvc camera.\n");

#if SAVE_UVC_PIC
    /* create uvc pic folder */
    // create folder
    uvc_folder_name = "uvc_" + utility::GetSystemTime();

    utility::CreateDirectory(uvc_folder_name.c_str());
#endif
    uvc_vision_thread_ = std::thread([&]()
                                     {
                                       FILE *uvc_file;
                                       std::string uvc_file_name;
                                       double odm_last_time = 0;
                                       double odm_this_time = 0;
                                       double odm_delta;
                                       // double odm_total = 0;
                                       while (stop_uvc_signal_ == false)
                                       {
                                         uvc_cam_.Stream();
                                         

                                         /* udp send */
                                         photo_info_t *photo_info = (photo_info_t *)visual_buf.data;

                                         photo_info->position = 4;
                                         photo_info->group = 0;
                                         photo_info->sequence = 0;
                                         photo_info->time_stamp = uvc_cam_.frame_cnt_;
                                         photo_info->width = uvc_cam_.img_width_;
                                         photo_info->height = uvc_cam_.img_height_;
                                         photo_info->bytes_per_pixel = 3;
                                         photo_info->reserved = 0;

                                         memcpy((char *)visual_buf.data + sizeof(photo_info_t),
                                                (char *)uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].start,
                                                uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].length);

#if SAVE_UVC_PIC
                                         //  save uvc jpg
                                         odm_this_time = param_->odometer;

                                         odm_delta = abs(odm_this_time - odm_last_time);

                                         // only save uvc pic every 500mm and move up direction.
                                         if (odm_delta > 500 && motion_.motion_cmd_->command == kMoveUpCmd)
                                         {
                                           log_debug("odm delta:%f", odm_delta);
                                           uvc_file_name = uvc_folder_name + "/uvc_odm_" +
                                                           std::to_string(static_cast<int>(odm_this_time)) + "mm" +
                                                           ".jpg";
                                           uvc_file = fopen(uvc_file_name.c_str(), "wb");
                                           fwrite((char *)uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].start,
                                                  uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].length, 1, uvc_file);
                                           fclose(uvc_file);
                                           odm_last_time = odm_this_time;
                                         }
#endif
                                         // send uvc frame to host
                                         visual_buf.data_length =
                                             uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].length + sizeof(photo_info_t);
                                         visual_buf.position = 4;
                                         visual_buf.time_stamp = uvc_cam_.frame_cnt_;

                                         // udp send
                                         send_visual(visual_buf, udp_client_);
                                         uvc_cam_.QBuf();

                                         // referesh time, 100ms
                                         usleep(100000);
                                       }
                                     });
  }

  void Robot::StopUVC()
  {
    stop_uvc_signal_ = true;
    uvc_vision_thread_.join();

    log_info("\rstop uvc camera.");
    // stop uvc
    uvc_cam_.DeInit();
  }

  // shut down robot
  void Robot::ShutDown()
  {
    // shutdown system
    log_info("shutdown system");

    // modbus_thread_.join();
  }

  double Robot::GetOdometer() const { return motion_.odometer_; }

} // namespace ccr_split

