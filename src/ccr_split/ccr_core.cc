#include "robot.hpp"
#include "uvc.hpp"
#define TEST_MOTION 1
#define TEST_UVC 1
#include <iostream>

using namespace std;

int main(int argc, char **argv) {

#if DEBUG_ON
  // set log level
  log_set_level(1);
#else
  log_set_level(2);
#endif
  
  ccr_split::Robot ccr;
  

  // connect host
  ccr.ConnectHost();

  // start guard thread
  ccr.Guard();

  //ccr.StartVisionDetection();
  
  //start pmhv thread
  ccr.PMhvRecv();

  // start tfmini thread
  ccr.TF03Recv();

  ccr.TF02Recv();

  ccr.TFminiRecv();
  
  /*
  ccr.Getatt();
  sleep(2);
  
  ccr.Getpos();
  
  sleep(2);
  ccr.Logdata();
  */
  
#if TEST_MOTION
  // robot run
  ccr.Run();
#endif
//printf("ccr Run done!\n");
#if TEST_UVC
  ccr.StartUVC();
#endif
  
 
  //sleep(60000);
  
  // wait to exit

  //PAUSE();
  do {                                                                         
    printf("---------------press e key to exit!---------------\n");        
    //printf("---------------press any key to exit!---------------\n");                                                              
  } while (getchar() != 'e');
    //getchar();
  //} while (0);

  //printf("ccr Stop start!\n");
#if TEST_MOTION
  //   wait thread complete
  ccr.Stop();
  printf("ccr Stop done!\n");
#endif

  

  ccr.PMhvStop();
/*
  ccr.LogdataStop();
  printf("LogdataStop done!\n");
  ccr.GetattStop();
  //sleep(1);
  ccr.GetposStop();
*/
  ccr.TF03Stop();
  ccr.TF02Stop();
  ccr.TFminiStop();
  printf("tf Stop done!\n");
#if TEST_UVC
  ccr.StopUVC();
#endif

  //ccr.StopVisionDetection();
  
  ccr.DisConnectHost();

  ccr.ShutDown();
  //printf("ccr ShutDown done!\n");
  return 0;
}

