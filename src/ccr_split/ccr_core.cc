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
  sleep(2);
  //start pmhv thread
  ccr.PMhvRecv();
  sleep(1);
  // start tfmini thread
  ccr.TF03Recv();

  ccr.TF02Recv();
  
  ccr.TFminiRecv();
  

  ccr.Getatt();
  sleep(1);
  
  ccr.Getpos();
  
  sleep(1);
  ccr.Logdata();
  
  //*/
  //sleep(2);
#if TEST_MOTION
  // robot run
  ccr.Run();
  //sleep(2);
#endif
  printf("ccr Run done!\n");
#if TEST_UVC
  ccr.StartUVC();
#endif
  
 
  //sleep(60000);
  
  // wait to exit

  //PAUSE();
  do {                                                                         
    printf("---------------press e key to exit!---------------\n");        
    sleep(1);                                                              
  } while (getchar() != 'e');
  
  /*do {                                                                                    
    printf("---------------press any key to exit!---------------\n");                                                              
    getchar();
  } while (0);
  */
  printf("ccr Stop start!\n");
#if TEST_MOTION
  //   wait thread complete
  ccr.Stop();
  printf("ccr Stop done!\n");
#endif

  sleep(3); 

  ccr.PMhvStop();
  sleep(1);
  ccr.LogdataStop();
  printf("LogdataStop done!\n");
  sleep(1);
  ccr.GetposStop();

  ccr.GetattStop();
  sleep(1);
  
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

