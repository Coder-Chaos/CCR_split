#pragma once

#include <string>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
//#include "/usr/include/linux/i2c.h" 


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>
#include "log.hpp"

extern "C" {
  #include <linux/i2c.h>  
  #include <linux/i2c-dev.h>
  #include <i2c/smbus.h>
}

//#include <time.h>

//typedef unsigned short USHORT;
namespace cuav
{
  class PMhv
  {
    public:
      PMhv(std::string _name);
      ~PMhv(){};
      int getVolt();
      int getCurr();
      void closePort();

      float dataVolt[2];
      float dataCurr[1];

    private:
      std::string portName_;
      //int baudRate_;
      int file_;

      bool readData(uint8_t & _buf, uint8_t chn);
      bool writeData(uint8_t chn);
  };
  
}

