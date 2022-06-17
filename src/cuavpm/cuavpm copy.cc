#include "cuavpm.hpp"

//#define DEBUG
#define PMHV_SENSOR     0x48
#define PMHV_VOLT       0x40
#define PMHV_CURR       0x41
#define PMHV_VOLT_MAX   29.4f
#define PMHV_VOLT_MIN   19.25f

namespace cuav
{
    PMhv::PMhv(std::string _name) :
      portName_(_name)
    {
      //打开文件
      file_ = open(_name.c_str(), O_RDWR);
      if(file_ == -1)
      {
        log_error("Failed to open i2c port!");
        exit(0);
      }
      //声明为IIC设备文件
      if(ioctl(file_, I2C_SLAVE, PMHV_SENSOR) < 0) {    
        perror("Failed to connect to the sensor\n"); 
        exit(0); 
      }
  
    }

    bool PMhv::writeData(uint8_t chn)
    {
      //unsigned char _buf[1];     
      //地址+写位，写位为0，所以只需要地址左移1位即可
      //_buf[0] = _add; 
      //从机的读取协议
      /*_buf[1] = 0x00;
      _buf[2] = 0x00;
      _buf[3] = 0x00;
      _buf[4] = 0x01;
      _buf[5] = 0x60;
      */
      //文件写操作
      if (i2c_smbus_write_byte_data(file_, PMHV_SENSOR, chn) < 0){
      //if (write(file_, _buf, 1) != 1){ 
        perror("Failed write to device"); 
        return false; 
      }
      return true; 
    }

    bool PMhv::readData(uint8_t & buf, uint8_t chn)
    {
      int total = 0, ret = 0;

      char writeBuff[1];
      //读，从机地址+读位
		  //writeBuff[0]= (s_add<<1)|1; 
		  //先把读的命令发给从机
      if(i2c_smbus_write_byte_data(file_, PMHV_SENSOR, chn) < 0) { 
        perror("Failed to reset the read address\n"); 
        return false; 
      }
      //然后利用文件读操作，把读到的数据全部放到buf数组里面
      buf = i2c_smbus_read_byte_data(file_, PMHV_SENSOR);
      if(buf < 0){ 
        perror("Failed to read in the buffer\n"); 
        return false; 
      }

      return true;
    }

    int PMhv::getVolt()
    {
      uint8_t _buf;
      //先写，引起从机的反馈
      //if(!writeData(PMHV_VOLT)){ 
      //  return -1; 
      //} 
      //再读取从机反馈
      if(!readData(_buf, PMHV_VOLT)){ 
        return -1; 
      }
      
      //从从机反馈的数据里面根据从机的协议提取我们想要的东西
			dataVolt[0] = _buf * 5.0 * 60 / (256 * 3.3);
      dataVolt[1] = (dataVolt[0] - PMHV_VOLT_MIN)/(PMHV_VOLT_MAX-PMHV_VOLT_MIN) * 100;
			//printf("buf%d; Volt: %4.2f\n", _buf,dataVolt[0]);
      return 1;

    }

    int PMhv::getCurr()
    {
      uint8_t _buf;
      //先写，引起从机的反馈
      //if(!writeData(PMHV_CURR)){ 
      //  return -1; 
      //} 
      //再读取从机反馈
      if(!readData(_buf, PMHV_CURR)){ 
        return -1; 
      }
      //buf = buf;
      
      //从从机反馈的数据里面根据从机的协议提取我们想要的东西
			dataCurr[0] = _buf * 0.2 / 56;
			//printf("buf%d; Current: %2.2f\n", _buf, dataCurr[0]);
      return 1;
    }

    void PMhv::closePort()
    {
      close(file_);
    }
}
