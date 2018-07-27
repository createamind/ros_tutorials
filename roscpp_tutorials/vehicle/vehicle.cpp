
#include "ros/ros.h"
#include "std_msgs/String.h"


#include <math.h>
#include "vehicle.h"
#include <stdio.h>
#include "controlcan.h"



Vehicle * haval = new Vehicle;


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  haval->send_vehicle_steer();
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  
  ros::Subscriber sub = n.subscribe("vehiclejzcreateaimind", 1000, chatterCallback);


  haval->can_open();
  haval->can_start(1);

  ros::spin();

  return 0;
}






  int Vehicle::can_open()
  {

      if(VCI_OpenDevice(VCI_USBCAN2,0,0) == 0)
      {
          printf(">>open deivce success!\n");
      }
      else
      {
          printf(">>open deivce error!\n");
          return -1;
      }

      if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)
      {
          printf(">>Get VCI_ReadBoardInfo success!\n");

          printf(" %08X", pInfo.hw_Version);printf("\n");
          printf(" %08X", pInfo.fw_Version);printf("\n");
          printf(" %08X", pInfo.dr_Version);printf("\n");
          printf(" %08X", pInfo.in_Version);printf("\n");
          printf(" %08X", pInfo.irq_Num);printf("\n");
          printf(" %08X", pInfo.can_Num);printf("\n");

          printf(">>Serial_Num:");
          for(int i = 0;i < 20;i++)
          {
              printf("%c",pInfo.str_Serial_Num[i]);
          }
          printf("\n");

          printf(">>hw_Type:");
          for(int i = 0;i < 10;i++)
          {
              printf("%c",pInfo.str_hw_Type[i]);
          }
          printf("\n");
      }else
      {
          printf(">>Get VCI_ReadBoardInfo error!\n");
          exit(1);
      }
  }

  int Vehicle::can_start(int channel_index)
  {
      VCI_INIT_CONFIG config;

      config.AccCode  =0;
      config.AccMask  =0xffffffff;
      config.Filter   =1;
      config.Mode     =0;

      //use 500khz
      config.Timing0  = 0x00;   // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-12K
      config.Timing1  = 0x1c;   // BTR1   041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K


      if(VCI_InitCAN(VCI_USBCAN2,0,channel_index,&config)!=1)
      {
          printf("init CAN error\n");
          VCI_CloseDevice(VCI_USBCAN2,0);
          return -1;
      }

      if(VCI_StartCAN(VCI_USBCAN2,0,channel_index)!=1)
      {
          printf("Start CAN error\n");
          VCI_CloseDevice(VCI_USBCAN2,0);
          return -1;
      }
      return 0;
  }

  int Vehicle::can_write(int channel,unsigned int id,unsigned char *buf,int len)
  {
      unsigned long sndCnt;

      VCI_CAN_OBJ send[1];

      send[0].ID          = id;
      send[0].SendType    = 0;  // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
      send[0].RemoteFlag  = 0;  // 0-数据帧；1-远程帧
      send[0].ExternFlag  = 0;  // 0-标准帧；1-扩展帧
      send[0].DataLen     = 8;     // DL

      memcpy(send[0].Data,buf,len);

      sndCnt = VCI_Transmit(VCI_USBCAN2, 0, channel, send, 1);

      return sndCnt;
  }

  void Vehicle::can_quit()
  {
      VCI_CloseDevice(VCI_USBCAN2,0);
  }

  void Vehicle::read_obstacle_info_from_sensor()
  {
      int channel_id = 0;
      VCI_CAN_OBJ rec[100];

      int reclen = 0;

      if((reclen = VCI_Receive(VCI_USBCAN2,0,channel_id,rec,100,100))>0)
      {
          for(int j = 0;j<reclen;j++){

  #if 1
              printf("(CAN_ID:%08X)\t=>", rec[j].ID);

              for(int i = 0; i < rec[j].DataLen; i++)
              {
                  printf(" %.2X", rec[j].Data[i]);
              }
              printf("\n");
  #endif
              int current_id = rec[j].ID;

              if(current_id >=0x610 && current_id <=0x62F)
              {
                  printf("master\n");
              }

              if(current_id >=0x6B0 && current_id <=0x6CF)
              {
                  //printf("slave\n");

                  float speed_x    = (((rec[j].Data[3] & 0xfe)>>1) + ((rec[j].Data[4] & 0x000f) << 8) -1024) * 0.1;
                  float speed_y    = (((rec[j].Data[4] & 0xf0)>>4) + ((rec[j].Data[5] & 0x00ef) << 8) -1024) * 0.1;

              }

              if(current_id == 0x601)
              {
  #if DEBUG_RADAR
                  printf("(CAN_ID:%08X)\t=>", rec[j].ID);

                  for(int i = 0; i < rec[j].DataLen; i++)
                  {
                      printf(" %.2X", rec[j].Data[i]);
                  }
                  printf("\n");
  #endif
              }

              if(current_id == 0x6FA)
              {
  #if DEBUG_RADAR
                  printf("(CAN_ID:%08X)\t=>", rec[j].ID);

                  for(int i = 0; i < rec[j].DataLen; i++)
                  {
                      printf(" %.2X", rec[j].Data[i]);
                  }
                  printf("\n");
  #endif
              }else{

              }
          }
      } else {

      }
      //VCI_ClearBuffer(VCI_USBCAN2,0,channel_id);
  }

  void Vehicle::send_vehicle_steer()
  {
      /*
      *
              48 00 00 00 00 00 00 00     //0度
              48 00 00 01 2C 00 00 00	    //30度
              48 00 00 FE D4 00 00 00     //-30度
              48 00 00 02 58 00 00 00     //60度
              48 00 00 FD A8 00 00 00	    //-60度
              48 00 00 03 84 00 00 00	    //90度
              48 00 00 0E 10 00 00 00	    //360度
              48 00 00 F1 F0 00 00 00	    //-360度

              加速指令：
              28 0A 00 00 00 00 00 00
              制动指令：
              88 00 20 00 00 00 00 00
              速度控制指令：
              18 00 00 00 00 00 03 E8
      *
      */

      unsigned char buf[8]    = {0x48,00,00,00,0x32,00,00,00};

      // #int value = 10 * steer_slider->value();

      // #buf[3] = (value>>8) & 0xff;
      // #buf[4] = value & 0xff;
      printf("\n test 982737484 zdx   928384");

      can_write(0,0xE2,buf,8);
  }

