#ifndef VEHICLE_H
#define VEHICLE_H



#include "controlcan.h"

class Vehicle 
{
public:
    void read_obstacle_info_from_sensor();
    void send_vehicle_steer();

public:

    int can_open();

    int can_start(int channel_index);

    int can_write(int channel,unsigned int id,unsigned char *buf,int len);

    void can_quit();

private:
    VCI_BOARD_INFO pInfo;

    int is_can0_started ;
    int is_can1_started ;


};

#endif // VEHICLE_H
