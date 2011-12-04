#include <stdio.h>
#include <stdint.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "serial.h"

extern "C" {
    #include "powerboard_sdk/wireformat.h"
}

void send_packet(Serial* serial, packet_t* pkt);

static int packet_drops = 0;

int main(int argc, char **argv)
{
    Serial* serial = new Serial("/dev/ttyACM0");
    packet_t pkt;
    status_t status;
    status.recvd = 0;
    std_msgs::UInt8 msg;

    if (serial->openPort() < 0) {
        printf("unable to open port\n");
        return -1;
    }

    ros::init(argc, argv, "bilibot_node");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::UInt8>("/status", 1);

    while(ros::ok())
    {
        uint8_t byte = serial->readByte();

        if(PKT_Decoded(byte, &pkt, &status) != DECODE_STATUS_INCOMPLETE) {
            switch(status.state)
            {
            case DECODE_STATUS_COMPLETE: 
               msg.data = pkt.payload[0];
                pub.publish(msg);
                status.recvd = 0;
                break;
            default:
                if (status.recvd > 0)
                    ROS_WARN("dropped packets so far: %d", ++packet_drops);
                status.recvd = 0;
                break;
            }
        }

        ros::spinOnce();
    }

    serial->closePort();
    return 0;
}

void send_packet(Serial* serial, packet_t* pkt)
{
  serial->writeByte(0);
}

