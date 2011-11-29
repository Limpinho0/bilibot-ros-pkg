#include <stdio.h>
#include <stdint.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
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
    std_msgs::String msg;

    if (serial->openPort() < 0) {
        printf("unable to open port\n");
        return -1;
    }

    ros::init(argc, argv, "bilibot_node");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::String>("/status", 1);

    // COMMS THROUGH EXTERNAL UART PORT (XBee serial)
    while(ros::ok())
    {
        uint8_t byte = serial->readByte();
        std::stringstream ss;

        if(PKT_Decoded(byte, &pkt, &status) != DECODE_STATUS_INCOMPLETE) {
            switch(status.state)
            {
            case DECODE_STATUS_COMPLETE: 
                ss << "good packet: " << pkt.payload[0];
                msg.data = ss.str();
                pub.publish(msg);
                break;
            case DECODE_STATUS_INVALID: 
                ss << "bad packets: " << packet_drops++;
                break;
            default:
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
  uint16_t i;
  serial->writeByte(0);
}

