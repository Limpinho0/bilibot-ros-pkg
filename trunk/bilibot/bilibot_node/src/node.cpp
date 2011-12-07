#include <stdio.h>
#include <stdint.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "serial.h"

extern "C" {
    #include "powerboard_sdk/wireformat.h"
}

void sendPacket(Serial* serial, packet_t* rxPkt);

static int packet_drops = 0;

int main(int argc, char **argv)
{
    Serial* serial = new Serial("/dev/ttyACM0");
    packet_t rxPkt;
    packet_t* txPkt;
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

    uint8_t position = 0;
    txPkt = PKT_Create(PKTYPE_CMD_SET_ARM_POS, 0, &position, 1);
    sendPacket(serial, txPkt);

    while(ros::ok())
    {
        uint8_t byte = serial->readByte();

        if(PKT_Decoded(byte, &rxPkt, &status) != DECODE_STATUS_INCOMPLETE) {
            switch(status.state)
            {
            case DECODE_STATUS_COMPLETE: 
                switch (rxPkt.type)
                {
                case PKTYPE_STATUS_HEARTBEAT:
                    break;
                case PKTYPE_STATUS_ARM_STATE:
                    msg.data = rxPkt.payload[0];
                    pub.publish(msg);
                    break;
                case PKTYPE_STATUS_GYRO_RAW: 
                    break;
                default:
                    ROS_WARN("received unknown packet type from powerboard");
                }
                break;
            case DECODE_STATUS_INVALID:
                ROS_WARN("dropped packets so far: %d", ++packet_drops);
                break;
            }
            status.recvd = 0;
        }

        ros::spinOnce();
    }

    serial->closePort();
    return 0;
}

void sendPacket(Serial* serial, packet_t* pkt)
{
    uint8_t* txBuffer = (uint8_t*)malloc(sizeof(packet_t));
    uint8_t len = PKT_ToBuffer(pkt, txBuffer);
    for(int i=0; i < len; i++) {
        serial->writeByte(txBuffer[i]);
    }
    free(txBuffer);
}

