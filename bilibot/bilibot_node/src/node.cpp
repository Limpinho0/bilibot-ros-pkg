#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "bilibot_node/PowerboardSensorState.h"
#include "serial.h"

extern "C" {
    #include "powerboard_sdk/wireformat.h"
}

void sendPacket(Serial* serial, packet_t* rxPkt);

static int packet_drops = 0;

    // fill in payload
//     payload[0] = ADC_BASE_POT;
//     payload[1] = HL_GetTargetPos();
//     payload[2] = HL_GetBaseSpeed();
//     payload[3] = HL_GetLimitState();
//     payload[4] = ADC_BASE_CURR;
//     payload[5] = ADC_HAND_CURR;

// #arm stuff
// uint8 gripper_current
// uint8 elbow_current
// uint8 elbow_pos
// bool gripper_closed
// int8 gripper_vel
// int8 elbow_vel
// bool lower_limit
// bool upper_limit
void updateArmState(bilibot_node::PowerboardSensorState &pbstate, packet_t rxPkt){
  pbstate.elbow_pos = rxPkt.payload[0];
  pbstate.elbow_vel = rxPkt.payload[2];
  pbstate.lower_limit = rxPkt.payload[3] & 0x01;
  pbstate.upper_limit = rxPkt.payload[3] & 0x02;
  pbstate.gripper_current = rxPkt.payload[5];
  pbstate.elbow_current = rxPkt.payload[4];
  
}





//     payload[0] = ADC_BATT;
//     payload[1] = ADC_AD0;
//     payload[2] = ADC_AD1;
//     if((READ_PWR_IND) == 0)  bstate |= 0x01;
//     if((READ_OVR_CHRG) == 0) bstate |= 0x02;
//     if(READ_KIN_EN)          bstate |= 0x04;
//     if(READ_CREATE_PWR_EN)   bstate |= 0x08;
//     if(READ_CREATE_ON)       bstate |= 0x10;
//     if(READ_CREATE_CHRG_IND) bstate |= 0x20;
//     if((READ_DEMO_BTN) == 0) bstate |= 0x40;
//     if((READ_ESTOP_BTN) ==0) bstate |= 0x80;
    

void updateBattState(bilibot_node::PowerboardSensorState &pbstate, packet_t rxPkt){
  pbstate.battery_voltage = rxPkt.payload[0];
  pbstate.charging_current = (int)rxPkt.payload[1] - (int)rxPkt.payload[2];
  pbstate.charger_power_ind = rxPkt.payload[3] & 0x01;
  pbstate.charger_trickle = rxPkt.payload[3] & 0x02;
  pbstate.kinect_enabled = rxPkt.payload[3] & 0x04;
  pbstate.create_charge_enabled = rxPkt.payload[3] & 0x08;
  pbstate.create_on = rxPkt.payload[3] & 0x10;
  pbstate.create_charging = rxPkt.payload[3] & 0x20;
  pbstate.demo_button = rxPkt.payload[3] & 0x40;
  pbstate.estop_button = rxPkt.payload[3] & 0x80;
  
  
}

int main(int argc, char **argv)
{
    Serial* serial = new Serial("/dev/ttyACM0");
    packet_t rxPkt;
    packet_t* txPkt;
    status_t status;
    status.recvd = 0;
    std_msgs::UInt8 msg;
    bilibot_node::PowerboardSensorState pbstate;

    if (serial->openPort() < 0) {
        printf("unable to open port\n");
        return -1;
    }

    ros::init(argc, argv, "bilibot_node");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::UInt8>("/status", 1);
    ros::Publisher pub2 = n.advertise<bilibot_node::PowerboardSensorState>("/powerboard", 1);

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
// 	    std::cout<<(int)rxPkt.type<<" ";
                switch (rxPkt.type)
                {
                case PKTYPE_STATUS_HEARTBEAT:
                    break;
                case PKTYPE_STATUS_ARM_STATE:
                    msg.data = rxPkt.payload[0];
                    pub.publish(msg);
		    updateArmState(pbstate,rxPkt);
		    pub2.publish(pbstate);
// 		    for(int i=0;i<6;i++)
// 		      std::cout<<(int)rxPkt.payload[i]<<" ";
                    break;
                case PKTYPE_STATUS_GYRO_RAW: 
		  pbstate.gyro_raw = rxPkt.payload[0];
                    break;
		    
                case PKTYPE_STATUS_BATT_RAW:
		  updateBattState(pbstate,rxPkt);
		  break;
                default:
                    ROS_WARN("received unknown packet type from powerboard");
                }
// 		    std::cout<<std::endl;
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

