#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <math.h>
#include <ros/console.h>
#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "bilibot_node/PowerboardSensorState.h"
#include "bilibot_node/SetArmPosition.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "serial.h"

extern "C" {
    #include "powerboard_sdk/wireformat.h"
}

void sendPacket(Serial* serial, packet_t* rxPkt);
bool toggleCreatePower(std_srvs::Empty::Request  &req,
            std_srvs::Empty::Response &res );
bool toggleKinectPower(std_srvs::Empty::Request  &req,
            std_srvs::Empty::Response &res );
bool toggleHandState(std_srvs::Empty::Request  &req,
            std_srvs::Empty::Response &res );
bool setArmPosition(bilibot_node::SetArmPosition::Request  &req,
            bilibot_node::SetArmPosition::Response &res );

static int packet_drops = 0;
Serial* serial;

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

// gyro globals
double cal_offset = 140;
double rate = 0;
double orientation = 0;

bool isMoving = false;
void odomCallback(const nav_msgs::Odometry& odomMsg)
{
    if (fabs(odomMsg.twist.twist.angular.x - 0.0) < 0.0001 && 
        fabs(odomMsg.twist.twist.angular.y - 0.0) < 0.0001 && 
        fabs(odomMsg.twist.twist.angular.z - 0.0) < 0.0001 && 
        fabs(odomMsg.twist.twist.linear.x - 0.0) < 0.0001 && 
        fabs(odomMsg.twist.twist.linear.y - 0.0) < 0.0001 && 
        fabs(odomMsg.twist.twist.linear.z - 0.0) < 0.0001) 
    {
        isMoving = false;
    }
    else
    {
        isMoving = true;
    }
}

void updateGyroState(sensor_msgs::Imu& imuMsg, packet_t rxPkt, boost::circular_buffer<float>& calibration)
{
    uint8_t gyro_adc = rxPkt.payload[0];
    double current_time = ros::Time::now().toSec();
    double last_time = imuMsg.header.stamp.toSec();

    if (!isMoving) {
        calibration.push_back(float(gyro_adc));
        double total = 0;
        BOOST_FOREACH( float reading, calibration )
        {
            total += reading;
        }
        cal_offset = total / calibration.size(); 
    }

    double dt = current_time - last_time;
    double scale_correction = 1.0;
    double maxValue = 255;
    double vRef = 5;
    double zeroRateV = cal_offset * vRef / maxValue;
    double sensitivity = 0.013;

    rate = (gyro_adc * vRef / maxValue - zeroRateV) / sensitivity;

    orientation += rate * dt;
    //ROS_INFO("orientation (deg): %f", (orientation));
    //ROS_INFO("orientation (rad): %f", (orientation * (M_PI/180.0)));

    imuMsg.header.stamp = ros::Time::now();
    imuMsg.orientation = tf::createQuaternionMsgFromYaw(orientation * (M_PI/180.0));
}

int main(int argc, char **argv)
{
    serial = new Serial("/dev/ttyACM0");
    packet_t rxPkt;
    packet_t* txPkt;
    status_t status;
    status.recvd = 0;
    bilibot_node::PowerboardSensorState pbstate;
    sensor_msgs::Imu imu;
    boost::circular_buffer<float> calibration(140);

    if (serial->openPort() < 0) {
        ROS_ERROR("unable to open port\n");
        return -1;
    }

    ros::init(argc, argv, "bilibot_node");

    ros::NodeHandle n;

    // fill in imu data (start at 0 orientatino)
    imu.orientation = tf::createQuaternionMsgFromYaw(0.0);
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "gyro_link";
    boost::array<double, 9> cov = {{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6}}; 
    boost::array<double, 9> cov2 = {{-1, 0, 0, 0, 0, 0, 0, 0, 0}}; 
    std::copy(cov.begin(), cov.end(), imu.orientation_covariance.begin());
    std::copy(cov.begin(), cov.end(), imu.angular_velocity_covariance.begin());
    std::copy(cov2.begin(), cov2.end(), imu.linear_acceleration_covariance.begin());

    ros::ServiceServer hand_state_service = n.advertiseService("toggle_hand_state", toggleHandState);
    ros::ServiceServer create_pwr_service = n.advertiseService("toggle_create_power", toggleCreatePower);
    ros::ServiceServer kinect_pwr_service = n.advertiseService("toggle_kinect_power", toggleKinectPower);
    ros::Publisher sensor_state_pub = n.advertise<bilibot_node::PowerboardSensorState>("sensor_state", 1);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 1);
    ros::Subscriber odom_sub = n.subscribe("odom", 1, odomCallback);

    uint8_t position = 255;
    txPkt = PKT_Create(PKTYPE_CMD_SET_ARM_POS, 0, &position, 1);
    sendPacket(serial, txPkt);
    free(txPkt);

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
                    updateArmState(pbstate,rxPkt);
                    sensor_state_pub.publish(pbstate);
                    break;
                case PKTYPE_STATUS_GYRO_RAW: 
                    updateGyroState(imu, rxPkt, calibration);
                    pbstate.gyro_raw = rxPkt.payload[0];
                    imu_pub.publish(imu);
                    break;
		    
                case PKTYPE_STATUS_BATT_RAW:
                    updateBattState(pbstate,rxPkt);
                    break;
                default:
                    ROS_WARN("received unknown packet type from powerboard");
                }
                break;
            case DECODE_STATUS_INVALID:
                //ROS_WARN("dropped packets so far: %d", ++packet_drops);
                break;
            }
            status.recvd = 0;
        }

        ros::spinOnce();
    }

    serial->closePort();
    
    return 0;
}

bool toggleCreatePower(std_srvs::Empty::Request  &req,
            std_srvs::Empty::Response &res )
{
    ROS_INFO("toggling power to create");
    uint8_t unused = 0;
    packet_t* txPkt = PKT_Create(PKTYPE_CMD_TOGGLE_CREATE, 0, &unused, 1);
    sendPacket(serial, txPkt);
    free(txPkt);
    return true;
}

bool toggleHandState(std_srvs::Empty::Request  &req,
            std_srvs::Empty::Response &res )
{
    ROS_INFO("toggling hand state");
    uint8_t unused = 0;
    packet_t* txPkt = PKT_Create(PKTYPE_CMD_TOGGLE_HAND_STATE, 0, &unused, 1);
    sendPacket(serial, txPkt);
    free(txPkt);
    return true;
}

bool toggleKinectPower(std_srvs::Empty::Request  &req,
            std_srvs::Empty::Response &res )
{
    ROS_INFO("toggling power to kinect");
    uint8_t unused = 0;
    packet_t* txPkt = PKT_Create(PKTYPE_CMD_TOGGLE_KINECT, 0, &unused, 1);
    sendPacket(serial, txPkt);
    free(txPkt);
    return true;
}

bool setArmPosition(bilibot_node::SetArmPosition::Request  &req,
            bilibot_node::SetArmPosition::Response &res )
{
    ROS_INFO("setting arm postion to %d", req.position );
    packet_t* txPkt = PKT_Create(PKTYPE_CMD_SET_ARM_POS, 0, &req.position, 1);
    sendPacket(serial, txPkt);
    free(txPkt);
    return true;
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

