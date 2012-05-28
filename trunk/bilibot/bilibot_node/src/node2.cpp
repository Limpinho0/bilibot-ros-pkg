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
#include "bilibot_node/MotorState.h"
#include "bilibot_node/SetMotorPWM.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "serial.h"

//TODO: change to motordriver_sdk?
extern "C" {
    #include "powerboard_sdk/wireformat.h"  
}

void sendPacket(Serial* serial, packet_t* rxPkt);
bool toggleKinectPower(std_srvs::Empty::Request  &req,
            std_srvs::Empty::Response &res );
bool toggleHandState(std_srvs::Empty::Request  &req,
            std_srvs::Empty::Response &res );
//For 1.0...
// bool setArmPosition(bilibot_node::SetArmPosition::Request  &req,
//             bilibot_node::SetArmPosition::Response &res );
bool setMotorPWM(bilibot_node::SetMotorPWM::Request  &req,
            bilibot_node::SetMotorPWM::Response &res );

void sendMotorPWM(int16_t left_pwm, int16_t right_pwm);
	    
	    
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
// bool lower_limitpacket_t
// bool upper_limit
// void updateArmState(bilibot_node::PowerboardSensorState &pbstate, packet_t rxPkt){
//   pbstate.elbow_pos = rxPkt.payload[0];
//   pbstate.elbow_vel = rxPkt.payload[2];
//   pbstate.lower_limit = rxPkt.payload[3] & 0x01;
//   pbstate.upper_limit = rxPkt.payload[3] & 0x02;
//   pbstate.gripper_current = rxPkt.payload[5];
//   pbstate.elbow_current = rxPkt.payload[4];
//   
// }


//based on motorboard 2.0
void updateMotorState(bilibot_node::MotorState &pbstate, packet_t rxPkt){
  pbstate.left_current = rxPkt.payload[0];
  pbstate.right_current = rxPkt.payload[2];
  pbstate.left_count = (rxPkt.payload[3]<<8)+rxPkt.payload[4];
  pbstate.left_count = (rxPkt.payload[5]<<8)+rxPkt.payload[6];
  pbstate.right_temp = rxPkt.payload[7];
  pbstate.right_temp = rxPkt.payload[8];
  pbstate.right_vel = rxPkt.payload[9];
  pbstate.left_vel = rxPkt.payload[10];
  //TODO: convert imu data to floats...
  pbstate.imu_raw.linear_acceleration.x = rxPkt.payload[11];
  pbstate.imu_raw.linear_acceleration.y = rxPkt.payload[12];
  pbstate.imu_raw.linear_acceleration.z = rxPkt.payload[13];
  pbstate.imu_raw.angular_velocity.x = rxPkt.payload[14];
  pbstate.imu_raw.angular_velocity.y = rxPkt.payload[15];
  pbstate.imu_raw.angular_velocity.z = rxPkt.payload[16];
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
// void updateBattState(bilibot_node::PowerboardSensorState &pbstate, packet_t rxPkt){
//   pbstate.battery_voltage = rxPkt.payload[0];
//   pbstate.charging_current = (int)rxPkt.payload[1] - (int)rxPkt.payload[2];
//   pbstate.charger_power_ind = rxPkt.payload[3] & 0x01;
//   pbstate.charger_trickle = rxPkt.payload[3] & 0x02;
//   pbstate.kinect_enabled = rxPkt.payload[3] & 0x04;
//   pbstate.create_charge_enabled = rxPkt.payload[3] & 0x08;
//   pbstate.create_on = rxPkt.payload[3] & 0x10;
//   pbstate.create_charging = rxPkt.payload[3] & 0x20;
//   pbstate.demo_button = rxPkt.payload[3] & 0x40;
//   pbstate.estop_button = rxPkt.payload[3] & 0x80;
// }

// gyro globals
double cal_offset = 140;
double rate = 0;
double orientation = 0;

bool isMoving = false;

#define STEERMULT 400
#define VELMULT 400
#define MAXSPEED 1000

void velCallback(const geometry_msgs::Twist& velMsg)
{
  //assume linear and twist will be from -1.0 to 1.0
  //scale to -1000 to 1000
  //scale first to help avoid saturation
  int16_t linear = std::min(MAXSPEED,std::max(-MAXSPEED,(int)(VELMULT*velMsg.linear.x)));
  int16_t angular = std::min(MAXSPEED,std::max(-MAXSPEED,(int)(STEERMULT*velMsg.angular.z)));
  int16_t right_pwm = std::min(MAXSPEED,std::max(-MAXSPEED,linear+angular));  
  int16_t left_pwm = std::min(MAXSPEED,std::max(-MAXSPEED,linear-angular));  
  std::cout<<"sending motorpwm: L: "<<left_pwm<<"  R: "<<right_pwm<<std::endl;
  sendMotorPWM(left_pwm,right_pwm);
}


//TODO: change this a lot!
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

void printGryo(packet_t rxPkt){
  int16_t temp,x,y,z;
  temp = rxPkt.payload[1]+(rxPkt.payload[0]<<8);
  x = rxPkt.payload[3]+(rxPkt.payload[2]<<8);
  y = rxPkt.payload[5]+(rxPkt.payload[4]<<8);
  z = rxPkt.payload[7]+(rxPkt.payload[6]<<8);
  printf("Gyro Reading: T: %6i  x: %6i  y: %6i  z: %6i",temp,x,y,z);
  std::cout<<std::endl;
  
}

void printEncoder(packet_t rxPkt){
  int16_t left_pwm,right_pwm;
  left_pwm = rxPkt.payload[0]+(rxPkt.payload[1]<<8);
  right_pwm = rxPkt.payload[2]+(rxPkt.payload[3]<<8);
  printf("PWM Feedback: R: %6i  L: %6i  ",left_pwm,right_pwm);
  std::cout<<std::endl;
  
}


//TODO: make calibration routine for:
//  IMU
//  Wheel encoder
//  motor driver


int main(int argc, char **argv)
{
    serial = new Serial("/dev/ttyACM0");
    packet_t rxPkt;
    packet_t* txPkt;
    status_t status;
    status.recvd = 0;
    bilibot_node::MotorState pbstate;
    boost::circular_buffer<float> calibration(140);  //TODO: still used?

    if (serial->openPort() < 0) {
        ROS_ERROR("unable to open port\n");
        return -1;
    }

    ros::init(argc, argv, "bilibot_node");

    ros::NodeHandle n;

    // fill in imu data (start at 0 orientatino)
//     imu.orientation = tf::createQuaternionMsgFromYaw(0.0);  //TODO: initiate from accelerometer
//     imu.header.stamp = ros::Time::now();
//     imu.header.frame_id = "gyro_link";
//     boost::array<double, 9> cov = {{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6}}; 
//     boost::array<double, 9> cov2 = {{-1, 0, 0, 0, 0, 0, 0, 0, 0}}; 
//     std::copy(cov.begin(), cov.end(), imu.orientation_covariance.begin());
//     std::copy(cov.begin(), cov.end(), imu.angular_velocity_covariance.begin());
//     std::copy(cov2.begin(), cov2.end(), imu.linear_acceleration_covariance.begin());

//     ros::ServiceServer hand_state_service = n.advertiseService("toggle_hand_state", toggleHandState);
//     ros::ServiceServer arm_pos_service = n.advertiseService("set_arm_pos", setArmPosition);
    ros::ServiceServer kinect_pwr_service = n.advertiseService("toggle_kinect_power", toggleKinectPower);
    ros::Publisher motor_state_pub = n.advertise<bilibot_node::MotorState>("motor_state", 1);
//     ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 1);
    ros::Subscriber vel_sub = n.subscribe("cmd_vel", 1, velCallback);

    uint8_t position = 255;
    txPkt = PKT_Create(PKTYPE_CMD_SET_ARM_POS, 0, &position, 1);
    sendPacket(serial, txPkt);
    free(txPkt);

    while(ros::ok())
    {
        uint8_t byte = serial->readByte();
	std::cout<<".";
        if(PKT_Decoded(byte, &rxPkt, &status) != DECODE_STATUS_INCOMPLETE) {
            switch(status.state)
            {
            case DECODE_STATUS_COMPLETE: 
                switch (rxPkt.type)
                {
                case PKTYPE_STATUS_HEARTBEAT:
                    break;
                case PKTYPE_STATUS_ARM_STATE:
//                     updateArmState(pbstate,rxPkt);
//                     sensor_state_pub.publish(pbstate);
                    break;
                case PKTYPE_STATUS_MOTOR_STATE:
                     updateMotorState(pbstate,rxPkt);
                     motor_state_pub.publish(pbstate);
                    break;
                case PKTYPE_STATUS_3GYRO_RAW: 
		  printGryo(rxPkt);
//                     updateGyroState(imu, rxPkt, calibration);
			//TODO: fill this out!S
//                     pbstate.gyro_raw = rxPkt.payload[0];
//                     imu_pub.publish(imu);
                    break;
                case PKTYPE_STATUS_ENCODER_RAW:
		  std::cout<<std::endl;
		  printEncoder(rxPkt);
// 		  ROS_INFO("encoder state not supported...");
//                     updateGyroState(imu, rxPkt, calibration);
			//TODO: fill this out!S
//                     pbstate.gyro_raw = rxPkt.payload[0];
//                     imu_pub.publish(imu);
                    break;
		    
                case PKTYPE_STATUS_BATT_RAW:
//                     updateBattState(pbstate,rxPkt);
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



// bool toggleHandState(std_srvs::Empty::Request  &req,
//             std_srvs::Empty::Response &res )
// {
//     ROS_INFO("toggling hand state");
//     uint8_t unused = 0;
//     packet_t* txPkt = PKT_Create(PKTYPE_CMD_TOGGLE_HAND_STATE, 0, &unused, 1);
//     sendPacket(serial, txPkt);
//     free(txPkt);
//     return true;
// }

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

// bool setArmPosition(bilibot_node::SetArmPosition::Request  &req,
//             bilibot_node::SetArmPosition::Response &res )
// {
//     ROS_INFO("setting arm postion to %d", req.position );
//     packet_t* txPkt = PKT_Create(PKTYPE_CMD_SET_ARM_POS, 0, &req.position, 1);
//     sendPacket(serial, txPkt);
//     free(txPkt);
//     return true;
// }

void sendMotorPWM(int16_t left_pwm, int16_t right_pwm){
    uint8_t payload[4];
    //convert to packets  
    memcpy(payload,&right_pwm,2);
    memcpy(payload+2,&left_pwm,2); 
    packet_t* txPkt = PKT_Create(PKTYPE_CMD_SET_BASE_VEL, 0, payload, 4);
    
    printf("PWM payload: R: %6i -> %6i %6i  L:%6i -> %6i %6i  \n",right_pwm, payload[0],payload[1],left_pwm,payload[2],payload[3] );
    sendPacket(serial, txPkt);
    free(txPkt); 
}



bool setMotorPWM(bilibot_node::SetMotorPWM::Request  &req,
            bilibot_node::SetMotorPWM::Response &res )
{
//     ROS_INFO("setting arm postion to %d", req.position );
    
  
 
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

