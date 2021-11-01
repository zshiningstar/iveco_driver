#include <ros/ros.h>
#include <iostream>
#include<assert.h>

#include <can2serial/can2serial.h>
#include <serial/serial.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <std_msgs/UInt64.h>
#include <std_msgs/String.h>

#include <driverless_common/VehicleCtrlCmd.h>
#include <driverless_common/VehicleState.h>

/*
	档位状态描述:
	0x1: Parking
	0x2: Driving
	0x3: Reverse
	0x4: Neutral
	
	控制模式描述:
	0x0: 无效
	0x1: 自动驾驶模式
	0x2: 手动驾驶模式
	0x3: 遥控器模式
	0x4: 保留
	
	驻车状态描述:
	0x0: 无效
	0x1: 驻车
	0x2: 非驻车
	0x3: 保留
*/

#define ID_CMD_STEER			   0x0C01D02A
#define ID_STATE_STEER			   0x0C012AD0

#define ID_CMD_EBS				   0x0C02D02A
#define ID_STATE_EBS			   0x0C022AD0

#define ID_CMD_VCU				   0x0C03D02A
#define ID_STATE_VCU			   0x0C032AD0

#define	ID_STATE_SPEED			   0x0218A006

#define ID_CMD_BCM				   0x0C04D02A
#define ID_STATE_BCM			   0x0C042AD0


#define STEERANGLE_ROADWHEELANGLE_RATIO  25.0

/**************************定义*************************/
class BaseControl
{
public:
	BaseControl();
	~BaseControl();
	bool init(int ,char**);
	void run();
	
	void parse_CanMsg();//读取CAN总线数据,解析车辆状态信息
	void timer_callBack(const ros::TimerEvent& event);//定时发布车辆状态信息
	void vehicleCtrlCmd_callback(const driverless_common::VehicleCtrlCmd::ConstPtr msg);//上位机控制指令回调函数
	
private:
	Can2serial can2serial;
	
//	bool manualCtrlDetected_; //是否检测到驾驶员介入
//	bool is_driverless_mode_; //是否为自动驾驶模式(实际值)
//	bool allow_driverless_;   //是否允许进入自动驾驶模式
	bool default_drive_gear_; //是否默认进入自动驾驶模式后为前进档
	
	boost::shared_ptr<boost::thread> readFromStm32_thread_ptr_; 
	
	ros::Subscriber vehicleCtrlCmd_sub_;//订阅上位机指令
	ros::Publisher vehicleState_pub_;//发布车辆状态信息
	ros::Timer timer_;//定时发布车辆状态信息
	
	std::string obd_can_port_name_;//can端口号
	
	float max_steering_speed_;  //Front and rear frame maximun steering angle difference
	int steering_offset_; 
	
	CanMsg_t canMsg_cmd1_;
	CanMsg_t canMsg_cmd2_;
	CanMsg_t canMsg_cmd3_;
	CanMsg_t canMsg_cmd4_;
	
	driverless_common::VehicleState vehicleState_;
	driverless_common::VehicleCtrlCmd vehicleCtrlCmd_;
	
	boost::mutex mutex_;
	
	bool wheel_speed_FL_valid;
	bool wheel_speed_FR_valid;
	bool wheel_speed_RL_valid;
	bool wheel_speed_RR_valid;
	
	float wheel_speed_RR;
	float wheel_speed_RL;
	float wheel_speed_FL;
	float wheel_speed_FR;
	
	float steer_angle;
	float curGearState;
	float curMotorTorque;
	
	//刹车状态信息
	float curDeceleration;
	float curBrakePedal0penPercentage;
	uint8_t curEPBState;
	uint8_t curEBSState;
	
	//车门状态
	uint8_t curDoorStatus;
	
	//转向状态
	float curSteerWheelAngle;
	float curSteerWheelAngleSpeed;
	
	//控制模式
	uint8_t curControlMode;
};
/*************************定义*************************/

BaseControl::BaseControl()
{
	canMsg_cmd1_.ID = ID_CMD_EBS;
    canMsg_cmd1_.len = 8;
    canMsg_cmd1_.type = Can2serial::EXT_DATA_FRAME; //EXT frame;
    
    *(long *)canMsg_cmd1_.data = 0;

    canMsg_cmd2_.ID = ID_CMD_STEER;
    canMsg_cmd2_.len = 8;
    canMsg_cmd2_.type = Can2serial::EXT_DATA_FRAME;//EXT frame;
    
    *(long *)canMsg_cmd2_.data = 0;
    canMsg_cmd2_.data[4] = 0x00;//set the steer angle value invalid
    
	canMsg_cmd3_.ID = ID_CMD_VCU;
    canMsg_cmd3_.len = 8;
    canMsg_cmd3_.type = Can2serial::EXT_DATA_FRAME; //EXT frame;
    
    *(long *)canMsg_cmd3_.data = 0;
    
	canMsg_cmd4_.ID = ID_CMD_BCM;
    canMsg_cmd4_.len = 8;
    canMsg_cmd4_.type = Can2serial::EXT_DATA_FRAME; //standard frame;
    
    *(long *)canMsg_cmd4_.data = 0;
}

BaseControl::~BaseControl()
{
	
}

bool BaseControl::init(int argc,char**argv)
{
	ros::init(argc,argv,"base_control");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	nh_private.param<std::string>("obd_can_port_name", obd_can_port_name_, "/dev/ttyUSB0");
	nh_private.param<float>("max_steering_speed",max_steering_speed_,2.0); //通过限制前后帧转角命令差值 控制转向最大速度**
	nh_private.param<int>("steering_offset",steering_offset_,0);
	nh_private.param<bool>("default_drive_gear", default_drive_gear_, true);//是否默认为前进档
	
	assert(!obd_can_port_name_.empty());
	assert(max_steering_speed_>0);

	vehicleCtrlCmd_sub_ = nh.subscribe("/VehicleCtrlCmd",1,&BaseControl::vehicleCtrlCmd_callback,this);
	vehicleState_pub_ = nh.advertise<driverless_common::VehicleState>("/VehicleState",10);
	
	timer_ = nh.createTimer(ros::Duration(0.03), &BaseControl::timer_callBack, this);
	
	if(!can2serial.configure_port(obd_can_port_name_.c_str()))
	{
		ROS_INFO("open port %s failed",obd_can_port_name_.c_str());
		return false;
	}
	else
		ROS_INFO("open port %s successfully",obd_can_port_name_.c_str());
	
	can2serial.clearCanFilter();
	/*
	can2serial.setCanFilter_alone(0x01,ID_STATE1); usleep(10000);
	can2serial.setCanFilter_alone(0x02,ID_STATE2); usleep(10000);
	can2serial.setCanFilter_alone(0x03,ID_STATE3); usleep(10000);
	can2serial.setCanFilter_alone(0x04,ID_STATE4); usleep(10000);
	*/
	can2serial.configBaudrate(500);
	
	can2serial.StartReading();
		
	ROS_INFO("System initialization completed");
	
	//usleep(1500000);
	
	//can2serial.clearCanFilter();
	return true;
}

void BaseControl::run()
{
	boost::thread parse_msg(boost::bind(&BaseControl::parse_CanMsg,this)); 
	ros::spin();
}

void BaseControl::parse_CanMsg()
{
	CanMsg_t canMsg;

	while(ros::ok())
	{
//		ROS_INFO("parse_CanMsg  ing.....");
		usleep(3000);
		if(!can2serial.getCanMsg(canMsg))
		{
			//ROS_INFO("nothing....");
			continue;
		}
			
//		ROS_INFO("ID:%x",canMsg.ID);
//		can2serial.showCanMsg(canMsg);
		
		bool key = true;
			
		switch(canMsg.ID)
		{
			case ID_STATE_VCU:
				curMotorTorque = canMsg.data[1] * 256 + canMsg.data[0] - 1000;
				curGearState = canMsg.data[2]&0x03;
				curControlMode = canMsg.data[5];//协议中的字节5与6反了
				vehicleState_.driverless = (curControlMode == 0x01) ? true : false;
				if(curGearState == 0x0)
					vehicleState_.gear = driverless_common::VehicleState::GEAR_PAKING;
				else if(curGearState == 0x1)
					vehicleState_.gear = driverless_common::VehicleState::GEAR_DRIVE;
				else if(curGearState == 0x2)
					vehicleState_.gear = driverless_common::VehicleState::GEAR_REVERSE;
				else if(curGearState == 0x3)
					vehicleState_.gear = driverless_common::VehicleState::GEAR_NEUTRAL;
				break;
				
			case ID_STATE_SPEED:
				
				wheel_speed_RR_valid = canMsg.data[0]&0x80;
				wheel_speed_RR       = ((canMsg.data[0]&0x1f << 8) + canMsg.data[1]) * 0.0625;
				
				wheel_speed_RL_valid = canMsg.data[2]&0x80;
				wheel_speed_RL       = ((canMsg.data[2]&0x1f << 8) + canMsg.data[3]) * 0.0625;
				
				wheel_speed_FR_valid = canMsg.data[4]&0x80;
				wheel_speed_FR       = ((canMsg.data[4]&0x1f << 8) + canMsg.data[5]) * 0.0625;
				
				wheel_speed_FL_valid = canMsg.data[6]&0x80;
				wheel_speed_FL       = ((canMsg.data[6]&0x1f << 8) + canMsg.data[7]) * 0.0625;
				
				{
				size_t i =0;
				float speed = 0.0;
				if(wheel_speed_FL_valid==false) {i++; speed += wheel_speed_FL;}
				if(wheel_speed_FR_valid==false) {i++; speed += wheel_speed_FR;}
				if(wheel_speed_RL_valid==false) {i++; speed += wheel_speed_RL;}
				if(wheel_speed_RR_valid==false) {i++; speed += wheel_speed_RR;}
				
				vehicleState_.speed = speed/i; //km/h
				}
				break;
	
			case ID_STATE_STEER:
				curSteerWheelAngle = ((canMsg.data[1]&0x0f) << 8) + canMsg.data[0] - 2000;//优先级注意
				std::cout << curSteerWheelAngle << std::endl;
				
				vehicleState_.roadwheel_angle = curSteerWheelAngle;
				curSteerWheelAngleSpeed = ((canMsg.data[3]&0x0f) << 8) + canMsg.data[2];
				curControlMode = canMsg.data[4]&0x07;
				break;
			
			case ID_STATE_EBS:
				curDeceleration = canMsg.data[0];
				curBrakePedal0penPercentage = canMsg.data[1];
				curEPBState = canMsg.data[2]&0x01;
				vehicleState_.hand_brake = curEPBState;
				curEBSState = (canMsg.data[2] >>1)&0x07; 
				break;
				
			case ID_STATE_BCM:
				curDoorStatus = canMsg.data[0]&0x01;
				break;
				
			default:
				key = false;
				break;
		}
		if(key)
			can2serial.showCanMsg(canMsg);
	}
}

void BaseControl::timer_callBack(const ros::TimerEvent& event)
{
	vehicleState_.emergency_brake = 0;
	vehicleState_pub_.publish(vehicleState_);
}

void BaseControl::vehicleCtrlCmd_callback(const driverless_common::VehicleCtrlCmd::ConstPtr msg)
{
	uint8_t set_gear 				= msg->gear;
	float set_speed 				= msg->speed;
	float set_brake 				= msg->brake; //刹车
	float set_roadwheel_angle 		= msg->roadwheel_angle;//应该由前轮转角转换为方向盘转角
	bool set_left_turn_light 		= msg->left_turn_light;
	bool set_right_turn_light 		= msg->right_turn_light;
	bool set_brake_light 			= msg->brake_light; //制动灯
	bool set_horn 					= msg->horn; //喇叭
	bool set_emergency_brake		= msg->emergency_brake; //紧急制动
	bool set_hand_brake 			= msg->hand_brake; //手刹
	bool set_driverless 			= msg->driverless;
	
	//发送EBS(刹车)指令
	canMsg_cmd1_.data[1] = 0x00;
	canMsg_cmd1_.data[0] = set_brake;//(此处应该为减速度)
	if(set_hand_brake)//符合刹车条件,电子驻车
		canMsg_cmd1_.data[1] = 0x01;
	else
		canMsg_cmd4_.data[1] = 0x00;
	can2serial.sendCanMsg(canMsg_cmd1_);
	
	//发送转向指令
	canMsg_cmd2_.data[0] = uint16_t(set_roadwheel_angle + 2000);
	canMsg_cmd2_.data[1] = uint16_t(set_roadwheel_angle + 2000) >> 8;
	
	canMsg_cmd2_.data[2] = uint16_t(max_steering_speed_);
	canMsg_cmd2_.data[3] = uint16_t(max_steering_speed_) >> 8;
	if(set_driverless)
		canMsg_cmd2_.data[4] = 0x1;
	else
		canMsg_cmd2_.data[4] = 0x2;//手动
	can2serial.sendCanMsg(canMsg_cmd2_);
	
	//发送驱动指令
	canMsg_cmd3_.data[0] = set_gear;
	canMsg_cmd3_.data[1] = set_speed;//此处需要转换成油门开度
	if(set_driverless)
		canMsg_cmd3_.data[2] = 0x1;
	else
		canMsg_cmd3_.data[2] = 0x2;//手动
	can2serial.sendCanMsg(canMsg_cmd3_);
	
	//BCM相关指令(开关)
	canMsg_cmd4_.data[0] |= 0x00;//车门关
	if(set_left_turn_light)
		canMsg_cmd4_.data[0] |= 0x02;
	else
		canMsg_cmd4_.data[0] &= 0xfd;
		
	if(set_right_turn_light)
		canMsg_cmd4_.data[0] |= 0x04;
	else
		canMsg_cmd4_.data[0] &= 0xfb;
	if(set_brake_light)
		canMsg_cmd4_.data[0] |= 0x08;
	else
		canMsg_cmd4_.data[0] &= 0xf7;
	canMsg_cmd4_.data[1] = 0x0;//双语播报
	canMsg_cmd4_.data[2] = 0x1;//nessage1

	can2serial.sendCanMsg(canMsg_cmd4_);

}

int main(int argc,char**argv)
{
	BaseControl base_control;
	
	if(base_control.init(argc,argv))
		base_control.run();
	
	printf("base_control_node has exited");
	return 0;
}

