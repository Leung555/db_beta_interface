#ifndef DB_BETA_POSITION_CONTROLLER_H
#define DB_BETA_POSITION_CONTROLLER_H 

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

// core ros2 library
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

// // Dynamixel workbench, msgs library
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/msg/dynamixel_state_list.h>
#include <dynamixel_workbench_msgs/srv/dynamixel_command.h>

// // Messages
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define BAUDRATE 4000000  // Default Baudrate of DYNAMIXEL X series of Db_Alpha
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

#define PROTOCOL_VERSION 2.0

class db_beta_interface : public rclcpp::Node
{
public:
    db_beta_interface();
    virtual ~db_beta_interface();

	// Dynamixel Workbench Parameters
	DynamixelWorkbench* dxl_wb;

	int motor_cnt = 0;
	int present_position;

    double read_period;
    double write_period;
    double publish_period;

	// Info of each motor
	std::vector<std::string> name_array;
	std::vector<int> id_array;
	std::vector<float> pos_array;
	std::vector<float> vel_array;
	std::vector<float> current_array;

	// Control Item for syncRead & syncWrite
	std::map<std::string, const ControlItem*> control_items;

	// Map between motor_name and motor_id
	std::map<std::string, uint32_t> motorNameIDPair;

	// Initialization 
	bool initWorkbench(const char* port_name, const uint32_t baudrate);
	bool scanMotors(uint8_t scanned_id[], uint8_t dxl_cnt, uint8_t range);
	void showDynamixelInfo();
	void initMsg();
	void initSyncReadWriteHandler();
	void setupDynamixel();
	bool initControlItems();
	void reboot();
	void disTorqueAllMotors();
	// bool getDynamixelsInfo(const std::string yaml_file);
	// bool loadDynamixels();
	void readDxl_publish_callback();
	void writeDxl_subscribe_callback(const sensor_msgs::msg::JointState & msg);
		// bool initDynamixels();
		// bool initSDKHandlers();
		// bool initHomePosition();
	
		// double getReadPeriod() { return read_period; }
		// double getWritePeriod() { return write_period; }
		// double getPublishPeriod() { return publish_period; }
	
		// void initPublisher();
		// void initSubscriber();
	
		// void initServer();
	
		// void readCallback(const ros::TimerEvent&);
		// void writeCallback(const ros::TimerEvent&);
		// void writeMultiCallback(const ros::TimerEvent&);
		// void publishCallback(const ros::TimerEvent&);
	
		// void onJointStateGoal(const sensor_msgs::JointState& msg);
		// void multiJointGoal(const std_msgs::Float32MultiArray& msg);
	
		// bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req, dynamixel_workbench_msgs::DynamixelCommand::Response &res);
		// bool dynamixelRebootCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
		// bool dynamixelTorqueOffCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
		// bool dynamixelTorqueOnCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

		// Joint configuration vector (format [motor1_ID, motor1_VALUE, motor2_ID, motor2_VALUE, ... , motor21_ID, motor21_VALUE])
		// std::vector<float> joint_configuration = {0,0,0,0,0,0, 
		// 									 	  0,0,0,0,0,0, 
		// 									 	  0,0,0,0,0,0,
		// 									 	  0,0,0,0,0,0,
		// 									 	  0,0,0,0,0,0,
		// 									 	  0,0,0,0,0,0,
		// 									 	  0,0,0,0,0,0};
    using Jointstate = sensor_msgs::msg::JointState;

	rclcpp::Publisher<Jointstate>::SharedPtr jointstates_publisher;
	// rclcpp::Publisher<std_msgs::msg::String>::SharedPtr jointstates_publisher;
	rclcpp::Subscription<Jointstate>::SharedPtr set_position_subscriber_;
	// rclcpp::Subscription<std_msgs::msg::String>::SharedPtr set_position_subscriber_;

	rclcpp::TimerBase::SharedPtr timer_;
	size_t count_;

    // portHandler_ = DynamixelDriver::dynamixel::PortHandler::getPortHandler();
    // packetHandler_ = DynamixelDriver::dynamixel::PacketHandler::getPacketHandler();

private:
};


#endif //DB_BETA_POSITION_CONTROLLER_H
