#include <chrono>
#include <functional>

#include <cstdio>
#include <memory>
#include <string>
#include <iostream>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "db_beta_interface.h"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
// #define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define BAUDRATE 4000000  // Default Baudrate of DYNAMIXEL X series of Db_Alpha
#define DEVICE_NAME "/dev/U2D2"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

bool scanDynamixelMode = true;

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;
// DynamixelWorkbench * dxl_wb;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;

db_beta_interface::db_beta_interface()
: Node("db_beta_interface"), count_(0)
{
    // initiate Dynamixel workbench
    dxl_wb = new DynamixelWorkbench;

    RCLCPP_INFO(this->get_logger(), "Run db_beta_interface node");

    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    // set up Publisher
    jointstates_publisher = this->create_publisher<Jointstate>("motor_feedback", 10);
    timer_ = this->create_wall_timer(
    10ms, std::bind(&db_beta_interface::readDxl_publish_callback, this));


    // set up subscriber
    set_position_subscriber_ = this->create_subscription<Jointstate>(
    "motor_command", 10, std::bind(&db_beta_interface::writeDxl_subscribe_callback, this, _1));


}

db_beta_interface::~db_beta_interface()
{
}

bool db_beta_interface::initWorkbench(const char* port_name, const uint32_t baudrate)
{
  const char *log;
  bool result = false;

  result = dxl_wb->init(port_name, baudrate, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to init\n");
  }
  else
    printf("Succeed to init(%d)\n", baudrate);  
}

bool db_beta_interface::scanMotors(uint8_t scanned_id[], uint8_t dxl_cnt, uint8_t range)
{
  const char *log;
  bool result = false;

  for (uint8_t num = 0; num < 100; num++) scanned_id[num] = 0;

  printf("Wait for scan...\n");
  result = dxl_wb->scan(scanned_id, &dxl_cnt, range, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to scan\n");
  }
  else
  {
    printf("Find %d Dynamixels\n", dxl_cnt);
    this->motor_cnt = (int) dxl_cnt;

    for (int cnt = 0; cnt < dxl_cnt; cnt++){
      printf("id : %d, model name : %s\n", scanned_id[cnt], this->dxl_wb->getModelName(scanned_id[cnt]));
      // store info of each motor
      this->id_array.push_back(scanned_id[cnt]);
      string str1 = "motor_";
      string motor_name = str1.append(to_string(cnt+1));
      this->name_array.push_back(motor_name);
    }
  }
}

void db_beta_interface::showDynamixelInfo(){
  for (int cnt = 0; cnt < motor_cnt; cnt++){
    cout << "id : " << id_array[cnt] << 
            ", model name : "<< name_array[cnt] <<"\n";
  }
}

bool db_beta_interface::initControlItems(){
  const char *log;
  bool result = false;

	uint32_t dxl_id = id_array[0];

	const ControlItem* goal_position = dxl_wb->getItemInfo(dxl_id, "Goal_Position");
	if (goal_position == NULL) return false;

	const ControlItem* present_position = dxl_wb->getItemInfo(dxl_id, "Present_Position");
	if (present_position == NULL) return false;

	const ControlItem* present_velocity = dxl_wb->getItemInfo(dxl_id, "Present_Velocity");
	if (present_velocity == NULL) return false;

	const ControlItem* present_current = dxl_wb->getItemInfo(dxl_id, "Present_Current");
	if (present_current == NULL) return false;

	control_items["Goal_Position"] = goal_position;

	control_items["Present_Position"] = present_position;
	control_items["Present_Velocity"] = present_velocity;
	control_items["Present_Current"] = present_current;

  return true;
}

void db_beta_interface::initSyncReadWriteHandler(){
  // adress for read&write for XM430-W350R
  const char *log;
  bool result = false;
  
	uint16_t write_start_adress = control_items["Goal_Position"]->address;
  uint16_t write_length = control_items["Goal_Position"]->data_length;
  printf("write_start_adress: %d  \n", write_start_adress);
  printf("write_length: %d  \n", write_length);

  result = dxl_wb->addSyncWriteHandler(write_start_adress, write_length, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to add sync write handler\n");
  }
  else{
    printf("Succeed to add sync write handler\n");  
  }
  
	uint16_t read_start_adress = std::min(control_items["Present_Position"]->address, 
                                        control_items["Present_Current"]->address);
  uint16_t read_length = control_items["Present_Position"]->data_length + 
                        control_items["Present_Velocity"]->data_length + 
                        control_items["Present_Current"]->data_length;
  // Print adress and data length for debugging
  // printf("start_address position: %d, start_address current: %d\n", 
  // control_items["Present_Position"]->address, 
  // control_items["Present_Current"]->address);
  // printf("data_length position: %d, data_length Vel: %d, data_length current: %d \n", 
  // control_items["Present_Position"]->data_length,
  // control_items["Present_Velocity"]->data_length, 
  // control_items["Present_Current"]->data_length);
  // printf("Failed to add sync read handler\n");
  
  result = dxl_wb->addSyncReadHandler(read_start_adress, read_length, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to add sync read handler\n");
  }
  else{
    printf("Succeed to add sync read handler\n");  
  }
}

void db_beta_interface::setupDynamixel()
{
  const char *log;
  bool result = false;

  for(int dxl_cnt=0; dxl_cnt < motor_cnt; dxl_cnt++){
    // Use Position Control Mode
    result = dxl_wb->setPositionControlMode( id_array[dxl_cnt], &log);
    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("db_beta_interface"), "Failed to set Position Control Mode of id: %d.", id_array[dxl_cnt]);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("db_beta_interface"), "Succeeded to set Position Control Mode.");
    }
  }
  for(int dxl_cnt=0; dxl_cnt < motor_cnt; dxl_cnt++){
    // Enable Torque of DYNAMIXEL
    result = dxl_wb->torqueOn(id_array[dxl_cnt], &log);
    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("db_beta_interface"), "Failed to enable torque of id: %d.", id_array[dxl_cnt]);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("db_beta_interface"), "Succeeded to enable torque.");
    }
  }
  for(int dxl_cnt=0; dxl_cnt < motor_cnt; dxl_cnt++){
    // Set up Velocity and Accelerration profile
    // result = dxl_wb->itemWrite(id_array[dxl_cnt], "LED", 1, &log);
    // if (result == false){
    //   printf("%s\n", log);
    //   printf("Failed to light up LED");
    // }
    result = dxl_wb->itemWrite(id_array[dxl_cnt], "Profile_Acceleration", 50, &log);
    if (result == false){
      printf("%s\n", log);
      printf("Failed to set Profile_Acceleration On id: %d\n", id_array[dxl_cnt]);
    }
    result = dxl_wb->itemWrite(id_array[dxl_cnt], "Profile_Velocity", 200, &log);
    if (result == false){
      printf("%s\n", log);
      printf("Failed to set Profile_Velocity On id: %d\n", id_array[dxl_cnt]);
    }

  }
  for(int dxl_cnt=0; dxl_cnt < motor_cnt; dxl_cnt++){
    // initialize Map between motor name and id
    motorNameIDPair[name_array[dxl_cnt]] = id_array[dxl_cnt];
    // cout << "motor name: " << name_array[dxl_cnt] << ",  motor id: " <<  +id_array[dxl_cnt] << endl;
  }
}

void db_beta_interface::disTorqueAllMotors()
{
  const char *log;
  bool result = false;

  for(int dxl_cnt=0; dxl_cnt < motor_cnt; dxl_cnt++){
    // Enable Torque of DYNAMIXEL
    result = dxl_wb->torqueOff(id_array[dxl_cnt], &log);
    if (result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("db_beta_interface"), "Failed to disable torque of id: %d.", id_array[dxl_cnt]);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("db_beta_interface"), "Succeeded to disable torque.");
    }
  }
}

void db_beta_interface::initMsg()
{
    printf("--------------------------------------------------------------------------\n");
    printf("\n"
        " __   __       __   ___ ___                 ___  ___  __   ___       __   ___ \n" 
        "|  \\ |__)     |__) |__   |   /\\      | |\\ |  |  |__  |__) |__   /\\  /  ` |__  \n"  
        "|__/ |__) ___ |__) |___  |  /~~\\ ___ | | \\|  |  |___ |  \\ |    /~~\\ \\__, |___ \n"          
        " __   __   __       __                __   __          ___  __ \n"               
        "|__) /  \\ /__`     |  \\ \\_/ |        |  \\ |__) | \\  / |__  |__) \n"              
        "|  \\ \\__/ .__/ ___ |__/ / \\ |___ ___ |__/ |  \\ |  \\/  |___ |  \\   \n"            
        "                                                                   \n");                                                                
    printf("--------------------------------------------------------------------------\n");
    printf("\n");
	printf("*******         POSITION CONTROLLER         *******");
	printf("\n");
	printf("--------------------------------------------------------------------------\n");
	printf("\n");
}

void db_beta_interface::readDxl_publish_callback()
{
  const char *log;
  bool result = false;

  Jointstate joints_message;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  // Read Multiple present position from motors
  uint8_t motor_cnt = this->motor_cnt;
	uint8_t id_array_uint8[motor_cnt];
	int32_t present_position[motor_cnt];
	int32_t present_velocity[motor_cnt];
	int32_t present_current[motor_cnt];

  for(int dxl_cnt =0; dxl_cnt< motor_cnt; dxl_cnt++){
    present_position[dxl_cnt] = (int32_t) 0;    
	  id_array_uint8[dxl_cnt] = (uint8_t) id_array[dxl_cnt];
  }

  const uint8_t handler_index = 0;

  result = dxl_wb->syncRead(handler_index, id_array_uint8, motor_cnt, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to sync read position\n");
  }

  result = dxl_wb->getSyncReadData(handler_index, id_array_uint8, motor_cnt, 
                                  control_items["Present_Position"]->address, 
                                  control_items["Present_Position"]->data_length,
                                  present_position,
                                  &log);
  if (result == false)  printf("%s\n", log);

  result = dxl_wb->getSyncReadData(handler_index, id_array_uint8, motor_cnt, 
                                  control_items["Present_Velocity"]->address, 
                                  control_items["Present_Velocity"]->data_length,
                                  present_velocity,
                                  &log);
  if (result == false)  printf("%s\n", log);

  result = dxl_wb->getSyncReadData(handler_index, id_array_uint8, motor_cnt, 
                                  control_items["Present_Current"]->address, 
                                  control_items["Present_Current"]->data_length,
                                  present_current,
                                  &log);
  if (result == false)  printf("%s\n", log);

  // pack data to Jointstate data
  joints_message.header.stamp = this->get_clock()->now();
  for(uint8_t dxl_cnt=0; dxl_cnt < motor_cnt; dxl_cnt++){
    joints_message.name.push_back(name_array[dxl_cnt]);
    joints_message.position.push_back(dxl_wb->convertValue2Radian(id_array[dxl_cnt], present_position[dxl_cnt]));
    joints_message.velocity.push_back(dxl_wb->convertValue2Velocity(id_array[dxl_cnt], present_velocity[dxl_cnt]));
    joints_message.effort.push_back(dxl_wb->convertValue2Current(present_current[dxl_cnt]));
  }

  jointstates_publisher->publish(joints_message);

}

void db_beta_interface::writeDxl_subscribe_callback(const Jointstate & msg)
{
  const char *log;
  bool result = false;
  const uint8_t handler_index = 0;

  // Read Multiple present position from motors
  uint8_t goal_motor_cnt_msg = msg.position.size();
	uint8_t goal_id_array_uint8[goal_motor_cnt_msg];
	int32_t goal_position[goal_motor_cnt_msg];
	// int32_t present_velocity[motor_cnt];
	// int32_t present_current[motor_cnt];
    cout << "goal_motor_cnt_msg: " << +goal_motor_cnt_msg << endl;

  for(int dxl_cnt = 0; dxl_cnt< goal_motor_cnt_msg; dxl_cnt++){
	  goal_id_array_uint8[dxl_cnt] = (uint8_t) motorNameIDPair[msg.name[dxl_cnt]];
    goal_position[dxl_cnt] = dxl_wb->convertRadian2Value(goal_id_array_uint8[dxl_cnt], msg.position[dxl_cnt]);    
    cout << "goal_id_array_uint8[dxl_cnt]: " << +dxl_cnt 
        << "  goal_position[dxl_cnt]" << +goal_position[dxl_cnt] << endl;
  }

  result = dxl_wb->syncWrite(handler_index, 
                              goal_id_array_uint8, 
                              goal_motor_cnt_msg, 
                              goal_position, 
                              1, //data_size (1 byte)
                              &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to sync write position\n");
  }
}

int main(int argc, char * argv[]){
    // initialize portHandler & packetHandler
    rclcpp::init(argc, argv);

    auto db_beta = std::make_shared<db_beta_interface>();

    // portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    // packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
    const char *log;
    bool result = false;

    // Open Serial Port
    // dxl_comm_result = portHandler->openPort();
    // if (dxl_comm_result == false) {
    //     RCLCPP_ERROR(rclcpp::get_logger("db_beta_node"), "Failed to open the port!");
    //     return -1;
    // } else {
    //     RCLCPP_INFO(rclcpp::get_logger("db_beta_node"), "Succeeded to open the port.");
    // }

    // // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    // dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    // if (dxl_comm_result == false) {
    //     RCLCPP_ERROR(rclcpp::get_logger("db_beta_node"), "Failed to set the baudrate!");
    //     return -1;
    // } else {
    //     RCLCPP_INFO(rclcpp::get_logger("db_beta_node"), "Succeeded to set the baudrate.");
    // }
    // cout << "test1: " <<endl;
    
    // set operating mode & Enable Torque
    const char* port_name = DEVICE_NAME;
    uint32_t baudrate = BAUDRATE;
    uint8_t scanRange = 73;
    // if (argc < 4)
    // {
    //   printf("Please set '-port_name', '-baud_rate', -scanRange arguments for connected Dynamixels\n");
    //   return 0;
    // }
    // else
    // {
    //   string port_name = argv[1];
    //   string baudrate  = argv[0];
    //   string scanRange = argv[0];
    // }
    // cout << "argc: " << argc << endl;
    // cout << "port_name: " << port_name << "  baudrate: " << baudrate << endl;
    // cout << "argv[3]: " << scanRange << "   " << endl;
    result = db_beta->initWorkbench(port_name, baudrate);
    // cout << result <<endl;
    if (result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("db_beta_node"), "Please check USB port name, baudrate");
      return 0;
    }

    // Find dynamixel Motors
    if (scanDynamixelMode){
      // uint32_t baudrate[BAUDRATE_NUM] = {9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000};
      uint8_t range = scanRange;
      uint8_t scanned_id[100];
      uint8_t dxl_cnt = 0;
      result = db_beta->scanMotors(scanned_id, dxl_cnt, range);
      if (result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("db_beta_node"), "scanMotors ERROR");
        return 0;
      } 
    }

    // store robot config data
    RCLCPP_INFO(db_beta->get_logger(), "Number of joints : %d", db_beta->motor_cnt);

    db_beta->setupDynamixel();
    db_beta->initMsg();

    // Check dynamixel data collection
    db_beta->showDynamixelInfo();
    result = db_beta->initControlItems();
    if (result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("db_beta_node"), "Unable to init Control Items");
      return 0;
    }

    db_beta->initSyncReadWriteHandler();
    // sleep(1);

    // Loop ROS node
    rclcpp::spin(db_beta);
    rclcpp::shutdown();

    // rclcpp::sleep_for(std::chrono::seconds(3));
    // Disable Torque of DYNAMIXEL
    db_beta->disTorqueAllMotors();
    // packetHandler->write1ByteTxRx(
    //   portHandler,
    //   BROADCAST_ID,
    //   ADDR_TORQUE_ENABLE,
    //   0,
    //   &dxl_error
    // );

    return 0;
}