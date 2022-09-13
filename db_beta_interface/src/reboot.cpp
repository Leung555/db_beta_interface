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
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

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

void db_beta_interface::reboot(){
  const char *log;
  bool result = false;

  for (int cnt = 0; cnt < motor_cnt; cnt++){
    printf("id: %d", id_array[cnt]);
    result = dxl_wb->reboot(id_array[cnt], &log);
    if (result == false){
      printf("%s\n", log);
      printf("Failed to reboot\n");
    }
    else{
      printf("Succeed to reboot\n");
      cout << "id : " << id_array[cnt] << 
            ", model name : "<< name_array[cnt] <<"\n";
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

int main(int argc, char * argv[]){
    // initialize portHandler & packetHandler
    rclcpp::init(argc, argv);

    auto db_beta = std::make_shared<db_beta_interface>();
    
    const char *log;
    bool result = false;
    
    // set operating mode & Enable Torque
    const char* port_name = DEVICE_NAME;
    uint32_t baudrate = BAUDRATE;
    result = db_beta->initWorkbench(port_name, baudrate);
    // cout << result <<endl;
    if (result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("db_beta_node"), "Please check USB port name, baudrate");
      return 0;
    }
    db_beta->initMsg();

    // Find dynamixel Motors
    if (scanDynamixelMode){
      // uint32_t baudrate[BAUDRATE_NUM] = {9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000};
      uint8_t range = 73;
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

    // Check dynamixel data collection
    db_beta->reboot();

    return 0;
}