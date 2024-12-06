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
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 37
#define ADDR_HARDWARE_ERROR 50

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
// #define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series of Db_Alpha
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
  const char* port_name = DEVICE_NAME;
  uint32_t baudrate = BAUDRATE;
  uint8_t scanRange = 73;  

  const char *log;
  bool result = false;

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  result = portHandler->openPort();
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to open the port!\n");

    return 0;
  }
  else{
    printf("Success to open the port!\n");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to set the baudrate!\n");

    return 0;
  }    
  else{
    printf("Success to set the baudrate!\n");
  }  
  ErrorFromSDK sdk_error = {0, false, false, 0};
  
  for (int dxl_id = 1; dxl_id < scanRange+1; dxl_id++){

    sdk_error.dxl_comm_result = packetHandler->ping(portHandler, dxl_id, &sdk_error.dxl_error);
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      // printf("Cannot ping motor id: %d\n", dxl_id);
    }
    else
    {
      printf("Success ping motor id: %d\n", dxl_id);
      // Read Hardware error status
      uint8_t data_1_byte  = 0;
      sdk_error.dxl_comm_result = packetHandler->read1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_HARDWARE_ERROR,
        &data_1_byte,
        &sdk_error.dxl_error
      );
      if (sdk_error.dxl_comm_result != COMM_SUCCESS)
      {
        // printf("Cannot read Hardware Error Status! id: %d\n", dxl_id);
      }    
      else{
        // printf("Success read Hardware Error Status! id: %d\n", dxl_id);
      }
      
      // Check hardware error status if error then reboot
      // cout << "Hardware error status: " << +data_1_byte << endl;
      if (data_1_byte != 0){
        cout << "Detect Hardware error status: " << +data_1_byte << endl;
        sdk_error.dxl_comm_result = packetHandler->reboot(portHandler, dxl_id, &sdk_error.dxl_error);
        if (sdk_error.dxl_comm_result != COMM_SUCCESS)
        {
          printf("Cannot reboot motor id: %d\n", dxl_id);
        }    
        else{
          printf("Success reboot motor id: %d\n", dxl_id);
        }
      }
      else{
        cout << "Hardware work fine" << endl;

      }
    }
  }



    return 0;
}