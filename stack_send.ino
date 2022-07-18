#include <M5Core2.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int64.h>
// #include <robot_data/msg/tag_msg.h>
#include <geometry_msgs/msg/pose.h>

#include <Wire.h>           // i2c to connect to IR communication board.

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define ROBOT_I2C_ADDR  8

#define WIDTH 320
#define HEIGHT 240

// Data to send(tx) and receive(rx)
// on the i2c bus.
// Needs to match the master device

#pragma pack(1)
typedef struct i2c_status {
  float x;                  // 4 bytes
  float y;                  // 4 bytes
  float theta;              // 4 bytes
  uint8_t status;           // 1 byte
} i2c_status_t;
#pragma pack()

i2c_status_t i2c_status_tx;
i2c_status_t i2c_status_rx;

rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
geometry_msgs__msg__Point point_msg;
geometry_msgs__msg__Pose pose_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// robot_data__msg__TagMsg msg;

void error_loop(){
  M5.lcd.println("\nFatal error.");
  while(1){
    delay(25565);
  }
}

void drawMarker(u_int64_t data) {
  int size = HEIGHT / 6;
  int inset = (WIDTH - HEIGHT) / 2;
  for (u_int64_t i = 0; i < 36; i++) {
    bool white = (data & ((u_int64_t)1 << i)) != 0;

    int x = inset + (i % 6) * size;
    int y = (i / 6) * size;
    if (white) {
      M5.lcd.fillRect(x, y, size, size, WHITE);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  
  //M5.begin(true, false, true, true);
  M5.begin();
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.begin();  
  drawMarker(0x000000056A56A56A);
  // M5.lcd.println("\nSetting up I2C aaa");

  // M5.lcd.println("\na");

  set_microros_wifi_transports("TP-Link_102C", "35811152", "192.168.0.230", 8888);

  //  M5.lcd.println("b");

  delay(2000);

  allocator = rcl_get_default_allocator();

  //  M5.lcd.println("c");

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  //  M5.lcd.println("d");

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  //  M5.lcd.println("e");

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "topic_name"));

  msg.data = 0;

  // M5.lcd.println("\ndone");
}

void loop() {

  // Setup data to send to robot


  // Send an update down to the robot
  
  i2c_status_tx.x = 987;
  i2c_status_tx.y = 654;
  i2c_status_tx.theta = 6.28;
  i2c_status_tx.status++;
  
  Wire.beginTransmission(ROBOT_I2C_ADDR);
  Wire.write( (uint8_t*)&i2c_status_tx, sizeof( i2c_status_tx ));
  Wire.endTransmission();


  // Read values back from the robot.
  Serial.println("Read: ");
  Wire.requestFrom( ROBOT_I2C_ADDR, sizeof( i2c_status_rx ));
  Wire.readBytes( (uint8_t*)&i2c_status_rx, sizeof( i2c_status_rx ));
  printRXStatus();

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  msg.data++;

  // M5.lcd.println("\ndid");
  

  delay(500);
}

void printRXStatus() {
  Serial.println( i2c_status_rx.x ); 
  Serial.println( i2c_status_rx.y );
  Serial.println( i2c_status_rx.theta );
  Serial.println( i2c_status_rx.status );
}
