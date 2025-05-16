#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

Servo servox;
Servo servoy;
ros::NodeHandle nh;
int servo_x_position = 0;
int servo_y_position = 0;
void setServoX(const std_msgs::Int16 &msg) {
  servo_x_position = msg.data;
  servox.write(servo_x_position);
}

void setServoY(const std_msgs::Int16 &msg) {
  servo_y_position = msg.data;
  servoy.write(servo_y_position);
}

ros::Subscriber<std_msgs::Int16> sub_servo_x("/servo_x", setServoX);
ros::Subscriber<std_msgs::Int16> sub_servo_y("/servo_y", setServoY);

void setup() {
  Serial.begin(57600);
  servox.attach(3);  
  servoy.attach(5); 
  servox.write(90);
  servoy.write(90);
  delay(1000);
  servox.write(0);
  servoy.write(0);
  nh.initNode();
  nh.subscribe(sub_servo_x);
  nh.subscribe(sub_servo_y);
}

void loop() {
  nh.spinOnce();
  delay(10); 
}
