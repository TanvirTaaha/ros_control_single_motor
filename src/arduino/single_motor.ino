#include <ros.h>
#include <std_msgs/Int32.h>
#include <ros_control_single_motor/JointStateMsg.h>
#include <Servo.h>

#define BIGGEST_POS 506
#define SMALLEST_POS 150

#define POS_PIN A0
#define SERVO_PIN 3

ros::NodeHandle nh;

int32_t cmd_angle;
Servo servo;

void cmd_angle_cb(const std_msgs::Int32& cmd_msg) {
  cmd_angle = cmd_msg.data;
  if (cmd_angle < 0 || cmd_angle > 180) {
    char buffer[20];
    sprintf(buffer, "OUT_OF_LIMIT:%d", cmd_angle);
    nh.logwarn(buffer);
  }
  servo.write(cmd_angle);
}

ros_control_single_motor::JointStateMsg pub_msg;

ros::Publisher pub("joint_state_from_arduino", &pub_msg);
ros::Subscriber<std_msgs::Int32> sub("/ros_control_to_arduino", cmd_angle_cb);

uint64_t last_time = 0;
int32_t last_pos = 0;

void publish_angle() {
  int32_t cur_pos = analogRead(POS_PIN);
  cur_pos = map(cur_pos, SMALLEST_POS, BIGGEST_POS, 0, 180);
  uint64_t cur_time = millis();
  double vel = (cur_pos - last_pos) * (1000) / (cur_time - last_time);
  
  pub_msg.pos = cur_pos;
  pub_msg.vel = vel;
  pub.publish(&pub_msg);
  last_pos = cur_pos;
  last_time = cur_time;
}


void setup() {
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  servo.attach(SERVO_PIN);
}

void loop() {
  publish_angle();
  nh.spinOnce();
}