#if (defined(ARDUINO)) && (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>

#define MOTOR_PWM_PIN 3
#define SERVO_PWM_PIN 9
#define ARMING_PWM 1500  // microsecond
#define ARMING_DELAY 200  // millisecond
#define SPIN_DELAY 1  //.millisecond

// Motor Arming: 1500us pulse for 200ms interval
// Motor: 1500us-> Stop, 1525-> Move Forward
// Servo: 75deg-> Left, 90deg-> Neutral, 105deg-> Right

ros::NodeHandle nh;
Servo motor; 
Servo servo;

/**
 * Performs arming sequence on ESC by starting from neutral.
 */
void arm_esc(){
  motor.writeMicroseconds(1500);
  delay(200);
}

/**
 * Translates command velocity message into PWM signals for motor and servo.
 *
 * @param cmd_msg: command velocity message
 */
void steering(const geometry_msgs::Twist& cmd_msg){
  // Toggle LED
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));
  
  int throttle = int(cmd_msg.linear.x);
  int angle = int(cmd_msg.angular.z);
  
  motor.writeMicroseconds(throttle);
  servo.write(angle);
}

ros::Subscriber<geometry_msgs::Twist> sub("car/cmd_vel", steering);

/**
 * Set-ups resources.
 */
void setup(){
  pinMode(LED_BUILTIN, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  motor.attach(MOTOR_PWM_PIN);
  servo.attach(SERVO_PWM_PIN);
  
  arm_esc();
}

/**
 * Processes callbacks.
 */
void loop(){
  nh.spinOnce();
  delay(SPIN_DELAY);
}
