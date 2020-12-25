#include <NewPing.h>
#include <SimpleKalmanFilter.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
 
#define SONAR_NUM 3          //The number of sensors. 
#define MAX_DISTANCE 200     //Mad distance to detect obstacles.
#define PING_INTERVAL 33     //Looping the pings after 33 microseconds.
 
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
 
unsigned long _timerStart = 0;
 
int LOOPING = 40; //Loop for every 40 milliseconds.
 
uint8_t oldSensorReading[3];    //Store last valid value of the sensors.
 
uint8_t leftSensor;             //Store raw sensor's value.
uint8_t centerSensor;
uint8_t rightSensor;
 
uint8_t leftSensorKalman;       //Store filtered sensor's value.
uint8_t centerSensorKalman;
uint8_t rightSensorKalman;
 
 
NewPing sonar[SONAR_NUM] = {
  NewPing(3, 2, MAX_DISTANCE), // Trigger pin, echo pin, and max distance to ping.
  NewPing(5, 4, MAX_DISTANCE),
  NewPing(7, 6, MAX_DISTANCE)
};
 
/*
  create Kalman filter objects for the sensors.
   SimpleKalmanFilter(e_mea, e_est, q);
   e_mea: Measurement Uncertainty
   e_est: Estimation Uncertainty
   q: Process Noise
*/
SimpleKalmanFilter KF_Left(2, 2, 0.01);
SimpleKalmanFilter KF_Center(2, 2, 0.01);
SimpleKalmanFilter KF_Right(2, 2, 0.01);
 
ros::NodeHandle nh; //create an object which represents the ROS node.
 
//looping the sensors
void sensorCycle() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle();
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
}
 
// If ping received, set the sensor distance to array.
void echoCheck() {
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
 
//Return the last valid value from the sensor.
void oneSensorCycle() {
  leftSensor   = returnLastValidRead(0, cm[0]);
  centerSensor = returnLastValidRead(1, cm[1]);
  rightSensor  = returnLastValidRead(2, cm[2]);
}
 
//If sensor value is 0, then return the last stored value different than 0.
int returnLastValidRead(uint8_t sensorArray, uint8_t cm) {
  if (cm != 0) {
    return oldSensorReading[sensorArray] = cm;
  } else {
    return oldSensorReading[sensorArray];
  }
}
 
//Apply Kalman Filter to sensor reading.
void applyKF() {
  leftSensorKalman   = KF_Left.updateEstimate(leftSensor);
  centerSensorKalman = KF_Center.updateEstimate(centerSensor);
  rightSensorKalman  = KF_Right.updateEstimate(rightSensor);
}
 
void startTimer() {
  _timerStart = millis();
}
 
bool isTimeForLoop(int _mSec) {
  return (millis() - _timerStart) > _mSec;
}
 
void sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26;
  range_name.min_range = 0.0;
  range_name.max_range = 2.0;
}
 
//Create three instances for range messages.
sensor_msgs::Range range_left;
sensor_msgs::Range range_center;
sensor_msgs::Range range_right;
 
//Create publisher onjects for all sensors
ros::Publisher pub_range_left("/ultrasound_left", &range_left);
ros::Publisher pub_range_center("/ultrasound_center", &range_center);
ros::Publisher pub_range_right("/ultrasound_right", &range_right);
 
void setup() {
  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
 
  nh.initNode();
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_center);
  nh.advertise(pub_range_right);
 
  sensor_msg_init(range_left, "/ultrasound_left");
  sensor_msg_init(range_center, "/ultrasound_center");
  sensor_msg_init(range_right, "/ultrasound_right");
}
 
void loop() {
  if (isTimeForLoop(LOOPING)) {
    sensorCycle();
    oneSensorCycle();
    applyKF();
    range_left.range   = leftSensorKalman;
    range_center.range = centerSensorKalman;
    range_right.range  = centerSensorKalman;
 
    range_left.header.stamp = nh.now();
    range_center.header.stamp = nh.now();
    range_right.header.stamp = nh.now();
 
    pub_range_left.publish(&range_left);
    pub_range_center.publish(&range_center);
    pub_range_right.publish(&range_right);
 
    startTimer();
  }
  nh.spinOnce();//Handle ROS events
}
