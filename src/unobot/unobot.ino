#include <Arduino.h>
#include <Encoder.h>
#include <PID_v2.h>
// #include <L298N.h>
#include <ros.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/Float32MultiArray.h>
#include <rospy_tutorials/Floats.h>
#include <std_msgs/Float32.h>

// left
#define B1A 3                       // Quadrature encoder A pin for right wheel
#define B1B 4
#define ENB 11  // black
#define IN3 12  // yellow
#define IN4 13  // green

// right
#define B2A 2   // black, Quadrature encoder A pin for right wheel
#define B2B 10  // blue
#define ENA 5 // brown
#define IN1 6 // red
#define IN2 7 // orange

// L298N   lMotor(ENB, IN3, IN4), rMotor(ENA, IN1, IN2);
Encoder lEnc(B1A, B1B), rEnc(B2A, B2B);

const double PPR = 4680.0;
const int WHEEL_RADIUS = 0.0325;
const float DEG_PER_TICK = 360.0 / PPR;
// float kp = 2.1, ki = 0.2 , kd = 0.5;          // modify for optimal performance
double kp = 0.35, ki = 0.475 , kd = 0.0;          // modify for optimal performance

double input[2] = {0}, output[2] = {0}, setpoint[2] = {0};
PID lPID(&input[0], &output[0], &setpoint[0], kp, ki, kd, PID::P_On::Measurement, PID::Direct);
PID rPID(&input[1], &output[1], &setpoint[1], kp, ki, kd, PID::P_On::Measurement, PID::Direct);

void rPwmOut(int out) {
  // drive motor CW
  digitalWrite(IN1, out > 0);
  digitalWrite(IN2, out < 0);
  analogWrite(ENA, abs(out));
}

void lPwmOut(int out) {
  double input = 0, output = 0, setpoint = 0;
  // drive motor CW
  digitalWrite(IN3, out < 0);
  digitalWrite(IN4, out > 0);
  analogWrite(ENB, abs(out));
}

void setup_motors() {
  // right
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  // left
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

ros::NodeHandle_<ArduinoHardware, 2, 2, 128, 128> nh;
// ros::NodeHandle nh;

std_msgs::Float32 lDeg;
std_msgs::Float32 rDeg;
// rospy_tutorials::Floats deg;
ros::Publisher lDegPub("/lwheel_deg", &lDeg);
ros::Publisher rDegPub("/rwheel_deg", &rDeg);

//ros::Publisher lPubTicks("/lWheel", &lTicks);
//ros::Publisher rPubTicks("/rWheel", &rTicks);

// for debug
// std_msgs::Int32 wheel;
// ros::Publisher pub("/wheel", &wheel);
// std_msgs::Float32 feedback;
// ros::Publisher pub_input("/feedback", &feedback);

ros::Subscriber<rospy_tutorials::Floats> motor_sub("/motor_cmd", &motorCallBack);
/* ros::Subscriber<std_msgs::Float32> lmotor_sub("lmotor_cmd", &lMotorCallBack);

  void lMotorCallBack(const std_msgs::Float32& motor_msg) {
  // in deg/s
  lPwmOut(motor_msg.data);
  setpoint[0] = motor_msg.data;
  }
*/
void motorCallBack(const rospy_tutorials::Floats& msg) {
  // setpoint is in degrees/s
  setpoint[0] = msg.data[0];
  setpoint[1] = msg.data[1];

  // lPwmOut(msg.data[0]);
  // rPwmOut(msg.data[1]);
}

void setup() {
  // put your setup code here, to run once:
  // Serial.begin(115200);
  // while (!Serial);
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // To prevent Motor Noise

  nh.initNode();
  //nh.advertise(lPubTicks);
  //nh.advertise(rPubTicks);
  nh.advertise(lDegPub);
  nh.advertise(rDegPub);

  // deg.data_length=2;

  // for debug
  // nh.advertise(pub);
  // nh.advertise(pub_input);

  nh.subscribe(motor_sub);
 
  setup_motors();

  lEnc.write(0);
  rEnc.write(0);
  // lPwmOut(100);
  // rPwmOut(100);
  
  lPID.SetMode(PID::Automatic);
  // lPID.SetSampleTime(50);  // default 100
  lPID.SetOutputLimits(-255, 255);
  rPID.SetMode(PID::Automatic); 
  rPID.SetOutputLimits(-255, 255);
    
  setpoint[0] = setpoint[1] = 0;
}

//int32_t lOldPosition = -999;
//int32_t rOldPosition = -999;
unsigned long lastTime = 0;
int32_t last_pos[2] = {0};

void loop() {
  int32_t enc[2];

  unsigned long now = millis();
  unsigned int duration = (now - lastTime);
  // if not set to 100, rad/s won't reach to 3.0rad/s (i.e., 0.1m/s)
  if (duration >= 100 )
  {
    enc[0] = -1 * lEnc.read();
    enc[1] = rEnc.read();
    lastTime = now;
    
    lDeg.data = (float)(enc[0] - last_pos[0]) * DEG_PER_TICK;
    lDegPub.publish(&lDeg);
    rDeg.data = (float)(enc[1] - last_pos[1]) * DEG_PER_TICK;
    rDegPub.publish(&rDeg);

    // input[i] is in degree/s
    input[0] = 1000 * lDeg.data / duration;
    // feedback.data = input[0];
    // pub_input.publish(&feedback);

    input[1] = 1000.0 * rDeg.data / duration;

    last_pos[0] = enc[0];
    last_pos[1] = enc[1];
    /*
        nh.getParam("kp", &kp);
        nh.getParam("ki", &ki);
        nh.getParam("kd", &kd);
        lPID.SetTunings(kp, ki, kd);
        rPID.SetTunings(kp, ki, kd);
    */
    lPID.Compute();
    lPwmOut((int)output[0]);
    rPID.Compute();
    rPwmOut((int)output[1]);
  }

  nh.spinOnce();
  // delay(10);
}
