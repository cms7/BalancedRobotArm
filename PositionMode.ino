#include <Dynamixel2Arduino.h>
#include <math.h>

// Using the Arduino_OpenRB150 Board
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;

// Initialiase the ID of dynamixel [Spring1,Spring2,Base,Shoulder,Elbow]
const uint8_t DXL_ID [] = {1,2,3,4,5};

// set dynamixel protocol - servos use protocol 1
const float DXL_PROTOCOL_VERSION = 1.0;

// DXL 
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names such as speed and acceleration
using namespace ControlTableItem;


// Define and initialise joysticks and map
#define x_Joystick_Pin A0
#define y_Joystick_Pin A1
#define z_Joystick_Pin A2

double x_joystick = 0;
double y_joystick = 0;
double z_joystick = 0;

/*//map the joysticks pwm values from -1 to 1 for change in x,y,z values of end effector
double mapJoystick_IK(double joyValue){
  double value = map(joyValue, 0, 1023, -1, 1);
  return value;
}

double mapJoystick_angle(double joyValue){
  double value = map(joyValue, 0, 1023, -1, 1);
  return value;
}*/

// Define robot features including link lengths, position of end effector, initial joint angles on start up
double l1 = 430; 
double l2 = 210;
double l3 = 300;

double px = 122.61;
double py = -244.63;
double pz = 377.39;

// Define the variables for the servo positions and angles
int base_position = 0;
int shoulder_position = 0;
int elbow_position = 0;

double base_angle = 0;
double shoulder_angle = 90;
double elbow_angle = 90;

// Define the variables for the inverse kinematics equations
double theta1 = 0;
double theta2 = 0;
double theta3 = 0;

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This matches dynamixel's baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Set up each dynamixel which controls the springs, this is in a different operating mode
    // Check Dynamixel is active
    dxl.ping(DXL_ID[1]);
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID[1]);
    dxl.setOperatingMode(DXL_ID[1], OP_VELOCITY);
    dxl.torqueOn(DXL_ID[1]);
    // Set DXLs velocity
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[1],60);
    // Check Dynamixel is active
    dxl.ping(DXL_ID[0]);
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID[0]);
    dxl.setOperatingMode(DXL_ID[0], OP_VELOCITY);
    dxl.torqueOn(DXL_ID[0]);
    // Set DXLs velocity
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[0],60);
  

  // Set up each dynamixel for controlling the kinematics
  for (int i=2;i<=4;i++){
    // Check Dynamixel is active
    dxl.ping(DXL_ID[i]);
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_POSITION);
    dxl.torqueOn(DXL_ID[i]);
    // Set DXLs velocity
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i],60);
  }
  
  //dxl.setGoalVelocity(DXL_ID[0], 2000);
  //dxl.setGoalVelocity(DXL_ID[1], 1300);
  // initialise the arms starup position
  dxl.setGoalPosition(DXL_ID[2], base_angle , UNIT_DEGREE); //base    - rotational
  dxl.setGoalPosition(DXL_ID[3], shoulder_angle , UNIT_DEGREE); //shoulder    - rotational
  dxl.setGoalPosition(DXL_ID[4], elbow_angle , UNIT_DEGREE); //elbow      - rotational

  // set a delay of 5 seconds 
  delay(5000);
}

// Function which determines the forward kinematics and returns a coordinate for end effector
void forward_kinematics(){
  px = cos(base_angle)*(l2*cos(shoulder_angle)+l3*cos(shoulder_angle+elbow_angle));
  py = sin(base_angle)*(l2*cos(shoulder_angle)+l3*cos(shoulder_angle+elbow_angle));
  pz = l2*sin(shoulder_angle)+l3*sin(shoulder_angle+elbow_angle)+l1;
}

// Function to convert radians to degrees
double radToDegree(double radian){
  double degree = radian*180/PI;
  return degree;
}

// Compute the inverse kinematics, which will be used to manipulate the robots end effector with joysticks
void inverse_kinematics(){

  //IK 1
  theta1 = atan2(py,px);
  double k = (sq(px)+sq(py)+sq(pz-l1)-sq(l2)-sq(l3))/(2*l2*l3);
  theta3 = atan2(sqrt(1-sq(k)),k);
  theta2 = atan2((l2+l3*k)*(pz-l1)+(l3*sqrt(1-sq(k)))*sqrt(sq(px)+sq(py)),(l2+l3*k)*sqrt(sq(px)+sq(py))+(l3*sqrt(1-sq(k)))*(pz-l1));

  //IK 2
  /*double r = sqrt(sq(px)+sq(py));
  double k = sqrt(sq(r)+sq(pz-l1));
  theta3 = acos((sq(k)-sq(l2)-sq(l3))/(2*l2*l3));

  double alpha = atan2(l3*sin(theta3),l2+l3*cos(theta3));
  double beta = atan2(pz-l1,r);  
  theta2 = beta-alpha;*/

  // Convert radians to degrees
  theta1 = radToDegree(theta1);
  theta2 = radToDegree(theta2);
  theta3 = radToDegree(theta3);
}

// Map the load of the motors and determine the direcition of the load
int mapLoad(int id){
  int motorload = dxl.readControlTableItem(PRESENT_LOAD, id);
  motorload = motorload>=1032 ? map(motorload, 1032, 2034, 0, 1032) : -motorload;
  //Serial.println(motorload);
  return motorload;
}

// Function to adjust the springs using the load and direction
void setSpringSpeed(int id, int load){
  if(load >0){
    if(load>10) dxl.setGoalVelocity(id,1020);
    else dxl.setGoalVelocity(id, map(load, 0, 300, 0, 1024));
  }
  else{
    if(load < -10) dxl.setGoalVelocity(id, 2040);
    else dxl.setGoalVelocity(id, map(load, 0, -300, 1024, 2048));
  }
}

void balanceArm(){
  if(abs(mapLoad(DXL_ID[3]))<=16){
      dxl.setGoalVelocity(DXL_ID[0], 2047);
  }
  if(abs(mapLoad(DXL_ID[4]))<=16){
      dxl.setGoalVelocity(DXL_ID[1], 1023);
  }
}

void loop() {
  // read and udjust the springs depending on the load put on the motors
  setSpringSpeed(DXL_ID[0], mapLoad(DXL_ID[3]));
  setSpringSpeed(DXL_ID[1], mapLoad(DXL_ID[4]));
  //balanceArm();
  

  // Map the joystick values to angles
  int x_increment = map(analogRead(x_Joystick_Pin), 0, 1023, -1, 1);
  int y_increment = map(analogRead(y_Joystick_Pin), 0, 1023, -1, 1);
  int z_increment = map(analogRead(z_Joystick_Pin), 0, 1023, -1, 1);


  // Increment the coordinates according to joysticks
  px += x_increment;
  py += y_increment;
  pz += z_increment;

  //Compute forward kinematics

  //forward_kinematics();
 
  //Comput the inverse kinematics
  inverse_kinematics();
  
  /*Serial.println();
  Serial.print("theta1:  ");
  Serial.print(theta1);
  Serial.print("    px:  ");
  Serial.print(px);
  Serial.print("    theta2:  ");
  Serial.print(theta2);
  Serial.print("    py:  ");
  Serial.print(py);
  Serial.print("     theta3:  ");
  Serial.print(theta3);
  Serial.print("     pz:  ");
  Serial.print(pz);*/

  Serial.print(mapLoad(DXL_ID[3]));
  Serial.print(",");
  Serial.println(mapLoad(DXL_ID[4]));

  // Set Velocity of each motor
  dxl.setGoalVelocity(DXL_ID[2], 150);
  dxl.setGoalVelocity(DXL_ID[3], 150);
  dxl.setGoalVelocity(DXL_ID[4], 150);

  // Adjust the position of the motors depending on inverse kinematics
  dxl.setGoalPosition(DXL_ID[2], theta1 , UNIT_DEGREE); //base    - rotational
  dxl.setGoalPosition(DXL_ID[3], theta2 , UNIT_DEGREE); //shoulder    - rotational
  dxl.setGoalPosition(DXL_ID[4], theta3 , UNIT_DEGREE); //elbow      - rotational

}


