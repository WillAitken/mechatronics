/* ELEC 299 Final Competition 
 *  Will Aitken
 *  Rachel Hickson
 *  Georgia Walker
*/

#include <QSerial.h>
#include <Servo.h>

#define IRpin  2              
#define GripSensor 3
#define GripPin 9
#define M1 7
#define M2 4 
#define E1 6
#define E2 5
#define RTHRESH 800
#define CTHRESH 800
#define LTHRESH 800
#define RPin A0
#define CPin A1
#define LPin A2
#define frontSensor A3

QSerial myIRserial;           
Servo GripServo;

int leftSpeed = 150;           
int rightSpeed = 150;
int dist_th = 500;       
int intersections_crossed = 0;

void setup() {
  //Initialize pinmodes
  Serial.begin(9600);
  Serial.print("setup");
  
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(RPin, INPUT);
  pinMode(CPin, INPUT);
  pinMode(LPin, INPUT);
}

void loop() {
  Serial.println("loop");
  main();
}

//Check starting location. Select and exectute path accordingly
int main(){
  Serial.println("main");
  int location = getCurrentLocation();
  int BallIDArray[] = {0, 0, 0, 0, 0};
  getBallIDArray(location, BallIDArray);
  for (int i=0; i < 5; i++){
    int ball;
    ball = BallIDArray[i];
    getPathDetails(ball);
    pickUpBall();
    rotate(0, 180);
    dropBall();
    rotate(0, 180);
  }
  stopRobot();
}

//Use serial communcation to read starting location
int getCurrentLocation(){          
  int position = 0;
  myIRserial.attach(IRpin, -1);
  int IRinput = myIRserial.receive(200);
  Serial.println(IRinput);
  if (IRinput == 'A'){
    position = 1;
    return position;
  }

  if (IRinput == 'B'){
    position = 2;
    return position;
  }

  if (IRinput == 'C'){
    position = 3;
    return position;
  }
}

// Based on starting location set ball path
int getBallIDArray(int position, int BallIDArray[]){
  if (position == 1){
    BallIDArray[0] = 3;
    BallIDArray[1] = 15;
    BallIDArray[2] = 6;
    BallIDArray[3] = 4;
    BallIDArray[4] = 5;
  }
  
  else if (position == 2){
    //BallIDArray[] = {2, 14, 8, 7, 9};
    BallIDArray[0] = 2;
    BallIDArray[1] = 14;
    BallIDArray[2] = 8;
    BallIDArray[3] = 7;
    BallIDArray[4] = 9;
  }

  else if (position == 3){
    //BallIDArray[] = {1, 13, 10, 11, 12};
    BallIDArray[0] = 1;
    BallIDArray[1] = 13;
    BallIDArray[2] = 10;
    BallIDArray[3] = 11;
    BallIDArray[4] = 12;
  }
  
  else{
    //BallIDArray = {0, 0, 0, 0, 0};
    return 0;
  }
}

//Use arm to pick up ball
void pickUpBall()
{
  int angle = 0;
  pinMode(GripSensor, INPUT);
  GripServo.attach(GripPin);
  GripServo.write(angle);

  int gripInput = digitalRead(GripPin);
  while (gripInput != HIGH)              
  {
    angle += 5;
    GripServo.write(angle);
    gripInput = digitalRead(GripPin);
  }
}

//Turn function
void rotate(int LRIndicator, int angle)
{
  if (LRIndicator == 0)     // 0 indicates rotate left
  {
    digitalWrite(M1, HIGH);                                 
    digitalWrite(M2, LOW);

    analogWrite(E1, leftSpeed);                                
    analogWrite(E2, rightSpeed);
    if (angle == 90){
      delay(100);                                         
    }
    else if (angle == 180){
      delay(500);                                                                                           
    }
  }
  if (LRIndicator == 1)     // 1 indicates rotate right
  {
    digitalWrite(M1, LOW);                               
    digitalWrite(M2, HIGH);

    analogWrite(E1, leftSpeed);                                
    analogWrite(E2, rightSpeed); 
    if (angle == 90){
      delay(100);                                          
    }
    else if (angle == 180){
      delay(500);                                         
    }
  }
}

void driveForwards()                                       
{
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, leftSpeed);                                 
  analogWrite(E2, rightSpeed);
  while(1){
    adjustSpeed();
    checkForObstacles();
    checkIntersections();
  }
}

void driveBackwards()                               
{
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1, leftSpeed);
  analogWrite(E2, rightSpeed);
  delay(100);
}

void dropBall()
{
  GripServo.write(0); //Release grip
}

void stopRobot()
{
  //set speed of both motors to zero
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}

//Due to age, the wheels rotate at different rates. This function reads an IR reflection off the surface the robots driving on 
  //and corrects its trajectory accordingly.
void adjustSpeed()
{
  int R = analogRead(RPin);
  int C = analogRead(CPin);
  int L = analogRead(LPin);
  if (L <= LTHRESH){                //If off tape                              
    leftSpeed += 10;                                          
    analogWrite(E1, leftSpeed);
  }
  
  if (R <= RTHRESH){                //If off tape
    rightSpeed += 10;
    analogWrite(E2, rightSpeed);
  } 
}

//Uses forward facing IR sensor to detect obstacles
void checkForObstacles()
{
  int dist_to_object = analogRead(IRpin);
  while (dist_to_object >= dist_th){
    stopRobot();
  }   
}

//This function counts how many lines the robot has crossed
void checkIntersections()         
{
  int R = analogRead(RPin);
  int C = analogRead(CPin);
  int L = analogRead(LPin);
  
  if (L > LTHRESH and C > CTHRESH and R > RTHRESH){      
    intersections_crossed += 1;
  }
}

//Hard coded paths for each ball
int getPathDetails(int ballID)
{
  if (ballID == 1)
  {
    while (intersections_crossed < 1){
      driveForwards();                                 
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;
    
    while (intersections_crossed < 3){
      driveForwards();
    }
    driveForwards();                                  
    intersections_crossed = 0;
    
  } 

  else if (ballID = 2){
    while (intersections_crossed < 2){
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 2){
      driveForwards();
    }
    driveForwards();
    intersections_crossed = 0;
  } 

  else if (ballID = 3){
    while (intersections_crossed < 3){
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 1){
      driveForwards();
    }
    driveForwards();
    intersections_crossed = 0;
  }

  else if (ballID = 4){
    while (intersections_crossed < 4){
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 2){
      driveForwards();
    }
    driveForwards();
    intersections_crossed = 0;
  } 

  else if (ballID = 5){
    while (intersections_crossed < 5){
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 1){
      driveForwards();
    }
    driveForwards();
    intersections_crossed = 0;
  } 

  else if (ballID = 6)
  {
    while (intersections_crossed < 3){
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 1){
      driveForwards();
    }

    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 2){
      driveForwards();
    }   
    driveForwards();
    intersections_crossed = 0;
  }

  else if (ballID = 7){
    while (intersections_crossed < 5)
    {
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 1)
    {
      driveForwards();
    }
    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;
    driveForwards();
  }

  else if (ballID = 8){
    while (intersections_crossed < 5){
      driveForwards();
    } 
    driveForwards();
    intersections_crossed = 0;
  }

  else if (ballID = 9){
    while (intersections_crossed < 5){
      driveForwards();
    }
    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 1){
      driveForwards();
    }

    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;  
    driveForwards();
  }

  else if (ballID = 10)
  {
    while (intersections_crossed < 3)
    {
      driveForwards();
    }
    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 1){
      driveForwards();
    }

    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 2){
      driveForwards();
    }   
    driveForwards();
    intersections_crossed = 0;
  }

  else if (ballID = 11)
  {
    while (intersections_crossed < 5)
    {
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 1)
    {
      driveForwards();
    }   
    driveForwards();
    intersections_crossed = 0;
  } 

  else if (ballID = 12)
  {
    while (intersections_crossed < 4)
    {
      driveForwards();
    }
    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 1)
    {
      driveForwards();
    }   
    driveForwards();
    intersections_crossed = 0;
  } 

  else if (ballID = 13)
  {
    while (intersections_crossed < 3)
    {
      driveForwards();
    }
    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 1)
    {
      driveForwards();
    }
    driveForwards();
    intersections_crossed = 0;
  } 

  else if (ballID = 14)
  {
    while (intersections_crossed < 2)
    {
      driveForwards();
    }
    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 2)
    {
      driveForwards();
    }
    driveForwards();
    intersections_crossed = 0;
  } 
  else if (ballID = 15)
  {
    while (intersections_crossed < 1)
    {
      driveForwards();
    }
    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 3)
    {
      driveForwards();
    }   
    driveForwards();
    intersections_crossed = 0;
  }
}

//Hard coded return paths for each ball
int GetReversePathDetails(int ballID)
{
  if (ballID == 1)
  {
    while (intersections_crossed < 3){
      driveForwards();
    }
    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;
    
    while (intersections_crossed < 1){
      driveForwards();                                 
    }
    
    driveForwards();                                   
    intersections_crossed = 0;
    
  } 

  else if (ballID = 2){
    while (intersections_crossed < 2){
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 2){
      driveForwards();
    }
    
    driveForwards();
    intersections_crossed = 0;
  } 

  else if (ballID = 3){
    while (intersections_crossed < 1){
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 3){
      driveForwards();
    }
    driveForwards();
    intersections_crossed = 0;
  }

  else if (ballID = 4){
    while (intersections_crossed < 2){
      driveForwards();
    }
    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 4){
      driveForwards();
    }
    driveForwards();
    intersections_crossed = 0;
  } 

  else if (ballID = 5){
    while (intersections_crossed < 1){
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 5){
      driveForwards();
    }
    driveForwards();
    intersections_crossed = 0;
  } 

  else if (ballID = 6)
  {
    while (intersections_crossed < 2){
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 1){
      driveForwards();
    }

    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 3){
      driveForwards();
    }   
    
    driveForwards();
    intersections_crossed = 0;
  }

  else if (ballID = 7){
    rotate(0, 90);
    while (intersections_crossed < 1)
    {
      driveForwards();
    }
    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 5)
    {
      driveForwards();
    }
    stopRobot();
    
    intersections_crossed = 0;
    driveForwards();
  }

  else if (ballID = 8){
    while (intersections_crossed < 5){
      driveForwards();
    } 
    driveForwards();
    intersections_crossed = 0;
  }

  else if (ballID = 9){
    rotate(0, 90);
    while (intersections_crossed < 1){
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 5){
      driveForwards();
    }

    stopRobot();
    
    intersections_crossed = 0;  
    driveForwards();
  }

  else if (ballID = 10)
  {
    while (intersections_crossed < 2)
    {
      driveForwards();
    }
    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 1){
      driveForwards();
    }

    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 3){
      driveForwards();
    }   
    driveForwards();
    intersections_crossed = 0;
  }

  else if (ballID = 11)
  {
    while (intersections_crossed < 1)
    {
      driveForwards();
    }
    stopRobot();
    rotate(1, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 5)
    {
      driveForwards();
    }   
    driveForwards();
    intersections_crossed = 0;
  } 

  else if (ballID = 12)
  {
    while (intersections_crossed < 1)
    {
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 4)
    {
      driveForwards();
    }   
    driveForwards();
    intersections_crossed = 0;
  } 

  else if (ballID = 13)
  {
    while (intersections_crossed < 1)
    {
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 3)
    {
      driveForwards();
    }
    driveForwards();
    intersections_crossed = 0;
  } 

  else if (ballID = 14)
  {
    while (intersections_crossed < 2)
    {
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 2)
    {
      driveForwards();
    }
    driveForwards();
    intersections_crossed = 0;
  } 
  else if (ballID = 15)
  {
    while (intersections_crossed < 3)
    {
      driveForwards();
    }
    stopRobot();
    rotate(0, 90);
    intersections_crossed = 0;

    while (intersections_crossed < 1)
    {
      driveForwards();
    }   
    driveForwards();
    intersections_crossed = 0;
  }
 
}
