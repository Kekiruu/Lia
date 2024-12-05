#include <IRremote.h>
#include <Servo.h>
//Motor pins
const int STBY = 3;  
const int PWMA = 5;  
const int AIN1 = 7;  
const int PWMB = 6;  
const int BIN1 = 8;  

//IR Remote Pin
const int IR_RECEIVE_PIN = 9;

//Ultrasonic pins
const int Serv = 10;    
const int Echo = 12;  
const int Trig = 13;

//sensor pin:
const int Right = A0; 
const int Middle = A1; 
const int Left = A2; 

//Values for speeds
const int Speed=70;
int IRSpeed = 80; 

//Values for IR remote
int Movement = 0;
unsigned long lastSignalTime = 0;

//Values for Obstacles
Servo myservo;
int Pos = 90;
bool sens = true;
float duration,distance;  

//Values for Line Tracking
unsigned long pDetectionTime = 0 ; 

//Values for all the modes
bool IRRemote=true;
bool OBS=false;
bool Manual=false;
bool LT=false;

//All of the HEX needed
//Forward:0xB946FF00
//Right:0xBC43FF00
//Left:0xBB44FF00
//Back:0xEA15FF00
//Speed up(1):0xE916FF00
//Speed down(4):0xF30CFF00
//Mode Obstacle(3):0xF20DFF00
//mode Irremote(2):0xE619FF00
//Mode Linetracking(5):0xE718FF00
//mode manual(6):0xA15EFF00


void setup() {

  // put your setup code here, to run once:
  Serial.begin(9600); 
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  pinMode(STBY,OUTPUT);  
  pinMode(PWMA,OUTPUT);  
  pinMode(PWMB,OUTPUT);  
  pinMode(AIN1,OUTPUT);  
  pinMode(BIN1,OUTPUT);  

  myservo.attach(Serv);
  myservo.write(Pos); // Start with the servo at center

}

void loop() {
//checks if the IRRemote mode is off 
if(!IRRemote){
   if (IrReceiver.decode()) {
    stop();
    switch(IrReceiver.decodedIRData.decodedRawData){
      //Turns IRremote control on and everthing else off
      case 0xE619FF00: IRRemote = true; OBS = Manual = LT = false; break;
      case 0xE718FF00: LT = true; IRRemote = OBS = Manual = false; break;
      case 0xA15EFF00: Manual = true; IRRemote = OBS = LT = false; break;
      case 0xF20DFF00: OBS = true; IRRemote = Manual = LT = false; break;
      default: break;
     }
     IrReceiver.resume();
  }
}

//Check whichs mode is on
  if (IRRemote) ModeIRRemote();
  if (OBS) ModeObstacle();
  if (Manual) ModeManual();
  if (LT) ModeLineTracking();
}
//Function for the IR Remote mode
void ModeIRRemote(){
  if (IrReceiver.decode()) {
    
    /*
    //Print for hex Number and also to test the code to make sure it works
    Serial.print("Received HEX Value: 0x");
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    Serial.print("Movement = ");
    Serial.println(Movement);
      Serial.print("Linetracking = ");
    Serial.println(LT);
    Serial.print("OBS = ");
    Serial.println(OBS);
    Serial.print("Manual = ");
    Serial.println(Manual);
    Serial.print("IRRemote = ");
    Serial.println(IRRemote );
    */

    switch(IrReceiver.decodedIRData.decodedRawData){
      //Turns IRremote control on and everthing else off
      case 0xE619FF00: IRRemote = true; OBS = Manual = LT = false; break;
      case 0xE718FF00: LT = true; IRRemote = OBS = Manual = false; break;
      case 0xA15EFF00: Manual = true; IRRemote = OBS = LT = false; break;
      case 0xF20DFF00: OBS = true; IRRemote = Manual = LT = false; break;
      default: break;
    
     //When you click the speedup button it increase the speed by 10
      case 0xE916FF00: if (IRSpeed < 250) IRSpeed += 10; Movement = 0; break;
  
     //When you click the slowdown button it decrease the speed by 10
      case 0xF30CFF00: if (IRSpeed > 10) IRSpeed -= 10; Movement = 0; break;
      
      //Makes the car go forward when you click the button
      case 0xB946FF00: foward(IRSpeed);   Movement = 1; break;

      //Makes the car go backward when you click the button
      case 0xEA15FF00: backward(IRSpeed); Movement = 2; break;

      //Makes the car go left when you click the button
      case 0xBB44FF00: left(IRSpeed);     Movement = 3; break;

      //Makes the car go right when you click the button
      case 0xBC43FF00: right(IRSpeed);    Movement = 4; break;
      //this case is for when you hold the button
      //It check what was button was held and repeats it
      case 0x0:
        if (Movement == 1)foward(IRSpeed);
        if (Movement == 2)backward(IRSpeed);
        if (Movement == 3)left(IRSpeed);
        if (Movement == 4)right(IRSpeed);
      break; 
      
      
     }

     lastSignalTime = millis();
     IrReceiver.resume();
  }
 //this is to makes it so you have to hold the button for it to work 
 //it works by checking the last time you release the button
 //and in all the movement cases theres a line of code to start a millis for when its clicked 
 //so that this code here can check when it was last clicked
    if (millis() - lastSignalTime > 150) {
    stop(); // Stop the robot after timeout
  
  }
}

//this functions is for the line tracking mode
void ModeLineTracking(){

  if (analogRead(Middle)<600){

//If it the right sensor detects a black line it goes right
   if (analogRead(Right)> 600 && analogRead(Right)<1000)  right(Speed); 

//If it the left sensor detects a black line goes left
   if (analogRead(Left)> 600 && analogRead(Left)<1000)    left(Speed); 
  }


//If it the middle sensors detects the black line it goes forward 
if (analogRead(Middle)> 600 && analogRead(Middle)<1000) foward(Speed); 

//If it doesnt detect a black line it stops
if (analogRead(Right)>1000 && analogRead(Middle)>1000 && analogRead(Left)>1000) { 
 stop(); 
} 

else if(analogRead(Right)<600 && analogRead(Middle)<600&& analogRead(Left)<600) { 

  unsigned long DetectionTime = millis(); 
  //Printing the Detection time to make sure it works
  Serial.println(DetectionTime); 

  if (DetectionTime-pDetectionTime <=600)right(Speed); 
  else if (DetectionTime-pDetectionTime >=600 && DetectionTime-pDetectionTime <=3600)  left(Speed); 
  else if (DetectionTime-pDetectionTime >=3600 && DetectionTime-pDetectionTime <=6600) right(Speed); 
  else if(DetectionTime-pDetectionTime >=6600 && DetectionTime-pDetectionTime <=7000)  stop(); 
  else if(DetectionTime-pDetectionTime >=7000){ 
    pDetectionTime =DetectionTime; 
  } 
} 

}

//This functions is for the Obstacle avoidance mode
void ModeObstacle(){
//Checks if the Ultrasonic is sensing
if (sens) {
    Search();
  }

  // Print distance to Serial Monitor
  Serial.print("Distance = ");
  if (distance >= 400 || distance <= 2) {
    Serial.println("Out of range");
  } else {
    Serial.println(distance);
  }

  Serial.println(Pos);

  // Main Logic
  myservo.write(Pos);
  if (distance >= 25 && Pos == 90) {
    foward(100);
    sens = true;
  } else if (distance >= 25 && Pos == 180) {
    sens = false;
    myservo.write(90);
    left(100);
    delay(500);
    Pos = 90;
  } else if (distance >= 25 && Pos == 0) {
    sens = false;
    myservo.write(90);
    right(100);
    delay(500);
    Pos = 90;
  } else if (distance <= 25) {
    stop();
    Pos = 180;
    myservo.write(Pos);
    delay(500);
    Search();
    delay(500);
    if (distance <= 25 && Pos == 180) {
      Pos = 0;
      myservo.write(Pos);
      delay(500);
      Search();
      delay(500);
      if (distance <= 25 && Pos == 0) {
        Pos = 90;
        myservo.write(Pos);
        backward(50);
        delay(2000);
        right(100);
        delay(1000);
      }
    }
  }
}

////This functions is to make the car do a Victory lap arround the table(cant stop it when it starts)
void ModeManual(){
  foward(100); 
  delay(6000); 
  leftFoward(100); 
  delay(2900); 
  foward(100); 
  delay(2000); 
  leftFoward(100); 
  delay(2900); 
  foward(100); 
  delay(6000); 
  leftFoward(100); 
  delay(2900); 
  foward(100); 
  delay(2000); 
  leftFoward(100); 
  delay(2900); 
}


//This functions is to make the ultrasonic work
void Search(){
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  duration = pulseIn(Echo, HIGH);
  distance = (duration/ 2) * 0.0343;
  if (distance < 2 || distance > 400) {
    distance = 1500; // Indicate invalid reading
  }
}

//This functions is to make the car stop
void stop(){  
  digitalWrite(STBY,HIGH);  
  digitalWrite(BIN1,HIGH);  
  analogWrite(PWMB,0);  
  digitalWrite(AIN1,HIGH);  
  analogWrite(PWMA,0);  
}  


//This functions is to make the car go forward
void foward(int speed){   
  digitalWrite(STBY,HIGH);  
  digitalWrite(BIN1,HIGH);  
  analogWrite(PWMB,speed);  
  digitalWrite(AIN1,HIGH);  
  analogWrite(PWMA,speed);  
}  

 
//This functions is to make the car go backward
void backward(int speed){  
  digitalWrite(STBY,HIGH);  
  digitalWrite(BIN1,LOW);  
  analogWrite(PWMB,speed);  
  digitalWrite(AIN1,LOW);  
  analogWrite(PWMA,speed);  
}  

 
 
//This functions is to make the car go left
void left(int speed){  
  digitalWrite(STBY,HIGH);  
  digitalWrite(BIN1,LOW);  
  analogWrite(PWMB,speed);  
  digitalWrite(AIN1,HIGH);  
  analogWrite(PWMA,speed);  
}  

 
 
//This functions is to make the car go right
void right(int speed){  
  digitalWrite(STBY,HIGH);  
  digitalWrite(BIN1,HIGH);  
  analogWrite(PWMB,speed);  
  digitalWrite(AIN1,LOW);  
  analogWrite(PWMA,speed);   
}  

//This functions is to make the car go Rightforward
void rightFoward(int speed){  
  digitalWrite(STBY,HIGH);  
  digitalWrite(BIN1,HIGH);  
  analogWrite(PWMB,speed);  
  digitalWrite(AIN1,HIGH);  
  analogWrite(PWMA,speed/2);  

}  

//This functions is to make the car go Leftforward
void leftFoward(int speed){ 
  digitalWrite(STBY,HIGH); 
  digitalWrite(BIN1,HIGH); 
  analogWrite(PWMB,speed/2); 
  digitalWrite(AIN1,HIGH); 
  analogWrite(PWMA,speed); 
} 
