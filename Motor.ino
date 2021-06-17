#include <SoftwareSerial.h>
#include <AFMotor.h>

//뒷쪽
AF_DCMotor motor_1(1);
AF_DCMotor motor_2(2);
//앞쪽
AF_DCMotor motor_3(3);
AF_DCMotor motor_4(4);


void setup() {
  Serial.begin(9600);
  //뒷쪽
  motor_1.setSpeed(255);
  motor_1.run(RELEASE);
  motor_2.setSpeed(255);
  motor_2.run(RELEASE);
  //앞쪽
  motor_3.setSpeed(255);
  motor_3.run(RELEASE);
  motor_4.setSpeed(255);
  motor_4.run(RELEASE);

}

void forward(){
  //뒷쪽
  motor_1.run(FORWARD);
  motor_2.run(FORWARD);
  //앞쪽
  motor_3.run(FORWARD);
  motor_4.run(FORWARD);
  
}

void backward(){
  //뒷쪽
  motor_1.run(BACKWARD);
  motor_2.run(BACKWARD);
  //앞쪽
  motor_3.run(BACKWARD);
  motor_4.run(BACKWARD);
}

void rel(){
  //뒷쪽
  motor_1.run(RELEASE);
  motor_2.run(RELEASE);
  //앞쪽
  motor_3.run(RELEASE);
  motor_4.run(RELEASE);
}

void left(){
  //뒷쪽
  motor_1.run(FORWARD);
  motor_2.run(BACKWARD);
  //앞쪽
  motor_4.run(FORWARD);
  motor_3.run(BACKWARD);
}

void right(){
  //뒷쪽
  motor_1.run(BACKWARD);
  motor_2.run(FORWARD);
  //앞쪽
  motor_4.run(BACKWARD);
  motor_3.run(FORWARD);
}
char op='\0';

void loop() {

  left();
  //forward();
  delay(9000);
  rel();
  while(1){}
  /*
  if(Serial.available()){
    char rd = Serial.read();
    Serial.println(rd);
    op = rd;
   
    if(op=='w'){
      forward();
    }
    else if(op=='s'){
      backward();
    }
    else if(op=='a'){
      left();
    }
    else if(op=='d'){
      right();
    }
    else if(op=='x'){
      rel();
    }
   
    //while(1){}
    
    delay(10);
    }
    */
  
}
  
