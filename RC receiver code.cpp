// ***********************************reciver side ********************************************************************************************
// ********************************ardunio uno r3 bord*****************************************************************************************

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define CE_PIN 5   
#define CSN_PIN 4 

const int A_in1 = 9;   // direction
const int A_in2 = 8;   // direction
const int A_en  = 6;   // PWM speed

// Motor B (Right)
const int B_in1 = 7;   // direction
const int B_in2 = 2;   // direction
const int B_en  = 3;


RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";


struct DataPacket {
  int power_f_b;  
  int power_l_r; 
};

DataPacket receivedData;

void setupNRF() {
  if (!radio.begin()) {
    // Serial.println("NRF24L01 not detected!");
    while (1);
  }

  radio.openReadingPipe(0, address);  
  radio.setPALevel(RF24_PA_LOW);      
  radio.startListening();             
  // Serial.println("NRF Receiver Ready...");
}

bool receiveData(DataPacket &data) {
  if (radio.available()) {
    radio.read(&data, sizeof(data));
    // Serial.println(data.power_f_b);
    // Serial.println(data.power_l_r);
    return true;
  }
  return false; 
}

void setup() {
  pinMode(A_in1, OUTPUT);
  pinMode(A_in2, OUTPUT);
  pinMode(A_en, OUTPUT);

  pinMode(B_in1, OUTPUT);
  pinMode(B_in2, OUTPUT);
  pinMode(B_en, OUTPUT);
  setupNRF();
  Serial.begin(9600);
}

void loop() {
  // if (receiveData(receivedData)) {
  //   Serial.print("Forward/Backward: ");
  //   Serial.print(receivedData.power_f_b);
  //   Serial.print(" | Left/Right: ");
  //   Serial.println(receivedData.power_l_r);
  // }

  receiveData(receivedData);

  int leftmotor=receivedData.power_f_b + receivedData.power_l_r;
  int rightmotor=receivedData.power_f_b - receivedData.power_l_r;
  
  int left_pwm=map(abs(leftmotor),0,100,0,255);
  int right_pwm=map(abs(rightmotor),0,100,0,255);

  setup_motor(A_in1,A_in2,A_en,leftmotor,left_pwm);
  setup_motor(B_in1,B_in2,B_en,rightmotor,right_pwm);

delay(1);
}

void setup_motor(int in1,int in2,int en_A,int motorval,int pwmval){
  if(motorval>5){             //forward
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(motorval<-5){                 //backward
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{                              //stop
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    pwmval=0;
  }
  analogWrite(en_A,pwmval);
};