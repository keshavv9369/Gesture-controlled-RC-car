#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// NRF setup...................................

#define CE_PIN 4
#define CSN_PIN 5
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

struct DataPacket {
  int power_f_b;  // Forward/Backward
  int power_l_r;  // Left/Right
};

// ...............nrf function ...............
void setupNRF() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
  Serial.println("NRF24 ready");
}

void transmitData(int power_f_b, int power_l_r) {
  DataPacket packet;
  packet.power_f_b = power_f_b;
  packet.power_l_r = power_l_r;
  radio.write(&packet, sizeof(packet));
}


MPU6050 mpu;

float filterdGX = 0, filterdGY = 0, filterdGZ = 0;
float filterdGX2 = 0, filterdGY2 = 0, filterdGZ2 = 0;
float filterdAX = 0, filterdAY = 0, filterdAZ = 0;
float filterdAX2 = 0, filterdAY2 = 0, filterdAZ2 = 0;
float alpha = 0.1;

float pitch = 0;  // angle estimation
float roll =0; 
unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU sensor is connected !!!!");
  } else {
    Serial.println("MPU is not connected !!!!");
  }

  setupNRF();
  prevTime = micros();  // Start timer
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  float sumAX2 = 0, sumAY2 = 0, sumAZ2 = 0;
  float sumGX2 = 0, sumGY2 = 0, sumGZ2 = 0;

  // Read and filter 10 samples
  for (int i = 0; i < 10; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float ax_scaled = ax / 16384.0;
    float ay_scaled = ay / 16384.0;
    float az_scaled = az / 16384.0;

    float gx_scaled = gx / 131.0;
    float gy_scaled = gy / 131.0;
    float gz_scaled = gz / 131.0;

    // Double LPF for accelerometer
    filterdAX = alpha * ax_scaled + (1 - alpha) * filterdAX;
    filterdAY = alpha * ay_scaled + (1 - alpha) * filterdAY;
    filterdAZ = alpha * az_scaled + (1 - alpha) * filterdAZ;

    filterdAX2 = alpha * filterdAX + (1 - alpha) * filterdAX2;
    filterdAY2 = alpha * filterdAY + (1 - alpha) * filterdAY2;
    filterdAZ2 = alpha * filterdAZ + (1 - alpha) * filterdAZ2;

    sumAX2 += filterdAX2;
    sumAY2 += filterdAY2;
    sumAZ2 += filterdAZ2;

    // Double LPF for gyro
    filterdGX = alpha * gx_scaled + (1 - alpha) * filterdGX;
    filterdGY = alpha * gy_scaled + (1 - alpha) * filterdGY;
    filterdGZ = alpha * gz_scaled + (1 - alpha) * filterdGZ;

    filterdGX2 = alpha * filterdGX + (1 - alpha) * filterdGX2;
    filterdGY2 = alpha * filterdGY + (1 - alpha) * filterdGY2;
    filterdGZ2 = alpha * filterdGZ + (1 - alpha) * filterdGZ2;

    sumGX2 += filterdGX2;
    sumGY2 += filterdGY2;
    sumGZ2 += filterdGZ2;

    delay(1);
  }

  // Average of filtered values
  float avgAX2 = sumAX2 / 10.0;
  float avgAY2 = sumAY2 / 10.0;
  float avgAZ2 = sumAZ2 / 10.0;

  float avgGX2 = sumGX2 / 10.0;
  float avgGY2 = sumGY2 / 10.0;
  float avgGZ2 = sumGZ2 / 10.0;

  // Time delta in seconds
  unsigned long now = micros();
  float dt = (now - prevTime) / 1000000.0;
  prevTime = now;

  // Step 1: Calculate pitch from accelerometer (in degrees)
  float pitch_acc = atan2(avgAY2, avgAZ2) * 180.0 / PI;
  float roll_acc=atan2(-avgAX2,sqrt(avgAY2*avgAY2 + avgAZ2*avgAZ2))*180 / PI;
  // Step 2: Integrate gyroscope pitch rate (X-axis)
  pitch += avgGX2 * dt;
  roll += avgGY2 *dt;

  // Step 3: Apply complementary filter
  float alpha_cf = 0.96;  // Complementary filter weight for gyro
  pitch = alpha_cf * pitch + (1 - alpha_cf) * pitch_acc;
  roll = alpha_cf * roll + (1 - alpha_cf) * roll_acc;

  // Serial.print("Pitch (deg): ");
  // Serial.print(pitch);
  // Serial.print("    ||  roll : "); 
  // Serial.println(roll);




  int angle_Y=constrain(pitch,-90,90);
  int angle_X=constrain(roll,-90,90);
 
  int power_f_b=map(angle_X,-90,90,-100,100);
  int power_l_r=map(angle_Y,-90,90,-100,100);

  transmitData(power_f_b, power_l_r);

  Serial.print("F/B Power: ");
  Serial.print(power_f_b);
  Serial.print("   ||   L/R Power: ");
  Serial.println(power_l_r);
  delay(1);
}
