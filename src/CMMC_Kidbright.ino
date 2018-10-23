//#define Receiver_Debug

#include "CMMC_Receiver.h"


#define LED 0
#define ENA 22
#define ENB 21
#define Motor_A_Pin 26
#define Motor_B_Pin 27
#define Motor_C_Pin 18
#define Motor_D_Pin 19


void setup() {
  receriver_init();

  pinMode(LED, OUTPUT);
  pinMode(Motor_C_Pin, OUTPUT);
  pinMode(Motor_D_Pin, OUTPUT);

  ledcSetup(1, 5000, 8); //timer chanel 1 // freq = 5kHz // 8bit
  ledcSetup(2, 5000, 8); //timer chanel 2 // freq = 5kHz // 8bit

  ledcSetup(3, 5000, 8); //timer chanel 0 // freq = 5kHz // 8bit
  ledcSetup(4, 5000, 8); //timer chanel 0 // freq = 5kHz // 8bit

  ledcAttachPin(ENA, 1); //pin (22) // timer chanel 1
  ledcAttachPin(ENB, 2); //pin (21) // timer chanel 2

  ledcAttachPin(Motor_A_Pin, 3); //pin (27) // timer chanel 1
  ledcAttachPin(Motor_B_Pin, 4); //pin (26) // timer chanel 2
}

uint32_t time_now, time_prev1;

void loop()
{


  time_now = millis();

  if (time_now - time_prev1 >= 20)
  {
    time_prev1 += 20;

#ifdef Receiver_Debug
    Serial.print(Get_ChannelValue(1)); Serial.print("\t");
    Serial.print(Get_ChannelValue(2)); Serial.print("\t");
    Serial.print(Get_ChannelValue(3)); Serial.print("\t");
    Serial.print(Get_ChannelValue(4)); Serial.println("\t");
#endif

    float tmp;
    float tmp1 = (float)Get_ChannelValue(1) * 10.23f;
    float tmp2 = (float)Get_ChannelValue(2) * 10.23f;
    float tmp3 = (float)Get_ChannelValue(3) * 10.23f;
    float tmp4 = (float)Get_ChannelValue(4) * 10.23f;

    float motor_L = constrain((tmp2 - tmp1 * 2), -1024, 1024);
    float motor_R = constrain((tmp2 + tmp1 * 2), -1024, 1024);

    if (motor_R > 0) //forward
    {
      // Motor A
      ledcWrite(3, 255); //timer chanel C
      ledcWrite(4, 0); //timer chanel D
      ledcWrite(1, motor_L); //timer chanel A
      //analogWrite(ENA, motor_L);

    } else {
      // Motor A
      ledcWrite(3, 0); //timer chanel C
      ledcWrite(4, 255); //timer chanel D
      ledcWrite(1, -motor_L); //timer chanel A
      //analogWrite(ENA, -motor_L);
    }
    if (motor_L > 0) //forward
    {
      // Motor B
      digitalWrite(Motor_C_Pin, HIGH);
      digitalWrite(Motor_D_Pin, LOW);
      ledcWrite(2, motor_R); //timer chanel B
      //analogWrite(ENB, motor_R);

    } else {
      // Motor B
      digitalWrite(Motor_C_Pin, LOW);
      digitalWrite(Motor_D_Pin, HIGH);
      ledcWrite(2, -motor_R); //timer chanel B
      //analogWrite(ENB, -motor_R);
    }
    delay(1);
  }

  receriver_loop();
}
//..
