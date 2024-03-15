#include <Arduino.h>
#include "Preferences.h"
#include <Update.h>
#include <ezButton.h>
#include "PCF8574.h"

#define INCREASE_CHANNEL_1_PIN  4
#define DECREASE_CHANNEL_1_PIN  16
#define INCREASE_CHANNEL_2_PIN  2
#define DECREASE_CHANNEL_2_PIN  5

#define DIMMER_1_PIN  16
#define DIMMER_2_PIN  17
#define SDA_PIN  26
#define SCL_PIN  27

#define LEDstep 25

#define pcfAdd 0x20 


ezButton button1(INCREASE_CHANNEL_1_PIN);  // create ezButton object that attach to pin 6;
ezButton button2(DECREASE_CHANNEL_1_PIN);  // create ezButton object that attach to pin 7;
ezButton button3(INCREASE_CHANNEL_2_PIN);  // create ezButton object that attach to pin 8;
ezButton button4(DECREASE_CHANNEL_2_PIN);  // create ezButton object that attach to pin 8;

TaskHandle_t Task1;
Preferences NVS;

PCF8574 pcf(&Wire ,pcfAdd);


bool increament1 = false;
bool decreament1 = false;
bool increament2 = false;
bool decreament2 = false;
bool leftLedState = false;
bool rightLedState = false;
bool newTouchFlag = false;
bool magnetSens_1 = false; 
bool magnetSens_2 = false; 

const int PwmFrequency = 500;
const int LED_Channel_1 = 0;
const int LED_Channel_2 = 1;
const int PwmResolution = 8;
int inc = 0;

int channelDuty_1 = 0;
int channelDuty_2 = 0;
int lastChannelDuty_1 = 0;
int lastChannelDuty_2 = 0;
int savedvalue = 0;

uint32_t holdTime1 = 0;
uint32_t holdTime2 = 0;
uint32_t holdTime3 = 0;
uint32_t holdTime4 = 0;
uint32_t debounseMag1 = 0;
uint32_t debounseMag2 = 0;

const char *addrDuty1 = "Addduty1";
const char *addrDuty2 = "Addduty2";


void IRAM_ATTR ExtI_pcf() {
  newTouchFlag = true;
}

void pcfInit(void) {
  pcf.pinMode(P0, INPUT);
	pcf.pinMode(P1, INPUT);
  pcf.pinMode(P2, INPUT);
	pcf.pinMode(P3, INPUT);
  pcf.pinMode(P4, INPUT);
	pcf.pinMode(P5, INPUT);
  pcf.pinMode(P6, INPUT);
	pcf.pinMode(P7, INPUT);

	Serial.print("Init pcf...");

	if (pcf.begin(SDA_PIN ,SCL_PIN)) {
		Serial.println("OK");
	} else{
		Serial.println("Not Found");
	}
}

void interruptInit(void) {
  Serial.println("interruptInit");
  pinMode(14, INPUT_PULLUP);

  attachInterrupt(14, ExtI_pcf, FALLING);
}

void IRAM_ATTR magnet_1_ISR() {
  magnetSens_1 = true;
}

void IRAM_ATTR magnet_2_ISR() {
  magnetSens_2 = true;
}

void PWM_Init(void) {
  ledcSetup(LED_Channel_1, PwmFrequency, PwmResolution);
  ledcSetup(LED_Channel_2, PwmFrequency, PwmResolution);
  
  ledcAttachPin(DIMMER_1_PIN, LED_Channel_1);
  ledcAttachPin(DIMMER_2_PIN, LED_Channel_2);

  ledcWrite(LED_Channel_1, channelDuty_1);
  ledcWrite(LED_Channel_2, channelDuty_2);

  pinMode(33 ,INPUT_PULLUP);
  attachInterrupt(33 ,magnet_1_ISR ,CHANGE);

  pinMode(25 ,INPUT_PULLUP);
  attachInterrupt(25 ,magnet_2_ISR ,CHANGE);
}

void Task1code( void * parameter) {
  for(;;) {
    if (rightLedState == true) {
      if ((channelDuty_1 > lastChannelDuty_1)) {
        lastChannelDuty_1++;
        ledcWrite(LED_Channel_1, lastChannelDuty_1);
      } else {
        lastChannelDuty_1--;
        ledcWrite(LED_Channel_1, lastChannelDuty_1);
      }
    } else {
      lastChannelDuty_1 = 0;
      ledcWrite(LED_Channel_1, lastChannelDuty_1);
    }

    if (leftLedState == true) {
      if ((channelDuty_2 > lastChannelDuty_2)) {
        lastChannelDuty_2++;
        ledcWrite(LED_Channel_2, lastChannelDuty_2);
      } else {
        lastChannelDuty_2--;
        ledcWrite(LED_Channel_2, lastChannelDuty_2);
      }
    } else {
      lastChannelDuty_2 = 0;
      ledcWrite(LED_Channel_2, lastChannelDuty_2);
    }

    vTaskDelay(5);
  }
}

void setup() {
  Serial.begin(115200);
  NVS.begin("SM1001", false);
  // channelDuty_1 = NVS.getInt(addrDuty1);
  // channelDuty_2 = NVS.getInt(addrDuty2);
  savedvalue = NVS.getInt(addrDuty2);
  channelDuty_1 = savedvalue;
  channelDuty_2 = savedvalue;

  PWM_Init();
  interruptInit();
  pcfInit();

  // button1.setDebounceTime(50); 
  // button2.setDebounceTime(50);
  // button3.setDebounceTime(50); 
  // button4.setDebounceTime(50);

  xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */

  debounseMag1 = millis();
  debounseMag2 = millis();
}

void loop() {

  if (newTouchFlag) {
    Serial.println("Touch Handler --> Touched");
    newTouchFlag = false;

		PCF8574::DigitalInput di = pcf.digitalReadAll();

    if(di.p6 == HIGH) {
      Serial.println("The button 6 is pressed");

      channelDuty_1 += LEDstep;
      channelDuty_2 += LEDstep;
      if (channelDuty_1 > 250) channelDuty_1 = 250;
      if (channelDuty_2 > 250) channelDuty_2 = 250;

      Serial.print("channel 1 INC : ");  Serial.println(channelDuty_1);

      holdTime1 = millis();
      increament1 = true;
    } else if(di.p6 == LOW) {
      Serial.println("The button 6 is released");
      increament1 = false;
    }
    if (increament1 && (millis() - holdTime1) > 500) {
      channelDuty_1 += LEDstep;
      channelDuty_2 += LEDstep;
      if (channelDuty_1 > 250) channelDuty_1 = 250;
      if (channelDuty_2 > 250) channelDuty_2 = 250;

      Serial.print("channel 1 INC : ");  Serial.println(channelDuty_1);

      holdTime1 = millis();
    }

    if(di.p2 == HIGH) {
      Serial.println("The button 2 is pressed");

      channelDuty_1 -= LEDstep;
      channelDuty_2 -= LEDstep;
      if (channelDuty_1 < 100) channelDuty_1 = 100;
      if (channelDuty_2 < 100) channelDuty_2 = 100;
      

      Serial.print("channel 1 DIC : "); Serial.println(channelDuty_1);

      decreament2 = true;
      holdTime2 = millis();
    } else if(di.p2 == LOW) {
      Serial.println("The button 2 is released");
      decreament2 = false;
    }
    if (decreament2 && (millis() - holdTime2) > 500) {
      channelDuty_1 -= LEDstep;
      channelDuty_2 -= LEDstep;
      if (channelDuty_1 < 100) channelDuty_1 = 100;
      if (channelDuty_2 < 100) channelDuty_2 = 100;

      Serial.print("channel 1 DIC : "); Serial.println(channelDuty_1);
      
      holdTime2 = millis();
    }

    if(di.p3 == HIGH) {
      Serial.println("The button 3 is pressed");
      leftLedState = !leftLedState;
      Serial.println("leftLedState : "); Serial.println(leftLedState);
    }

    if(di.p5 == HIGH) {
      Serial.println("The button 5 is pressed");
      rightLedState = !rightLedState;
      Serial.println("rightLedState : "); Serial.println(rightLedState);
    }

    if (channelDuty_1 != savedvalue)
    {
      savedvalue = channelDuty_1;
      NVS.putInt(addrDuty2 ,savedvalue);
    }
  }

  if (magnetSens_1) {
    if ((millis() - debounseMag1) > 500) {
      if (digitalRead(33)) {
        leftLedState = false;
      } else {
        leftLedState = true;
      }
      magnetSens_1 = false;
      Serial.println("Start mg1");
      debounseMag1 = millis();
    }
  }

  if (magnetSens_2) {
    if ((millis() - debounseMag2) > 500) {
      if (digitalRead(25)) {
        rightLedState = false;
      } else {
        rightLedState = true;
      }
      magnetSens_2 = false;
      debounseMag2 = millis();
    }
  }
}
