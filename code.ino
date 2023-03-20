#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"
#include <Arduino.h>
#include <hp_BH1750.h>
#include <FastLED.h>
#define NUM_LEDS 256
#define DATA_PIN 10
hp_BH1750 BH1750;
VR myVR(2,3);    
CRGB leds[NUM_LEDS];
uint8_t records[7]; 
uint8_t buf[64];

#define led 11
#define buzz 5

#define onRecord    (0)
#define offRecord   (1) 
#define partyMode   (2)

void printSignature(uint8_t *buf, int len)
{
  int i;
  for(i=0; i<len; i++){
    if(buf[i]>0x19 && buf[i]<0x7F){
      Serial.write(buf[i]);
    }
    else{
      Serial.print("[");
      Serial.print(buf[i], HEX);
      Serial.print("]");
    }
  }
}

void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if(buf[0] == 0xFF){
    Serial.print("NONE");
  }
  else if(buf[0]&0x80){
    Serial.print("UG ");
    Serial.print(buf[0]&(~0x80), DEC);
  }
  else{
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if(buf[3]>0){
    printSignature(buf+4, buf[3]);
  }
  else{
    Serial.print("NONE");
  }
  Serial.println("\r\n");
}

void setup()
{
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  myVR.begin(9600);
  bool avail = BH1750.begin(BH1750_TO_GROUND);
  Serial.begin(9600);
  Serial.println("Elechouse Voice Recognition V3 Module\r\nControl LED sample");
  
  pinMode(led, OUTPUT);
  pinMode(buzz, OUTPUT);

    
  if(myVR.clear() == 0){
    Serial.println("Recognizer cleared.");
  }else{
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    while(1);
  }
  
  if(myVR.load((uint8_t)onRecord) >= 0){
    Serial.println("onRecord loaded");
  }
  
  if(myVR.load((uint8_t)offRecord) >= 0){
    Serial.println("offRecord loaded");
  }

  if(myVR.load((uint8_t)partyMode) >= 0){
    Serial.println("partyMode loaded");
  }
  
  
}

bool chicken = false;

void loop()
{
  int ret;
  float analogValue = analogRead(A3);
  float iv = 3*analogValue/205;
  if(iv < 9.5 && chicken == false){
    for(int i = 0; i < 3; i++){
      tone(buzz, 1000);
      delay(1000);     
      noTone(buzz);    
      delay(1000);
    }
    chicken = true;
  }
  ret = myVR.recognize(buf, 50);
  if(ret>0){
    switch(buf[1]){
      case onRecord:
        for(int i = 0; i < 256; i++){
              leds[i] = CRGB::Red;
        }
        FastLED.show();
        while(1){
          ret = myVR.recognize(buf, 50);
          float lux = readlight();
          if(lux <= 10){
            float y = 10.0-lux;
            int z = y*25.5;
            analogWrite(led,z);
          }
          else{
            analogWrite(led,0);
          }
          if(iv < 9.5 && chicken == false){
            for(int i = 0; i < 3; i++){
              tone(buzz, 1000);
              delay(1000);     
              noTone(buzz);    
              delay(1000);
            }
            chicken = true;
          }
          if(buf[1] == offRecord){
            digitalWrite(led, LOW);
            for(int i = 0; i < 256; i++){
              leds[i] = CRGB::Black;
            }
            FastLED.show(); 
            break;
          }
        }
        break;
      case offRecord:
        /** turn off LED*/
        digitalWrite(led, LOW);
        break;
      case partyMode:
        while(1){
          pride();
          FastLED.show();
        }
        break;
      default:
        Serial.println("Record function undefined");
        break;
    }
    /** voice recognized */
    printVR(buf);
  }
}

float readlight(){
    BH1750.start(); 
    float lux=BH1750.getLux(); 
    Serial.println(lux);
    return lux;
}

void pride(){
  digitalWrite(led, HIGH);
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
 
  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);
  
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;
  
  for( uint16_t i = 0 ; i < NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    
    CRGB newcolor = CHSV( hue8, sat8, bri8);
    
    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS-1) - pixelnumber;
    
    nblend( leds[pixelnumber], newcolor, 64);
  }
  delay(100);
  digitalWrite(led, LOW);
  delay(100);
}
