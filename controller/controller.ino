#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include <Servo.h>
#include <SoftwareSerial.h>

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

extern uint8_t packetbuffer[];

void moveTo(uint16_t x, uint16_t y);
void circle(void);
void wander(void);
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

const int16_t wx = 15;  // X width (blue)
const int16_t sx = 36;  // X starting position
const int16_t wy = 20; // Y width (40 max)
const int16_t sy = 35; // Y starting position
const uint8_t d = 100;   // delay

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);
Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
Servo servo1; 
Servo servo2;

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup(void)
{
  Serial.begin(115200);

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  ble.echo(false);
  ble.info();
  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  ble.setMode(BLUEFRUIT_MODE_DATA);
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
   servo1.attach(5);
   servo2.attach(6); 
  
  int max=170;
  int min = 27;
  int x = min;
  int y = min;
  int xInc = 0;
  int yInc = 0;

  pinMode(7, OUTPUT); 
  servo1.write(max - ((max-min)/2));
  servo2.write(max - ((max-min)/2));

  while(1)
  {
    uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
    if (len != 0)
    {
      // Buttons
      if (packetbuffer[1] == 'B') 
      {
        uint8_t buttnum = packetbuffer[2] - '0';
        boolean pressed = packetbuffer[3] - '0';
        Serial.println(buttnum);
        Serial.println(pressed);
        
        switch(buttnum)
        {
          case 1:
          {
             if (pressed)
             {
                wander(); 
             }
             break;
          }
          case 2:
          {
            if (pressed)
            {
              circle(); 
            }
            break;
          };
          case 3:
          {
            if (pressed)
            {
              digitalWrite(7,HIGH); 
            }
            break;
          }
          case 4:
          {
            if (pressed)
            {
              digitalWrite(7, LOW); 
            }
            break;
          }
        
          case 6:
          {
            if (pressed)
            {
              xInc = 1;        
            }
            else
            {
              xInc = 0;
            }
            break;
          }
        
          case 5:
          {
            if (pressed)
            {
              xInc = -1;
            }
            else
            {
              xInc = 0;
            }
            break;
          }
        
          case 7:
          {
            if (pressed)
            {
              yInc = 1;     
            }
            else
            {
              yInc = 0;
            }
            break;
          }
          case 8:
          {
            if (pressed)
            {
              yInc = -1;
            }
            else
            {
              yInc = 0;
            }
            break;
          }
        } /// end switch    
      } // end button
    } // end len=0
  
      x = x + xInc;
      if (x > max)
      {
        x = max;
      }
      if (x < min)
      {
        x = min;
      }
    
      y = y + yInc;
      if (y > max)
      {
         y = max;
      }
      if (y < min)
      {
        y = min;
      }
      servo1.write(x);
      servo2.write(y);
      delay(50);

  }
}


//*******************************
//* wander()
//*
//*******************************
void wander(void)
{
   int16_t xIncrement;  
   int16_t yIncrement;
   int8_t xPos=64;
   int8_t yPos=64;
      
   randomSeed(analogRead(0));
   for(int i = 0; i < 1000; i++)
   {
      xIncrement = random(-1, 2);
      yIncrement = random(-1, 2);
      xPos += xIncrement;
      yPos += yIncrement;
      if (xPos > (sx + wx))
      {
        xPos = (sx + wx);
      }
      if (xPos < sx)
      {
        xPos = sx;
      }
      if (yPos > (sy + wy))
      {
        yPos = (sy + wy);
      }
      if (yPos < sy)
      {
        yPos = sy;
      }
      moveTo(xPos, yPos);
      delay(d/8);
   }
}

//*******************************
//* circle()
//*
//*******************************
void circle(void)
{
   double t;
   uint16_t x;
   uint16_t y;
   int rx=20;
   int ry=15;
   
  // while(1)
   {
     for (t = 0; t < 6.28; t += .01)
     {
        x = (cos(t) + 1) * (rx / 2) + sx;
        y = (sin(t) + 1) * (ry / 2) + sy;
        moveTo(x,y);
        delay(10);
     }  
   }
}

void moveTo(uint16_t x, uint16_t y)
{
   servo1.write(x);
   servo2.write(y);
}
