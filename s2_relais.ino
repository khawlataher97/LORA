#include "Arduino.h"
#include "LoRa_E32.h"

#define M0  12
#define M1  14
#define AUX 15
#define option      4 
//#define led 2
//int ledPin=2; //definition digital 8 pins as pin to control the LED

LoRa_E32 e32ttl(&Serial2,15,12,14);
uint8_t config_struct[6];
const int digPin=13;

void setup() {

  Serial.begin(9600);
  
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  
  e32ttl.begin();

  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  
  
  delay(1000);
   uint8_t config_struct[6];
   config_struct[0] = 0xC0; //Store in internal flash
   config_struct[1] = 0x08; // Address High, set a different value on the receiving side
   config_struct[2] = 0xA6; //Address Low
   config_struct[3] = 0x18; //8N1, 9600 Baud rate, 19.2 kbps Air data rate AB-0x1A
   config_struct[4] = 0x07; //433 MHz
   config_struct[5] = 0xC0; //Default Options

   Serial2.write(config_struct,6);
   delay(2000);

   Serial.println("Config Complete");

   digitalWrite(M0, LOW);
   digitalWrite(M1, LOW);

   

   pinMode(digPin, OUTPUT);
//   pinMode(ledPin,OUTPUT);
}

void loop()
{   while (e32ttl.available()  > 1)
    
      {ResponseContainer r = e32ttl.receiveMessage();
       String Data = r.data;
       Serial.println(r.status.getResponseDescription());
       Serial.println(Data);
       if (Data=="ON")
       {digitalWrite(digPin, HIGH);
        digitalWrite(option, HIGH);;
        ResponseStatus r = e32ttl.sendBroadcastFixedMessage(2,"ON");}
        else if (Data=="Of")
       { digitalWrite(digPin, LOW);
         digitalWrite(option, LOW);
       ResponseStatus r = e32ttl.sendBroadcastFixedMessage(2,"Of");}
        
       }
       
  }
    
    
    
   

  
  
