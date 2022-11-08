#include "Arduino.h"
#include "LoRa_E32.h"
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;


#define M0  12
#define M1  14
#define AUX 15

int intervalle = 30000;
long lastSendTime = 0;        // last send time

LoRa_E32 e32ttl(&Serial2,15,21,19);
uint8_t config_struct[6];
const int digPin=13;
#define option      4                

#include <ArduinoJson.h>

/*const int dry = 3620; // value for dry sensor
const int wet = 1700; // value for wet sensor*/

DynamicJsonDocument doc(512);


void Send(int H, int L, int chan,String Data)
{ Serial.println("Send message to 01 04 04");
  ResponseStatus rs = e32ttl.sendFixedMessage(H, L, chan, Data);
  Serial.println(rs.getResponseDescription());
}
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
   config_struct[1] = 0x00; // Address High, set a different value on the receiving side
   config_struct[2] = 0x00; //Address Low
   config_struct[3] = 0x18; //8N1, 9600 Baud rate, 19.2 kbps Air data rate AB-0x1A
   config_struct[4] = 0x02; //433 MHz
   config_struct[5] = 0xC0; //Default Options

   Serial2.write(config_struct,6);
   delay(2000);

   Serial.println("Config Complete");

   digitalWrite(M0, LOW);
   digitalWrite(M1, LOW);

   /*Serial.println();
   Serial.println("Start listening!");*/

   pinMode(digPin, OUTPUT);
     if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
     }
   pinMode(option, OUTPUT);

}
/*void sensor(){
  int sensorVal = analogRead(36);
  int percentageHumididy = map(sensorVal, wet, dry, 100, 0); 
  doc["percentageHumididy %"]=percentageHumididy;
  String buf;
  serializeJson(doc,buf);
 // Send(1,4,4,buf);
         ResponseStatus r = e32ttl.sendBroadcastFixedMessage(4,buf);

  //Send(8,0xA6,7,buf);
  //Serial2.print(buf);
  Serial.print(buf);

}*/
void sensor(){
    float p = (bmp.readPressure()/100);
    float t = bmp.readTemperature(); 
    doc["temperature"]=t;
    doc["pression"]=p;
   String buf;
   serializeJson(doc,buf);
   //ResponseStatus r = e32ttl.sendBroadcastFixedMessage(4,buf);
   Send(1,4,4,buf);
   Serial.print(buf);

}
void loop()
{   
  if (millis() - lastSendTime > 30000) 
          { 
            sensor();
            lastSendTime = millis();            // timestamp the message
          }
  while (e32ttl.available()  > 1)
   {ResponseContainer r = e32ttl.receiveMessage();
    String Data = r.data;
    Serial.println(r.status.getResponseDescription());
    Serial.println(Data);
    if (Data=="ON")
    {digitalWrite(option, HIGH); }  // turn the LED on (HIGH is the voltage level)

    
     else if (Data=="Of")
     {digitalWrite(option, LOW);}
     else if (Data == "on")
     {digitalWrite(digPin, HIGH);
     digitalWrite(option, HIGH);
      //ResponseStatus r = e32ttl.sendBroadcastFixedMessage(7,"ON");
      }
      else if (Data == "off")
     {digitalWrite(digPin, LOW);
     digitalWrite(option, LOW);
      //ResponseStatus r = e32ttl.sendBroadcastFixedMessage(7,"Off");
      }
     }
}

  

  
    
    
   

  
  
