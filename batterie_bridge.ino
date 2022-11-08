#include <PubSubClient.h>
#include <Wire.h>
#include "Arduino.h"
#include "LoRa_E32.h"
#include <ArduinoJson.h>
#include <virtuabotixRTC.h>
virtuabotixRTC myRTC(32, 33, 25);//sclk/i-o/

uint32_t chipId = 0;
char buffer1[30]; 

#define TINY_GSM_MODEM_SIM800 // Modem is SIM800L
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1


// Your GPRS credentials, if any
const char apn[] = "internet.ooredoo.tn";//"weborange";//"internet.tn"; 
const char gprsUser[] = "";
const char gprsPass[] = "";

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = ""; 
#define GSM_PIN ""

#define SMS_TARGET  "+216xxxxxxxx"

// MQTT details
const char* broker = "public.cloud.shiftr.io";;                    
const char* mqttUsername = "public";  // MQTT username
const char* mqttPassword = "public";  // MQTT password

const char* topicOutput1 = "public/relay/Rly1";
const char* topicOutput2 = "public/relay/Rly2";
const char* topicTemperature = "public/tmp";
const char* topicpression = "public/Pression";
const char* topicGust = "public/Rafale";
const char* topicAverage = "public/SpeedAverage";
const char* topicrain = "public/RainNow";
const char* topicdaily = "public/RainDay";
const char* topicpower = "public/power";

#include <TinyGsmClient.h>
//#include <PubSubClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif


TinyGsmClient client(modem);
PubSubClient mqtt(client);


#define MODEM_TX             27
#define MODEM_RX             26


#define OUTPUT_1             0
#define OPTION                  4
const int analogInPin = 2;  // Analog input pin that the potentiometer is attached to
int sensorValue = 0;        // value read from the pot
#define M0  12
#define M1  14
#define AUX 15
LoRa_E32 e32ttl(&Serial2,15,12,14);

uint32_t lastReconnectAttempt = 0;


float temperature = 0;
float humidity = 0;
long lastMsg = 0;
int interval = 2000;          // interval between sends
long lastSendTime = 0;        // last send time

uint8_t config_struct[6];
//..........Anemometre.............
#define ANEMOMETRE 39 //carte 39   //pin D3, interruption n°1
#define PI        3.1415
#define RAYON     0.065  //rayon en mètre de l'anémomètre en mètre

volatile unsigned long Rotations         = 0;       // Cup rotation counter used in interrupt routine
unsigned long WindSampleTimePrevMillis   = 0;       // Store the previous millis   
unsigned long ReportTimerLongPrevMillis  = 0;       // Store the previous millis
unsigned long  lastWindIRQ               =0;
const unsigned long WindSampleTime       = 1000;    // Timer in milliseconds for windspeed (gust)calculation    
const unsigned long ReportTimerLong      = 600000;  // Timer in milliseconds (10 min) to report temperature, average wind and restart max/min calculation      

float windSpeed                          = 0;       // Wind speed in miles per hour sampled during WindSampleTime
float windSpeed_sum                      = 0;       // Summarizing of all windspeed measurements to calculate average wind
float WindSpeedAverage                   = 0;       // Average wind measured during ReportIntervall
float windSamples                        = 0;
float windGustin                         = 0;

//.....................Pluvioètre..........
#define PLUVIOMETRE 34    //interruption n°0
#define VALEUR_PLUVIOMETRE 0.3  //valeur en mm d'eau à chaque bascule d'auget
volatile unsigned int countPluviometre = 0;
int currentPulseCount;
unsigned long dailyPulseCount;
float currentRain;
float DailytRain;
unsigned long  previousMillis =  0;
unsigned long delaipluviometre =  1000;   //1 sec

ICACHE_RAM_ATTR void interruptAnemometre()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  float deltaTime = micros() - lastWindIRQ;
  if (deltaTime > 15) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = micros(); //Grab the current time
    
    Rotations++; //There is 1.492MPH for each click per second.
    //Serial.print(Rotations);
  }
}
 
void get_WindSpeed() {
  if((millis() - WindSampleTimePrevMillis) > WindSampleTime ) {   // Calculate gust wind speed  .....windsampletime=2000                 

  Serial.print("Rotations "); 
  Serial.println(Rotations);
                
    
  windSpeed  = 2*PI*RAYON*3.6;
  windSpeed       *= Rotations;
  Rotations        = 0; //after I take the sample I do need to reset the number of clicks!

  if (windSpeed > 0){
  windSpeed_sum   += windSpeed;
  windSamples++;
  Serial.print("  WindSpeed (km/h)  "); 
  Serial.println(windSpeed);}

   
  float averageWindSpeed=(windSpeed_sum / windSamples);

  
  WindSampleTimePrevMillis = millis();
  }
}
void windGust(){
   if (windSpeed > windGustin)
       {windGustin = windSpeed;}
       }

void getAverageWindSpeed() {
    
     if (windSamples > 0) {
       WindSpeedAverage = windSpeed_sum / windSamples; }  //  Calculate average windspeed, if rotations is 0 the division fails 
     else {WindSpeedAverage = 0;}                                                                           
}

void resetAverageWindSpeed() {

    WindSpeedAverage = 0;
    windSamples = 0;
    windSpeed_sum = 0;
    windGustin=0;
}
//....................
ICACHE_RAM_ATTR void interruptPluviometre(){
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // debounce for a quarter second = max. 4 counts per second
  if (interrupt_time - last_interrupt_time > 350){//350milliseconde
  countPluviometre++;
  Serial.print(countPluviometre);
  last_interrupt_time = interrupt_time;}
  }
void interruptrain(){
   unsigned long currentMillis   = millis(); // read time passed 
  if (currentMillis - previousMillis >= delaipluviometre){
    noInterrupts();
    currentPulseCount+=countPluviometre; // add to current counter
    countPluviometre=0; // reset ISR counter
    interrupts();
    dailyPulseCount+=currentPulseCount;
    currentRain=currentPulseCount*VALEUR_PLUVIOMETRE;
    currentPulseCount=0;
    DailytRain=dailyPulseCount*VALEUR_PLUVIOMETRE;
    /*Serial.print("currentRain : ");
    Serial.println(currentRain);*/
    char rainString[8];
    dtostrf(currentRain, 1, 2, rainString);
    mqtt.publish(topicrain, rainString); 
    char dayString[8];
    dtostrf(DailytRain, 1, 2, dayString);
    mqtt.publish(topicdaily, dayString);
    previousMillis=millis();
    }   
}
//................
void Battery_level(){
    sensorValue = analogRead(analogInPin);
    float batteryLevel = map(sensorValue, 0.0f, 4095.0f, 0, 100);
    char powerString[8];
    dtostrf(batteryLevel, 1, 2, powerString);
    mqtt.publish(topicpower, powerString);
}

void setup_lora(){
  e32ttl.begin();

  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  

  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);

  delay(1000);
   
   config_struct[0] = 0xC0; //Store in internal flash
   config_struct[1] = 0x01; // Address High, set a different value on the receiving side
   config_struct[2] = 0x04; //Address Low
   config_struct[3] = 0x18; //8N1, 9600 Baud rate, 19.2 kbps Air data rate AB-0x1A
   config_struct[4] = 0x04; //433 MHz
   config_struct[5] = 0xC0; //Default Options

   Serial2.write(config_struct,6);
   delay(2000);

   Serial.println("Config Complete");

   digitalWrite(M0, LOW);
   digitalWrite(M1, LOW);

}
void callback(char* topic, byte* message, unsigned int len) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < len; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == topicOutput1) {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(OUTPUT_1, HIGH);
      ResponseStatus r = e32ttl.sendBroadcastFixedMessage(2,"on");
  String smsMessage = "OUTPUT_1 is on";    
 if(modem.sendSMS(SMS_TARGET, smsMessage))
    SerialMon.println(smsMessage);//
 else{    SerialMon.println("SMS failed to send");  }//
    }
    else {
      Serial.println("off");
      digitalWrite(OUTPUT_1, LOW);
      ResponseStatus r = e32ttl.sendBroadcastFixedMessage(2,"off");
    }
  }

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);


  // Authenticate MQTT:
  boolean status = mqtt.connect("GsmClientN", mqttUsername, mqttPassword);

  if (status == false) {
    SerialMon.println(" fail");//
    ESP.restart();
    return false;
  }
  SerialMon.println(" success");//
  mqtt.subscribe(topicOutput1);

  return mqtt.connected();
}


void setup() {
  // Set console baud rate
  SerialMon.begin(9600);//
  setup_lora();
  delay(5000);
    myRTC.setDS1302Time(00, 43, 1, 3, 7, 9, 2022);
   
   for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
   
  }
   sprintf(buffer1,"%X",chipId);Serial.println(buffer1);

   
  
  pinMode(OUTPUT_1, OUTPUT);
  pinMode(OPTION, OUTPUT);
  setup_modem();
  

  pinMode(ANEMOMETRE,INPUT_PULLUP); 
  attachInterrupt(ANEMOMETRE,interruptAnemometre,FALLING) ;
  pinMode(PLUVIOMETRE,INPUT_PULLUP); 
  attachInterrupt(PLUVIOMETRE,interruptPluviometre,FALLING) ;
}
void setup_modem(){
  SerialMon.println("Wait...");

  // Set GSM module baud rate and UART pins
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");//
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");//
  SerialMon.println(modemInfo);//

  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }

    
  SerialMon.print("Connecting to APN: ");//
  SerialMon.print(apn);//
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");//
    ESP.restart();
     digitalWrite(OPTION, LOW);
  }
  else {
    SerialMon.println(" OK");//
  }
  
  if (modem.isGprsConnected()) {
    Serial.println("GPRS connected");digitalWrite(OPTION, HIGH);//
  }
 
  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(callback);
}

void loop() {
  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }
  DynamicJsonDocument jsonBuffer(512);
   while (e32ttl.available()  > 1)
    {    
     ResponseContainer r = e32ttl.receiveMessage();
     String Data = r.data;
     Serial.println(Data);
     DeserializationError error =deserializeJson(jsonBuffer ,Data); 
     float temperature =jsonBuffer["temperature"];
     float pression =jsonBuffer["pression"];
    

      
      if(temperature>=30)// && temperature <=30)
    {
      ResponseStatus r = e32ttl.sendBroadcastFixedMessage(7,"ON");
    }
    else if (temperature <30)
    {
     ResponseStatus r = e32ttl.sendBroadcastFixedMessage(7,"Of");
    }
    delay(5000);
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    mqtt.publish(topicTemperature, tempString);
    char presString[8];
    dtostrf(pression, 1, 2, presString);
    mqtt.publish(topicpression, presString);
}
    get_WindSpeed();
    windGust();
    interruptrain();
    Battery_level();
    myRTC.updateTime();
    if((myRTC.hours== 23)&&(myRTC.minutes==59)) {
      dailyPulseCount= currentPulseCount;
     }

    if((millis() - ReportTimerLongPrevMillis) > ReportTimerLong ) { 

        getAverageWindSpeed(); 
        Serial.print("Average WindSpeed (m/s) "); 
        Serial.print(WindSpeedAverage);
        char AverageString[8];
        dtostrf(WindSpeedAverage, 1, 2, AverageString);
        mqtt.publish(topicAverage, AverageString);  //  Calculate average windspeed, if rotations is 0 the division fails 
        Serial.print("windGustin "); 
        Serial.print(windGustin);
        Serial.println("");
        char GustString[8];
        dtostrf(windGustin, 1, 2, GustString);
        mqtt.publish(topicGust, GustString);  //  Calculate average windspeed, if rotations is 0 the division fails 
        
        ReportTimerLongPrevMillis = millis();
        resetAverageWindSpeed();}
  
  mqtt.loop();
}

void Send1(int H, int L, int chan,String Data)
{ Serial.println("Send message to 05 C5 08");
  ResponseStatus rs = e32ttl.sendFixedMessage(H, L, chan, Data);
  Serial.println(rs.getResponseDescription());
}
void Send2(int H, int L, int chan,String Data)
{ Serial.println("Send message to 08 A6 07");
  ResponseStatus rs = e32ttl.sendFixedMessage(H, L, chan, Data);
  Serial.println(rs.getResponseDescription());
}

void Recieve()
{ if (e32ttl.available()  > 1)
   {ResponseContainer rs = e32ttl.receiveMessage();
    String message = rs.data;
    Serial.println(rs.status.getResponseDescription());
    Serial.println(message);
   }
}
