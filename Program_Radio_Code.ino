#define FREQUENCY    160                  // valid 80, 160
#include "ESP8266WiFi.h"
extern "C" {
#include "user_interface.h"
}
void pp_soft_wdt_stop();    // close software watchdog

byte const pin0=D0; //Play
byte const pin1=D1; //Next
byte const pin2=D2; //Prev

long unsigned int canID;
unsigned char len;
unsigned char buf[8];

#include <SPI.h>
#include "mcp_can.h"


MCP_CAN CAN(D8);                                    // D8 for esp8266 setup plus cheap MCP board, 10 for the elecfreaks on mega


void setup() {
Serial.begin(230400);

  WiFi.forceSleepBegin();                  // turn off ESP8266 RF
  delay(1);                                // give RF section time to shutdown
  system_update_cpu_freq(FREQUENCY);

  while (CAN_OK != CAN.begin(CAN_50KBPS, MCP_16MHz))              // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS no es bueno!"); // Ref to AvE
    delay(1000);
  }
  
  // put your setup code here, to run once:
 pinMode(D1,INPUT);
}

//void bufprint(unsigned char buf[8], byte l) {
//  for (int i = 0; i < l; i++) // print the data (for debugging)
//  {
//    Serial.print(buf[i]);
//    if (i < l - 1) {
//      Serial.print(",");
//      Serial.print("\t");
//    }
//  }
//  Serial.print("\n");
//  return;
//}

void Play(){
  pinMode(pin0,OUTPUT);
  digitalWrite(pin0,LOW);
  delay(300);
  pinMode(pin0,INPUT);
//  Serial.println("Play");
}

void Next(){
  pinMode(pin1,OUTPUT);
  digitalWrite(pin1,LOW);
  delay(300);
  pinMode(pin1,INPUT);
//  Serial.println("Next");
}
void Prev(){
  pinMode(pin2,OUTPUT);
  digitalWrite(pin2,LOW);
  delay(300);
  pinMode(pin2,INPUT);
//  Serial.println("Prev");
}
void loop() {
//  Play();
//  delay(5000);
  
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data is coming
  {
      CAN.readMsgBufID(&canID, &len, buf);   // read data,  len: data length, buf: data buf
  }
//  if (canID==656 ){
//   Serial.print(canID);
//   Serial.println(":");
//   bufprint(buf,len);
//  }
// 64=vol up; 128= vol down; 32=src; 4=nxt; 16=next; 8=prev

 if (canID==656){ //can message for steering wheel buttons
   
   if (buf[0]==128){ //changed since the last message
          if (buf[4]==4){
            Play();
          }
          if (buf[4]==16){
            Next();
          }
          if (buf[4]==8){
            Prev();
          }
        }      
   }
}
