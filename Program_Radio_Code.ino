#define FREQUENCY    160                  // valid 80, 160
unsigned char stmp[8];

#define FIRST 0 //Auth methods
#define SECOND 1
/* initialize */
int method = SECOND;
unsigned int Empty;
float tic = 0;
float toc = 0;
int ReceiveFilters[5]={0};
byte EngStatus=1;
unsigned int EngRPM;
unsigned int TempRPM;
unsigned int EngP;
unsigned int TempP;
long  unsigned         SValues[5];

unsigned char ACK[4] = {64, 161,  63, 128}; // Acknowledge seed
unsigned int  ACKID = 614;

unsigned char Dinit[7] = {63, 129,  0,  17, 2,  64, 0};
unsigned int  DinitID = 544;

unsigned char DinitAck[8] = {64, 191,  33, 193,  0,  17, 2,  88};
unsigned int  DinitAckID = 568;

unsigned char SecAccReq[8] = {64,  161,  2,  39, 5,  0,  0,  0}; // Req security access
unsigned int  SecAccReqID = 576;

unsigned char SeedRcv[8] = {192,  191,  4,  103,  5,  201,  39, 0}; // receive seed 201 39
unsigned int  SeedRcvID = 600;

unsigned char SeedAns[8] = {64,  161,  4,  39, 6,  145,  72, 0}; // Send calculated answer
unsigned int  SeedAnsID = 576;

unsigned char Data[8] = {192, 191, 5, 99, 240,  94, 248, 12};
unsigned int  DataID = 600;

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
Serial.begin(9600);

//  WiFi.forceSleepBegin();                  // turn off ESP8266 RF
  delay(1);                                // give RF section time to shutdown
//  system_update_cpu_freq(FREQUENCY);

  while (CAN_OK != CAN.begin(CAN_50KBPS, MCP_16MHz))              // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS no es bueno!"); // Ref to AvE
    delay(1000);
  }
  
  // put your setup code here, to run once:

Serial.println("start"); ///begin sending
datainit(1);


unsigned char stmp11[8] = {0x3f, 0x81, 0x00, 0x28, 0x02, 0x40, 0x00, 0x00};
CAN.sendMsgBuf(0x220, 0, 8, stmp11); //
delay(70);


unsigned char stmp12[8] = {0x3f, 0x81, 0x00, 0x81, 0x02, 0x40, 0x00, 0x00};
CAN.sendMsgBuf(0x220, 0, 8, stmp12); //
delay(70);



unsigned char stmp1[8] = {0x40, 0x91, 0x01, 0x3E, 0x00, 0x00, 0x00, 0x00 };
CAN.sendMsgBuf(0x240, 0, 8, stmp1); //tester present
delay(70);

unsigned char ack[8] = {0x40, 0x91, 0x3F, 0x80, 0x00, 0x00, 0x00, 0x00 };
CAN.sendMsgBuf(0x266, 0, 8, ack); //Ack
delay(10);



unsigned char stmp2[8] = {0x40, 0x91, 0x05, 0x23, 0x00, 0xF7, 0x14, 0x02 };
CAN.sendMsgBuf(0x240, 0, 8, stmp2); //tester present
delay(70);

CAN.sendMsgBuf(0x266, 0, 8, ack); //Ack
delay(10);



unsigned char stmp3[8] = {0x40, 0x91, 0x02, 0x31, 0x50, 0x00, 0x00, 0x00 };
CAN.sendMsgBuf(0x240, 0, 8, stmp3); //tester present
delay(70);

CAN.sendMsgBuf(0x266, 0, 8, ack); //Ack
delay(10);



unsigned char stmp4[8] = {0x40, 0x91, 0x05, 0x23, 0x00, 0xF7, 0x14, 0x02 };
CAN.sendMsgBuf(0x240, 0, 8, stmp4); //tester present
delay(70);

CAN.sendMsgBuf(0x266, 0, 8, ack); //Ack
delay(10);


unsigned char stmp5[8] = {0x40, 0x91, 0x01, 0x82, 0x00, 0x00, 0x00, 0x00 };
CAN.sendMsgBuf(0x240, 0, 8, stmp5); //tester present
delay(70);

CAN.sendMsgBuf(0x266, 0, 8, ack); //Ack
delay(10);
Serial.println("Done");//end sending
}

void loop() {

  
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data is coming
  {
      CAN.readMsgBufID(&canID, &len, buf);   // read data,  len: data length, buf: data buf
//      Serial.println(canID);//debug if works properly
  }
//
// if (canID==656){ //can message for steering wheel buttons
//   
//   if (buf[0]==128){ //changed since the last message
//          if (buf[4]==4){
//            Play();
//          }
//          if (buf[4]==16){
//            Next();
//          }
//          if (buf[4]==8){
//            Prev();
//          }
//        }      
//   }
}


void datainit(int printed) {
  while (true) {
    CAN.sendMsgBuf(DinitID, 0, 7, Dinit);  // Initialize the T7 data communication
    buf[8] = MessageWait(DinitAckID);
    
    delay(0);
    int Correct = isCorrectFrame(buf, DinitAck, 0, 3);

    if (Empty == 1 || Correct == 0) {
      if (printed==1){ Serial.println(String("Reinitializing ECU"));}
      CAN.sendMsgBuf(DinitID, 0, 8, Dinit);
      delay(500);
    }
    else {
      if (printed==1){Serial.println("ECU is ready to communicate");}
      break;
    }
  }
}

void authentication(int printed) {
  while (true) {

    method = FIRST + SECOND - method; //toggle method between 0 and 1
    uint16_t seed = CalcKey(method, SeedRcv);
    SeedAns[5] = seed >> 8 & 0xff;
    SeedAns[6] = seed & 0xff;
    CAN.sendMsgBuf(SeedAnsID, 0, 8, SeedAns);  // Send back the calculated authentication
    buf[8] = MessageWait(DataID); // Acknowledge and return data frame
    int Correct = isCorrectFrame(buf, SeedRcv, 0, 3);
    if (Correct == 1) {
      SeedRcv[5] = buf[5];
      SeedRcv[6] = buf[6];
    }


    delay(0);

    if (Empty == 1 ) {
      if (printed==1){Serial.println(String("Cannot get access, trying again"));}
      delay(500);
      break;
    }
    if (buf[3] == 103 && buf[5] == 52) {
      if (printed==1){Serial.println("Security access OK");}
      if (printed==1){Serial.println("We are requesting data");}
      break;
    }
  }
}


uint16_t CalcKey(uint8_t method, unsigned char ReceivedSeed[8]) {

  uint16_t seed;
  seed = ReceivedSeed[5] << 8 | ReceivedSeed[6];
  seed  = seed << 2 & 0xffff;
  seed ^= (method ? 0x4081 : 0x8142);
  seed -= (method ? 0x1f6f : 0x2356);
  return seed;
}


unsigned char MessageWait(int ID)
{
  tic = millis();
  toc = tic;
  while (true)           // check if data coming
  {

    tic = millis();
    if ((tic - toc) > 5) // give up waiting if DATA is not received in time
    {
      memset(buf, 0, 8 * sizeof(buf[0]));
      Empty=1;
      return buf[8];
    }
    if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data is coming
    {
      CAN.readMsgBufID(&canID, &len, buf);   // read data,  len: data length, buf: data buf
    for (int i=0;i<5;i++){ // print messages that are interesting according to the filter
      if (canID==ReceiveFilters[i]){ReceivePrint(canID,len,buf);}
    }
      if (canID == 416) {
        EngStatus=buf[0];
        SValues[0] = buf[1] << 8 | buf[2] << 0;
        SValues[20] = buf[3];
        SValues[21] = buf[4];
        SValues[22] = buf[6];
        SValues[23] = buf[7];
        TempRPM = SValues[0];//
        TempP= buf[3];
        if (EngRPM!=TempRPM && EngP!=TempP){
          EngRPM=TempRPM;
          EngP=TempP;
          EngStatus=1;}
        if (EngRPM==TempRPM && EngP==TempP){EngStatus=0;}
        //      SValues[1] = buf[3] ;
        
        SValues[1] = buf[5] ;

      }
      if (canID == 768) {
        SValues[6] = (buf[0] << 8 | buf[1] << 0) ; //Speed FL
        SValues[7] = (buf[2] << 8 | buf[3] << 0) ; //Speed FR
        SValues[8] = (buf[4] << 8 | buf[5] << 0) ; //Speed RL
        SValues[9] = (buf[6] << 8 | buf[7] << 0) ; //Speed RR
      }
      if (canID == 288) {
        SValues[10] = buf[7] ; //Brake pressure
        SValues[11] = buf[4] ; // Steering angle

        SValues[14] = buf[0] ; // Unknown
        SValues[15] = buf[1] ; // Unknown
        SValues[16] = buf[2] ; // Unknown
        SValues[17] = buf[3] ; // Unknown
        SValues[18] = buf[5] ; // Unknown
        SValues[19] = buf[6] ; // Unknown
      }
      if (canID == 384) {
        SValues[12] = buf[3] ; //180_3 Unknown
        SValues[13] = buf[4] ; // 180_4 Unknown
      }
            
      if (canID == 928) {
        SValues[2] = (buf[3] << 8 | buf[4] << 0) / 10; //Speed
      }


      if (canID == 288) {
        SValues[10] = buf[7] ; //Brake pressure
        SValues[11] = buf[4] ; // Steering angle

        SValues[14] = buf[0] ; // Unknown
        SValues[15] = buf[1] ; // Unknown
        SValues[16] = buf[2] ; // Unknown
        SValues[17] = buf[3] ; // Unknown
        SValues[18] = buf[5] ; // Unknown
        SValues[19] = buf[6] ; // Unknown
      }
      

      if (canID == 384) {
        SValues[12] = buf[3] ; //180_3 Unknown
        SValues[13] = buf[4] ; // 180_4 Unknown
      }

//      if (canID == 801) {
//        DigFreqVal[0] = (buf[1] << 8) | buf[0];
//        DigFreqVal[1] = (buf[3] << 8) | buf[2];
//        DigFreqVal[2] = (buf[5] << 8) | buf[4];
//        DigFreqVal[3] = (buf[7] << 8) | buf[6];
//      }
//      if (canID == 802) {
//        AnValues[1] = (buf[1] << 8) | buf[0];
//        AnValues[2] = (buf[3] << 8) | buf[2];
//        AnValues[3] = (buf[5] << 8) | buf[4];
//        AnValues[4] = (buf[7] << 8) | buf[6];
//      }
//      if (canID == 803) {
//        AnValues[0] = (buf[1] << 8) | buf[0];
//        AcX = (buf[3] << 8) | buf[2];
//        AcY = (buf[5] << 8) | buf[4];
//        AcZ = (buf[7] << 8) | buf[6];
//      }

      if (canID == 1472) {
        SValues[3] = (buf[1] - 40) ;
        SValues[4] = buf[3] << 8 | buf[4] << 0;

      
      }

      if (ID == canID)  {
        SendACK(buf[0]);
        Empty=0;
        return buf[8];
      }
    }
  }
}

int isCorrectFrame(unsigned char buf[8], unsigned char Comp[8], int n1, int n2) {
  int CorrectFrame = 1;
  for (int n = n1; n < n2; n++) {
    if (Comp[n] != buf[n]) {
      CorrectFrame = 0; // test each element to be the same. if not, return false
    }
  }
  return CorrectFrame;
}

void ReceivePrint(int canID, int len, byte buf[8]){
  Serial.print("RX,");
  Serial.print(canID);
  Serial.print(",");
  for (int i=0;i<len;i++){
    Serial.print(buf[i]);
    if (i!=len-1){
    Serial.print(",");
    }
  } 
  Serial.println("");
}

void SendACK(int row) { //Make an ACK frame based on the receved frame
  if (row == 26) {
    ACK[3] = 195;
  }
  if (row == 195) {
    ACK[3] = 131;
  }
  if (row == 130) {
    ACK[3] = 130;
  }
  if (row == 129 || row == 193) {
    ACK[3] = 129;
  }
  if (row == 192 || row == 128) {
    ACK[3] = 128;
  }
  CAN.sendMsgBuf(ACKID, 0, 4, ACK);
  return;
}
