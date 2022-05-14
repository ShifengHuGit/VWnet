#include <mcp_can.h>
#include <SPI.h>
#include "base64.hpp"
#include "CANFunction.h"
#include <ArduinoJson.h>

/* Please modify SPI_CS_PIN to adapt to different baords.

   CANBed V1        - 17
   CANBed M0        - 3
   CAN Bus Shield   - 9
   CANBed 2040      - 9
   CANBed Dual      - 9
   OBD-2G Dev Kit   - 9
   Hud Dev Kit      - 9
*/
#define SPI_CS_PIN  17
const int LED  = 23;
MCP_CAN CAN(SPI_CS_PIN);// Set CS pin
CANFunction CANUDS(&CAN, false);
struct Message_t txMsg, rxMsg;
int pid =0;
int count = 0;
unsigned char ShortCodeMsg_0x710_StartSession[2] =    { 0x10, 0x03 };


struct UDS_Cmd CmdList[18] = {
{ 0,  "Session Control",      2,   { 0x10, 0x03, 0x00} , CANMSGID_TESTER_SESSION, CANMSGID_ECU_SESSION, 0  },
{ 0,  "Control",             2,    { 0x00, 0x01, 0x00} , CANMSGID_TESTER_GPS,   CANMSGID_ECU_GPS, 0  },
{ 1,  "GetEngRPM",           3,    { 0x22, 0xf4, 0x0C} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 2,  "GetEngWLD",         3,      { 0x22, 0xf4, 0x43} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 3,  "GetEngSTS",         3,      { 0x22, 0x39, 0x53} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 4,  "GetEngFuzai",     3,        { 0x22, 0x11, 0xE9} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 5 , "GetCoolantTemp",    3,      { 0x22, 0xf4, 0x05} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 6,  "GetThroPST",      3,        { 0x22, 0xf4, 0x11} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 7,  "GetOutsideTemp",  3,        { 0x22, 0xf4, 0x46} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 8,  "GetMileAge",      3,        { 0x22, 0x29, 0x5A} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 9,  "GetTubroPrs",     3,        { 0x22, 0x20, 0x29} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 10, "GetOilLevel",     3,        { 0x22, 0x20, 0x06} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 11, "GPSInfo",         3,        { 0x22, 0x24, 0x30} , CANMSGID_TESTER_GPS,   CANMSGID_ECU_GPS, 1  },
{ 12, "GPSDate"     ,    3,        { 0x22, 0x22, 0xB3} , CANMSGID_TESTER_GPS,   CANMSGID_ECU_GPS, 1  },
{ 13, "PadelPst",      3,          { 0x22, 0x38, 0x08} , CANMSGID_TESTER_GEARBOX, CANMSGID_ECU_GEARBOX, 0  },
{ 14, "CkpSpeed",      3,          { 0x22, 0x22, 0xD2} , CANMSGID_TESTER_COCKPIT, CANMSGID_ECU_COCKPIT, 0  },
{ 15, "Tml30Vlt",      3,          { 0x22, 0x02, 0x86} , CANMSGID_TESTER_COCKPIT, CANMSGID_ECU_COCKPIT, 0  },
{ 16, "CkpEngTmp",     3,          { 0x22, 0x20, 0x2F} , CANMSGID_TESTER_COCKPIT, CANMSGID_ECU_COCKPIT, 0  } 
};

//
// SEND AT CMD to LTE via SERIAL1 Port
// E.G. "AT+CSP";
//
 
String SendATCmd(char* CMD)
{

  unsigned char LTE_ERR[9] = {0x0d, 0x0a, 0x45, 0x52, 0x52, 0x4f, 0x52, 0x0d, 0x0a};
  String LTE_Resp;

  Serial1.write(CMD);
  Serial1.write("\r\n");
  
  delay(800);
  if(Serial1.available() > 0)
      { 
        LTE_Resp = Serial1.readString();
        //Serial.println(LTE_Resp);
        if(LTE_Resp == LTE_ERR )
        {return "ERROR";}
        else
        {return LTE_Resp;}
      }
}


//
// Initialize the LTE Module
//
// Check the Status of LTE
//
void initLTE()
{
  pinMode(LED_BUILTIN, OUTPUT);
  //Serial.println("Start to Init the LTE Module");
  String ResponseString;
  
  // LTE Module needs more than 10s to complete its initial phase.
  delay(7000);
  if(Serial1.available() > 0)
      { 
        ResponseString = Serial1.readString();
    //    Serial.println(ResponseString);
        
      }
      
  //Enter AT Config Mode;
  Serial1.write("+++");

  delay(500);
 // if(SendATCmd("AT+CREG") =="+OK=1")
  {
    //Serial.println("Register OK");
  }

  //Serial.println(SendATCmd("AT+CSQ"));
  //Serial.println(SendATCmd("AT+SOCK1"));
  //Serial.println(SendATCmd("AT+LBS"));
  //Serial.println(SendATCmd("AT+LINKSTA1"));

  //Exit AT Config Mode;
  Serial1.write("AT+EXAT\r\n");

  while(1){
      
      Serial1.write("INIT");
    //  Serial.println("Initialize the Session with Cloud Server, waitting the INIT_ACK");
      delay(800);
      if(Serial1.available() > 0)
      { 
        
        ResponseString = Serial1.readString();
        if (ResponseString == "INIT_ACK")
        {
      //    Serial.println("Session bulit, ready to talk with Cloud Server!");
          Serial1.write("ACK");
          break;
        }  
      }
    }
  
}



void  initVWCAN()
{
  Serial.println("Initialize CAN BUS!");

  while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS FAIL!");
        delay(100);
    }  
    /*
     * set mask, set both the mask to 0x3ff
     */
    CAN.init_Mask(0, 0, 0x0700);                         // there are 2 mask in mcp2515, you need to set both of them
   
    /*
     * set filter, we can receive id from 0x04 ~ 0x09
     */
    CAN.init_Filt(0, 0, CANMSGID_ECU_SESSION);                          // there are 6 filter in mcp2515
    CAN.init_Filt(1, 0, CANMSGID_ECU_ENG);                          // there are 6 filter in mcp2515
    
    CAN.init_Mask(1, 0, 0x0700);
    CAN.init_Filt(2, 0, CANMSGID_ECU_GEARBOX);                          // there are 6 filter in mcp2515
    CAN.init_Filt(3, 0, CANMSGID_ECU_GPS);                          // there are 6 filter in mcp2515
    CAN.init_Filt(4, 0, CANMSGID_ECU_COCKPIT);                          // there are 6 filter in mcp2515
    CAN.init_Filt(5, 0, 0x0701);                          // there are 6 filter in mcp2515


    // Send Initialize Session CAN request
    //  {0x02, 0x10, 0x03, 0x55, 0x55,0x55,0x55,0x55};
    //          ^      ^
    //          |      |
    //      Session   Extended 
    // 
      // Send this to CAN Bus
      // 发送 进入EXT会话的请求

    txMsg.tx_id = CANMSGID_TESTER_SESSION;
    txMsg.rx_id = CANMSGID_ECU_SESSION;
    txMsg.len = sizeof(ShortCodeMsg_0x710_StartSession);
    memcpy(txMsg.Buffer,ShortCodeMsg_0x710_StartSession,txMsg.len);
  
    rxMsg.tx_id = CANMSGID_TESTER_SESSION;
    rxMsg.rx_id = CANMSGID_ECU_SESSION;
      
    CANUDS.send(&txMsg);
    CANUDS.receive(&rxMsg);
    CANUDS.print_buffer(rxMsg.rx_id, rxMsg.Buffer, rxMsg.len);
    //CanID-【7a 07】 Can Data->【 06-|50 03 00 32 01 f4 aa |] 
    if(rxMsg.Buffer[0] != 0x50)
    {
      Serial.println("Failed to initialize the CAN Session !!!! ");
      exit(0);
     }
     else
     {
      Serial.println("Successfully initialize the CAN Session ~!!~ ");
     }
}

void setup()

{  
 
  Serial1.begin(115200);
  while(!Serial1);
 // Serial.begin(115200);
 // while(!Serial);

  txMsg.Buffer = (uint8_t *)calloc(MAX_MSGBUF, sizeof(uint8_t));
  rxMsg.Buffer = (uint8_t *)calloc(MAX_MSGBUF, sizeof(uint8_t));
  initLTE();

  digitalWrite(LED_BUILTIN, HIGH);
  
  initVWCAN();
  delay(200);

}


void loop()
{
  
  delay(100);
  StaticJsonDocument<384> doc;
  unsigned char tmp[5];
  if(count%2==0){
    digitalWrite(LED_BUILTIN, HIGH);
  }
/*
    doc["PID"] = pid;
    doc["GetEngRPM"] = "AAA=";
    doc["GetEngWLD"] = "AAA=";
    doc["GetEngSTS"] = "AA==";
    doc["GetEngFuzai"] = "AA==";
    doc["GetCoolantTemp"] = "QQ==";
    doc["GetThroPST"] = "KA==";
    doc["GetOutsideTemp"] = "OQ==";
    doc["GetMileAge"] = "AJVN";
    doc["GetTubroPrs"] = "A/Q=";
    doc["GetOilLevel"] ="AFw=";
    doc["GPSInfo"] = "121.32'27.6\"E  38.53'15.7\"N ";
    doc["GPSDate"] = "ARYFCQkpAA==";
    doc["PadelPst"] = "CA==";
    doc["CkpSpeed"] = "AAA=";
    doc["Tml30Vlt"] = "eQ==";
    doc["CkpEngTmp"] = "UQ==";

    pid++;
    //serializeJson(doc, Serial);
    serializeJson(doc, Serial1);
  */

    doc["PID"] = pid;

    for(int x=2; x<18; x++)
    {
      
      txMsg.tx_id=CmdList[x].tx_id;
      txMsg.rx_id=CmdList[x].rx_id;
      txMsg.len = CmdList[x].len; //SID+IDs
      rxMsg.tx_id = CmdList[x].tx_id;
      rxMsg.rx_id = CmdList[x].rx_id;
      memcpy(txMsg.Buffer, CmdList[x].Code,   txMsg.len);
      
      CANUDS.send(&txMsg);
      
      CANUDS.receive(&rxMsg);
      
      if(CmdList[x].FrameFlag)
      {
        //CANUDS.print_buffer(rxMsg.rx_id, rxMsg.Buffer, rxMsg.len);
        CANUDS.receive(&rxMsg);
        /*18:08:18.512 -> Buffer: 7DD [31] 62 24 30 31 32 31 B0 33 32 27 32 39 2E 34 22 45 20 20 33 38 B0 35 33 27 31 35 2E 35 22 4E 20 
          18:08:18.760 -> Buffer: 7DD [3] 7F 22 78 
          18:08:18.798 -> Buffer: 7DD [10] 62 22 B3 01 16 05 07 0A 08 12 
*/
        if(CmdList[x].id == 12){ // GPS date
          unsigned char LongTmp[9];
          encode_base64(rxMsg.Buffer+3, rxMsg.len-3, LongTmp);
          doc[CmdList[x].desc] = LongTmp;
        }
        else
        { // GPS info  , including a Non-Standard ASCII 0xB0, this not able to be supported to decode by Python Jsonload.
          for(int i = 3; i<rxMsg.len; i++)
          {
              if(rxMsg.Buffer[i]==0xB0 || rxMsg.Buffer[i]==0x27 || rxMsg.Buffer[i]==0x22){
                rxMsg.Buffer[i]=0x2E;
              }else{ 
                continue;
              }
          }
          doc[CmdList[x].desc] = rxMsg.Buffer+3;
        }
       }
      else
      {
        encode_base64(rxMsg.Buffer+3, rxMsg.len-3, tmp);
        doc[CmdList[x].desc] = tmp;
      }
      delay(55);
     }
     pid++;
     
     /*TODO - Json it and send it via LTE */

    serializeJson(doc, Serial1);
    digitalWrite(LED_BUILTIN, LOW);
    count++;

    
}




// END FILE
