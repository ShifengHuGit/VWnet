/*  send a frame from can bus
    support@longan-labs.cc
    
    CAN Baudrate,
    
    #define CAN_5KBPS           1
    #define CAN_10KBPS          2
    #define CAN_20KBPS          3
    #define CAN_25KBPS          4 
    #define CAN_31K25BPS        5
    #define CAN_33KBPS          6
    #define CAN_40KBPS          7
    #define CAN_50KBPS          8
    #define CAN_80KBPS          9
    #define CAN_83K3BPS         10
    #define CAN_95KBPS          11
    #define CAN_100KBPS         12
    #define CAN_125KBPS         13
    #define CAN_200KBPS         14
    #define CAN_250KBPS         15
    #define CAN_500KBPS         16
    #define CAN_666KBPS         17
    #define CAN_1000KBPS        18
    
    CANBed V1: https://www.longan-labs.cc/1030008.html
    CANBed M0: https://www.longan-labs.cc/1030014.html
    CAN Bus Shield: https://www.longan-labs.cc/1030016.html
    OBD-II CAN Bus GPS Dev Kit: https://www.longan-labs.cc/1030003.html
*/
   
#include <mcp_can.h>
#include <SPI.h>
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
MCP_CAN CAN(SPI_CS_PIN);// Set CS pin
CANFunction CANUDS(&CAN, false);
struct Message_t txMsg, rxMsg;



int pid =0;



unsigned char ShortCodeMsg_0x710_StartSession[2] =    { 0x10, 0x03 };

unsigned char ShortCodeMsg_0x7E0_GetEngRPM[3] =       { 0x22, 0xf4, 0x0C};
unsigned char ShortCodeMsg_0x7E0_GetEngWLD[3] =       { 0x22, 0xf4, 0x43};
unsigned char ShortCodeMsg_0x7E0_GetEngSTS[3] =       { 0x22, 0x39, 0x53};
unsigned char ShortCodeMsg_0x7E0_GetEngFuzai[3] =     { 0x22, 0x11, 0xE9};
unsigned char ShortCodeMsg_0x7E0_GetCoolantTemp[3] =  { 0x22, 0xf4, 0x05};
unsigned char ShortCodeMsg_0x7E0_GetThroPST[3] =      { 0x22, 0xf4, 0x11};
unsigned char ShortCodeMsg_0x7E0_GetOutsideTemp[3] =  { 0x22, 0xf4, 0x46};
unsigned char ShortCodeMsg_0x7E0_GetMileAge[3] =      { 0x22, 0x29, 0x5A};
unsigned char ShortCodeMsg_0x7E0_GetTubroPrs[3] =     { 0x22, 0x20, 0x29};
unsigned char ShortCodeMsg_0x7E0_GetOilLevel[3] =     { 0x22, 0x20, 0x06};

unsigned char ShortCodeMsg_0x773_GPSInfo[3] =         { 0x22, 0x24, 0x30};
unsigned char ShortCodeMsg_0x773_GPSDate[3] =         { 0x22, 0x22, 0xB3};
unsigned char ShortCodeMsg_0x773_GPSFlowCtrl[2] =     { 0x00, 0x01};
unsigned char ShortCodeMsg_0x7E1_PadelPst[3] =        { 0x22, 0x38, 0x08};
unsigned char ShortCodeMsg_0x714_CkpSpeed[3] =        { 0x22, 0x22, 0xD2};
unsigned char ShortCodeMsg_0x714_Tml30Vlt[3] =        { 0x22, 0x02, 0x86};
unsigned char ShortCodeMsg_0x714_CkpEngTmp[3] =       { 0x22, 0x20, 0x2F};

struct UDS_Cmd CmdList[18] = {
{ 0, "Session Control",      2,        { 0x10, 0x03, 0x00} , CANMSGID_TESTER_SESSION, CANMSGID_ECU_SESSION, 0  },
{ 0, "Flow Control",             2,    { 0x00, 0x01, 0x00} , CANMSGID_TESTER_GPS,   CANMSGID_ECU_GPS, 0  },
{ 1, "Data GetEngRPM",           3,    { 0x22, 0xf4, 0x0C} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 2, "Data GetEngWLD",         3,      { 0x22, 0xf4, 0x43} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 3, "Data GetEngSTS",         3,      { 0x22, 0x39, 0x53} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 4, "Data GetEngFuzai",     3,        { 0x22, 0x11, 0xE9} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 5 ,"Data GetCoolantTemp",    3,      { 0x22, 0xf4, 0x05} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 6, "Data GetThroPST",      3,        { 0x22, 0xf4, 0x11} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 7, "Data GetOutsideTemp",  3,        { 0x22, 0xf4, 0x46} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 8, "Data GetMileAge",      3,        { 0x22, 0x29, 0x5A} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 9, "Data GetTubroPrs",     3,        { 0x22, 0x20, 0x29} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 10, "Data GetOilLevel",     3,        { 0x22, 0x20, 0x06} , CANMSGID_TESTER_ENG,   CANMSGID_ECU_ENG, 0  },
{ 11, "Data GPSInfo",         3,        { 0x22, 0x24, 0x30} , CANMSGID_TESTER_GPS,   CANMSGID_ECU_GPS, 1  },
{ 12, "Data GPSDate"     ,    3,        { 0x22, 0x22, 0xB3} , CANMSGID_TESTER_GPS,   CANMSGID_ECU_GPS, 1  },
{ 13, "Data PadelPst",      3,          { 0x22, 0x38, 0x08} , CANMSGID_TESTER_GEARBOX, CANMSGID_ECU_GEARBOX, 0  },
{ 14, "Data CkpSpeed",      3,          { 0x22, 0x22, 0xD2} , CANMSGID_TESTER_COCKPIT, CANMSGID_ECU_COCKPIT, 0  },
{ 15, "Data Tml30Vlt",      3,          { 0x22, 0x02, 0x86} , CANMSGID_TESTER_COCKPIT, CANMSGID_ECU_COCKPIT, 0  },
{ 16, "Data CkpEngTmp",     3,          { 0x22, 0x20, 0x2F} , CANMSGID_TESTER_COCKPIT, CANMSGID_ECU_COCKPIT, 0  } 
};
/*
String HandleData(int cmdid)
{
  switch(cmdid)
  case GetEngRPM | 
  
  
  }
*/

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
        Serial.println(LTE_Resp);
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
  Serial.println("Start to Init the LTE Module");
  String ResponseString;
  
  // LTE Module needs more than 10s to complete its initial phase.
  delay(5000);
  if(Serial1.available() > 0)
      { 
        ResponseString = Serial1.readString();
        Serial.println(ResponseString);
        
      }
      
  //Enter AT Config Mode;
  Serial1.write("+++");

  delay(500);
 // if(SendATCmd("AT+CREG") =="+OK=1")
  {
    Serial.println("Register OK");
  }

  Serial.println(SendATCmd("AT+CSQ"));
  Serial.println(SendATCmd("AT+SOCK1"));
  Serial.println(SendATCmd("AT+LBS"));
  Serial.println(SendATCmd("AT+LINKSTA1"));

  //Exit AT Config Mode;
  Serial1.write("AT+EXAT\r\n");

  while(1){
      
      Serial1.write("INIT");
      Serial.println("Initialize the Session with Cloud Server, waitting the INIT_ACK");
      delay(800);
      if(Serial1.available() > 0)
      { 
        
        ResponseString = Serial1.readString();
        if (ResponseString == "INIT_ACK")
        {
          Serial.println("Session bulit, ready to talk with Cloud Server!");
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

{    /*
    NetStr["ID"];
    NetStr["GetEngRPM"];
    NetStr["GetEngWLD"];
    NetStr["GetEngSTS"];
    NetStr["GetEngFuzai"];
    NetStr["GetCoolantTemp"];
    NetStr["GetThroPST"];
    NetStr["GetOutsideTemp"];
    NetStr["GetMileAge"];
    NetStr["GetTubroPrs"];
    NetStr["GetOilLevel"];
    NetStr["GetOilLevel"];
    NetStr["GetOilLevel"];
    NetStr["GetOilLevel"];
    NetStr["GPSInfo"];
    NetStr["GPSDate"];
    NetStr["GPSFlowCtrl"];
    NetStr["PadelPst"];
    NetStr["CkpSpeed"];
    NetStr["Tml30Vlt"];
    NetStr["CkpEngTmp"];

    serializeJson(NetStr, Serial1); */
    
    

    
    //serializeJson(doc, Serial1);

   txMsg.Buffer = (uint8_t *)calloc(MAX_MSGBUF, sizeof(uint8_t));
   rxMsg.Buffer = (uint8_t *)calloc(MAX_MSGBUF, sizeof(uint8_t));
    
    String ResponseString;

    Serial1.begin(115200);
    while(!Serial1);
    Serial.begin(115200);
    while(!Serial);

    //initLTE();
    initVWCAN();
    delay(200);

}


void loop()
{
  delay(1100);
  /*
    StaticJsonDocument<256> doc;
    
    char date[]={0x01,0x16,0x05,0x07,0x04,0x3A,0x20};
    doc["PID"] = pid;
    doc["GetEngRPM"] = 0x0AF9;
    doc["GetEngWLD"] = 0x0AF9;
    doc["GetEngSTS"] = 0x0AF9;
    doc["GetEngFuzai"] = 0x0AF9;
    doc["GetCoolantTemp"] = 0x0AF9;
    doc["GetThroPST"] = 0x0AF9;
    doc["GetOutsideTemp"] = 0x0AF9;
    doc["GetMileAge"] = 0x0AF9;
    doc["GetTubroPrs"] = 0x0AF9;
    doc["GetOilLevel"] = 0x77;
    doc["GPSInfo"] = "N121.38.231;E38.18'13";
    //doc["GPSDate"] = date;
    doc["PadelPst"] = 70;
    doc["CkpSpeed"] = 45;
    doc["Tml30Vlt"] = 13;
    doc["CkpEngTmp"] = 90;
        
    JsonArray GPSDate = doc.createNestedArray("GPSDate");
    
    GPSDate.add(20);
    GPSDate.add(22);
    GPSDate.add(5);
    GPSDate.add(7);
    GPSDate.add(13);
    GPSDate.add(23);
    GPSDate.add(56);
    
    serializeJson(doc, Serial1);
    pid++;
 */
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
        CANUDS.print_buffer(rxMsg.rx_id, rxMsg.Buffer, rxMsg.len);
        CANUDS.receive(&rxMsg);
        }
      CANUDS.print_buffer(rxMsg.rx_id, rxMsg.Buffer, rxMsg.len);

      delay(200);
    }

    /*TODO - Json it and send it via LTE */
    

    

/*
    NetStr["GetEngRPM"]=        CANFunction(CANMSGID_TESTER_ENG, CANMSGID_ECU_ENG, Msg_0x7E0_GetEngRPM     );
    NetStr["GetEngWLD"] =       CANFunction(CANMSGID_TESTER_ENG, CANMSGID_ECU_ENG, Msg_0x7E0_GetEngWLD     );
    NetStr["GetEngSTS"] =       CANFunction(CANMSGID_TESTER_ENG, CANMSGID_ECU_ENG, Msg_0x7E0_GetEngSTS     );
    NetStr["GetEngFuzai"] =     CANFunction(CANMSGID_TESTER_ENG, CANMSGID_ECU_ENG, Msg_0x7E0_GetEngFuzai   );
    NetStr["GetCoolantTemp"] =  CANFunction(CANMSGID_TESTER_ENG, CANMSGID_ECU_ENG, Msg_0x7E0_GetCoolantTemp);
    NetStr["GetThroPST"] =      CANFunction(CANMSGID_TESTER_ENG, CANMSGID_ECU_ENG, Msg_0x7E0_GetThroPST    );
    NetStr["GetOutsideTemp"] =  CANFunction(CANMSGID_TESTER_ENG, CANMSGID_ECU_ENG, Msg_0x7E0_GetOutsideTemp);
    NetStr["GetMileAge"] =      CANFunction(CANMSGID_TESTER_ENG, CANMSGID_ECU_ENG, Msg_0x7E0_GetMileAge    );
    NetStr["GetTubroPrs"] =     CANFunction(CANMSGID_TESTER_ENG, CANMSGID_ECU_ENG, Msg_0x7E0_GetTubroPrs   );
    NetStr["GetOilLevel"] =     CANFunction(CANMSGID_TESTER_ENG, CANMSGID_ECU_ENG, Msg_0x7E0_GetOilLevel   );
    NetStr["GPSInfo"] =         CANFunction(CANMSGID_TESTER_GPS, CANMSGID_ECU_GPS, Msg_0x773_GPSInfo       );
    NetStr["GPSDate"] =         CANFunction(CANMSGID_TESTER_GPS, CANMSGID_ECU_GPS, Msg_0x773_GPSDate       );
    NetStr["PadelPst"] =        CANFunction(CANMSGID_TESTER_GEARBOX, CANMSGID_ECU_GEARBOX, Msg_0x7E1_PadelPst      );
    NetStr["CkpSpeed"] =        CANFunction(CANMSGID_TESTER_COCKPIT, CANMSGID_ECU_COCKPIT, Msg_0x714_CkpSpeed      );
    NetStr["Tml30Vlt"] =        CANFunction(CANMSGID_TESTER_COCKPIT, CANMSGID_ECU_COCKPIT, Msg_0x714_Tml30Vlt      );
    NetStr["CkpEngTmp"] =       CANFunction(CANMSGID_TESTER_COCKPIT, CANMSGID_ECU_COCKPIT, Msg_0x714_CkpEngTmp    );
*/
}




// END FILE
