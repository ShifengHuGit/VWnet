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




unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];
StaticJsonDocument<100> NetStr;


unsigned char Msg_0x710_StartSession[8] =    { 0x02, 0x10, 0x03, 0x55, 0x55, 0x55, 0x55, 0x55};

unsigned char Msg_0x7E0_GetEngRPM[8] =       { 0x03, 0x22, 0xf4, 0x0C, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x7E0_GetEngWLD[8] =       { 0x03, 0x22, 0xf4, 0x43, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x7E0_GetEngSTS[8] =       { 0x03, 0x22, 0x39, 0x53, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x7E0_GetEngFuzai[8] =     { 0x03, 0x22, 0x11, 0xE9, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x7E0_GetCoolantTemp[8] =  { 0x03, 0x22, 0xf4, 0x05, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x7E0_GetThroPST[8] =      { 0x03, 0x22, 0xf4, 0x11, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x7E0_GetOutsideTemp[8] =  { 0x03, 0x22, 0xf4, 0x46, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x7E0_GetMileAge[8] =      { 0x03, 0x22, 0x29, 0x5A, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x7E0_GetTubroPrs[8] =     { 0x03, 0x22, 0x20, 0x29, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x7E0_GetOilLevel[8] =     { 0x03, 0x22, 0x20, 0x06, 0x55, 0x55, 0x55, 0x55};


unsigned char Msg_0x773_GPSInfo[8] =         { 0x03, 0x22, 0x24, 0x30, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x773_GPSDate[8] =         { 0x03, 0x22, 0x22, 0xB3, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x773_GPSFlowCtrl[8] =     { 0x30, 0x00, 0x01, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

unsigned char Msg_0x7E1_PadelPst[8] =        { 0x03, 0x22, 0x38, 0x08, 0x55, 0x55, 0x55, 0x55};

unsigned char Msg_0x714_CkpSpeed[8] =        { 0x03, 0x22, 0x22, 0xD2, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x714_Tml30Vlt[8] =        { 0x03, 0x22, 0x02, 0x86, 0x55, 0x55, 0x55, 0x55};
unsigned char Msg_0x714_CkpEngTmp[8] =       { 0x03, 0x22, 0x20, 0x2F, 0x55, 0x55, 0x55, 0x55};




//17F00010

#define SPI_CS_PIN  17 
MCP_CAN CAN(SPI_CS_PIN);// Set CS pin

struct Message_t txMsg, rxMsg;


CANFunction CANUDS(&CAN, false);
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
  int count=0;
  char R_SIDCode = 0x00;

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
      CAN.sendMsgBuf(CANMSGID_TESTER_SESSION, 0, 8, Msg_0x710_StartSession);
  
      // Logging the Request
      Serial.print("U-->C:");
      for(int i = 0; i<8; i++)    // print the data
          {
              Serial.print("0x");
              Serial.print(Msg_0x710_StartSession[i], HEX);
              Serial.print("\t");
          }
       Serial.println("<---END---");



    // Check whether the Response coming.
    //

  while(1)
  {

      while(CAN_MSGAVAIL != CAN.checkReceive());
    /*  {
        Serial.println("No Data!");
        delay(1500);
      }*/
      
      CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

       if(CAN.getCanId() != CANMSGID_ECU_SESSION)
       {
          Serial.print("ID not match: ");
          Serial.println(CAN.getCanId(),HEX);
          continue;
       }
       else
       {
        
          Serial.print("U<--C: ID:");
          Serial.println(CAN.getCanId(),HEX);
          Serial.print(" Len:");
          Serial.println(len);
          //记录Response结果
          for(int i = 0; i<8; i++)    // print the data
          {
              Serial.print("0x");
              Serial.print(buf[i], HEX);
              Serial.print("\t");
          }
          Serial.println(">---END---");

  
          R_SIDCode = buf[1];
          
          if(R_SIDCode == 0x7f || R_SIDCode != 0x50 || buf[2] != 0x03)
          {
             Serial.println("Retry Build the session");
             Serial.println("Failed to initialize the CAN Session ! ");
             CAN.sendMsgBuf(CANMSGID_TESTER_SESSION, 0, 8, Msg_0x710_StartSession);
             continue;
          }
          else
          {
             Serial.println("Successfully Initialize the CAN Session!");
             break;
          }

        }
  }
    
}

int Hander_ENG_RPM(char H, char L)
{
  //( 0x0A * 256 + 0xF9 ) / 4 
  return (((int)H*256+(int)L)/4);
}

void setup()

{    
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

    serializeJson(NetStr, Serial1);

  txMsg.Buffer = (uint8_t *)calloc(MAX_MSGBUF, sizeof(uint8_t));
  rxMsg.Buffer = (uint8_t *)calloc(MAX_MSGBUF, sizeof(uint8_t));
    
    String ResponseString;

    Serial1.begin(115200);
    while(!Serial1);
    Serial.begin(115200);
    while(!Serial);

    //Initilaize the CAN session.
    // Enter the Extend Session : 0x02 0x10 0x03 X X X X;
   

    initLTE();

    initVWCAN();
}

char * CANFunction(int CanTest_ID, int CanECU_ID, unsigned char CMDSTR[8])
{

  int UDS_PayloadSize=0;
  int NumberofLoop=0;
  int PendingByte=0;
  int SavedBytes=0;
  int loops=0;

  char buf[8];
  char Data2X[100];

  //发送命令,等待回应,阻塞态.
  CAN.sendMsgBuf(CanTest_ID, 0, 8, CMDSTR);

  
  while(1)
      {
          //阻塞在等待数据,如果有数据了.继续执行
          while(CAN_MSGAVAIL != CAN.checkReceive());
        /*  {
            Serial.println("No Data!");
            delay(1500);
          }*/
          //读取数据
          CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

            //如果ID是期望的ID才继续执行,不然continue 本次循环, 等待下一个响应包.   
           if(CAN.getCanId() != CanECU_ID)
           {
              Serial.print("ID not match: ");
              Serial.println(CAN.getCanId(),HEX);
              break;
              //continue;
           }//这是想要的ID 的回应
           else
           {
            
              Serial.print("U<--C: ID:");
              Serial.println(CAN.getCanId(),HEX);
              Serial.print(" Len:");
              Serial.println(len);
              //记录Response结果
              for(int i = 0; i<8; i++)    // print the data
              {
                  Serial.print("0x");
                  Serial.print(buf[i], HEX);
                  Serial.print("\t");
              }
              Serial.println(">---END---");
    
              
              int SIDCode = buf[1];
              int IdentCode = buf[2]*0xFF+buf[3];
              int Frameflag = UDS_PayloadSize >> 4;
              switch( Frameflag | 0x0F)
              {
                
                // Signal Frame
                case 0x00: 
                {
                  if(SIDCode == 0x7f || SIDCode != 0x62 )
                  {
                    //分包的情况, 不能跳出循环, 需要等待下一个包.
                     Serial.println("Skip this");
                     continue; 
                  }
                  else
                  {
                     UDS_PayloadSize = (int)buf[0];
                      // 正常的Response, 结果都在这条Response之中.
                     char Data[UDS_PayloadSize-3];
  
                     for(int i=0; i<UDS_PayloadSize-3; i++)
                     {
                       Data[i]=buf[i+4];
                     }
                    //数据copy完了, 应该返回了,退出while循环
                     return Data;
                     break;
                  }
                }
                
                // 1st Frame of the Flow
                //如果是首帧的话,
                case 0x01:
                {
                  //整个数据段的大小,比如 31. 0x1F 减去 SID+IdentID (3bytes).
                  UDS_PayloadSize = (int)buf[1]-3;
                 
                  //开辟一个存储整个数据段大小的空间
                  char Data[UDS_PayloadSize];
                  memset(Data, 0, UDS_PayloadSize); 
                  //计算需要剩余多少次0x2X发送后面的byte, 第一个0x10中已经带了6byte数据了, 所以 是 buf[1]-6
                  loops  = (UDS_PayloadSize - 3)/ 7 + (((UDS_PayloadSize - 3) % 7 != 0)? 1 : 0 );
                  PendingByte=loops*7-((int)buf[1]-6); //3 
                  for(int i=0; i<3; i++)
                  {
                    Data[i]=buf[i+5]; // Data[0,1,2]
                    SavedBytes++;
                  }
                  //0x10的情况需要一个流控.
                  CAN.sendMsgBuf(CanTest_ID, 0, 8, Msg_0x773_GPSFlowCtrl);
                  break;
                  
                }

                // the following Data 0x2X
                /*  1F, 31 个, 第一个Frame 有了6个,还剩 25byte, 一个frame 有 7个byte, 所以需要4 个loops (4*7=28个byte), 最后28-25=3个byte 填充 AA
                 *  SENT:     0x773   03 22 [24 30] 55 55 55 55
                    REV:      0x7DD   03 7F 22 78 AA AA AA AA
                    (1)REV:   0x7DD   10 1F 62 24 30  <ASCII jingdu> :  FRIST Frame  10,  Size (All)  1F, 后面 Data
                    SENT :    0x773   30 00 01 55 55 55 55
                    (2)REV:   0x7DD   21 <ASCII jingdu>     :  21 帧序列号, 22 23 ….   后面Data
                    (N)REV:   0x7DD   22 <ASCII jingdu>
                    (Last)REV:    0x7DD   NN  <ASCII jingdu> AA AA AA AA
                 */
                case 0x02:
                {
                  
                  if( NumberofLoop < loops && loops != 1 )
                  {
                    for(int i=0; i<7; i++)
                    {
                      Data2X[3+i+(NumberofLoop*7)]=buf[i+1]; //Data[3, 4, 5,6,7,8,9]
                      SavedBytes++;
                    }
                    NumberofLoop++; //Data[10, 11, 12, 13,14,15,16]
                  } else {
                    for(int i=0; i<(7-PendingByte); i++)
                    {
                      Data2X[3+i+(NumberofLoop*7)]=buf[i+1];
                      SavedBytes++;
                    }     
                  }
                  if(UDS_PayloadSize == SavedBytes)
                  {
                    PendingByte=0;
                    NumberofLoop=0;   
                    SavedBytes=0;
                    UDS_PayloadSize=0;
                    return Data2X;
                  }
                  break;
                }

                default: break;
              
                }
              
             
    
            }
      }

}

void loop()
{
    int R_Flag=0;
    delay(1000);
    String TempStr;
    char R_SIDCode;
    int rpm=0;
    char JsonStr[1024];
    //TempStr="{\"Trip_ID\" : \"Yndd7Ddndsy\",\"Package_ID\" : 222,\"Time_Stamp\" : \"2012-04-09 12:13:44.34\",\"MileAge\" : 2234 ,\"FuelCost\": 2234,\"Engine_RPM\": 2234,\"Engine_Power\": 2234,\"Engine_Temp\": 98,\"Engine_Throttle\": 25,\"Battery_Volt\": 15.5,\"GPS_Longitude\": \"ABCDEF\",\"GPS_Latitude\": \"GHIJKLMN\",\"GPS_Altitude\": 8848,\"GPS_Course\": 354,\"Car_Speed\": 140,\"Car_Mode\": 3,\"Car_Temp\": 24.5,\"Car_BreakPostion\": 70,\"GearBox_Postion\": 23}";
    //Serial1.println(TempStr);
    Serial.println(TempStr);
     //【 03-|22 f4 0c 55 55 55 55 |]
     //char RPM_CODE[8]={0x03, 0x22, 0xF4,0x0C, 0x55,0x55,0x55,0x55};


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

    serializeJson(NetStr, Serial1);
  
    delay(50);                       // send data per 100ms
}



// END FILE
