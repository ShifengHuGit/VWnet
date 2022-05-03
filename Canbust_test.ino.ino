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
#define MSGINFO "a"

unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];
int J_ID, J_MA, J_FC, J_ERPM, J_EPW,J_ETP, J_ETH, J_BV, J_GA, J_GC, J_CS, J_CM, J_CTMP, J_BP, J_GP;
StaticJsonDocument<100> NetStr;



#define CANMSGID_ECU_ENG                  0x07E8
#define CANMSGID_TESTER_ENG               0x07E0
#define CANMSGID_ECU_GEARBOX              0x07E9
#define CANMSGID_TESTER_GEARBOX           0x07E1
#define CANMSGID_ECU_SESSION              0x077A
#define CANMSGID_TESTER_SESSION           0x0710
//17F00010


MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

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
  unsigned char SessionFrameTX[8] = {0x02, 0x10, 0x03, 0x55, 0x55,0x55,0x55,0x55};
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
    CAN.init_Filt(0, 0, 0x07E8);                          // there are 6 filter in mcp2515
    CAN.init_Filt(1, 0, 0x07E9);                          // there are 6 filter in mcp2515
    
    CAN.init_Mask(1, 0, 0x0700);
    CAN.init_Filt(2, 0, 0x077A);                          // there are 6 filter in mcp2515
    CAN.init_Filt(3, 0, 0x0710);                          // there are 6 filter in mcp2515
    CAN.init_Filt(4, 0, 0x0720);                          // there are 6 filter in mcp2515
    CAN.init_Filt(5, 0, 0x0730);                          // there are 6 filter in mcp2515


    // Send Initialize Session CAN request
    //  {0x02, 0x10, 0x03, 0x55, 0x55,0x55,0x55,0x55};
    //          ^      ^
    //          |      |
    //      Session   Extended 
    // 
      // Send this to CAN Bus
      // 发送 进入EXT会话的请求
      CAN.sendMsgBuf(CANMSGID_TESTER_SESSION, 0, 8, SessionFrameTX);
  
      // Logging the Request
      Serial.print("U-->C:");
      for(int i = 0; i<8; i++)    // print the data
          {
              Serial.print("0x");
              Serial.print(SessionFrameTX[i], HEX);
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

       if(CAN.getCanId() != 0x77A)
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
             CAN.sendMsgBuf(CANMSGID_TESTER_SESSION, 0, 8, SessionFrameTX);
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

    J_ID=0;
    J_MA=2084; J_FC=40; J_ERPM=0; J_EPW=234; J_ETP=23; J_ETH=50; J_BV=11; J_GA=130; J_GC=324; J_CS=90; J_CM=1; J_CTMP=23; J_BP=13; J_GP=3;
    NetStr["Trip_ID"]="";
    NetStr["Package_ID"]="";
    NetStr["Time_Stamp"]="";
    NetStr["MileAge"];
    NetStr["FuelCost"];
    NetStr["Engine_RPM"];
    NetStr["Engine_Power"];
    NetStr["Engine_Temp"];
    NetStr["Engine_Throttle"];
    NetStr["Battery_Volt"];
    JsonArray GPSdata = NetStr.createNestedArray("GPSdata");
    GPSdata.add(J_GA);
    GPSdata.add(J_GA);
    GPSdata.add(J_GA);
    GPSdata.add(J_GA);
    
    NetStr["Car_Speed"];
    NetStr["Car_Mode"];
    NetStr["Car_Temp"];
    NetStr["Car_BreakPostion"];
    NetStr["GearBox_Postion"];

    serializeJson(NetStr, Serial1);

    
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



void loop()
{

    J_ID=J_ID+1;
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
     char RPM_CODE[8]={0x03, 0x22, 0xF4,0x0C, 0x55,0x55,0x55,0x55};


    CAN.sendMsgBuf(CANMSGID_TESTER_ENG, 0, 8, RPM_CODE);

    
     while(1)
      {
    
          while(CAN_MSGAVAIL != CAN.checkReceive());
        /*  {
            Serial.println("No Data!");
            delay(1500);
          }*/
          
          CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    
           if(CAN.getCanId() != CANMSGID_ECU_ENG)
           {
              Serial.print("ID not match: ");
              Serial.println(CAN.getCanId(),HEX);
              break;
              //continue;
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


              
              if(R_SIDCode == 0x7f || R_SIDCode != 0x62 )
              {
                 Serial.println("I Can not reply you");
                 
                 //CAN.sendMsgBuf(CANMSGID_TESTER_ENG, 0, 8, RPM_CODE);
                 continue;
              }
              else
              {
                 Serial.println("Responsed!--->");
                 J_ERPM = Hander_ENG_RPM(buf[4],buf[5]);
                 break;
              }
    
            }
      }

      
 
 sprintf(JsonStr,  
"{\
\"Trip_ID\" : \"DummyStr\",\
\"Package_ID\" : %d,\
\"Time_Stamp\" : \"2012-04-09 12:13:44.34\",\
\"MileAge\" : %d ,\
\"FuelCost\": %d,\
\"Engine_RPM\": %d,\
\"Engine_Power\": %d,\
\"Engine_Temp\": %d,\
\"Engine_Throttle\": %d,\
\"Battery_Volt\": %d,\
\"GPS_Longitude\": \"ABCDEF\",\
\"GPS_Latitude\": \"GHIJKLMN\",\
\"GPS_Altitude\": %d,\
\"GPS_Course\": %d,\
\"Car_Speed\": %d,\
\"Car_Mode\": %d,\
\"Car_Temp\": %d,\
\"Car_BreakPostion\": %d,\
\"GearBox_Postion\": %d\
}",    J_ID, J_MA, J_FC, J_ERPM, J_EPW,J_ETP, J_ETH, J_BV, J_GA, J_GC, J_CS, J_CM, J_CTMP, J_BP, J_GP);
    
    Serial.println(JsonStr);
    Serial1.println(JsonStr);
    delay(50);                       // send data per 100ms
}



// END FILE
