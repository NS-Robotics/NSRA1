#include <SPI.h>
#include <mcp_can.h>

MCP_CAN CAN(10);

unsigned char len = 0;
unsigned char buf[8];
unsigned int canID;

int open = 900;

void setup()
{
    Serial.begin(115200);
    Serial.println("Start");
    if(CAN_OK == CAN.begin(CAN_100KBPS))
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
    pinMode(5, OUTPUT);
    pinMode(3, OUTPUT);
}



void loop()
{
    if(CAN_MSGAVAIL == CAN.checkReceive())
    {
        CAN.readMsgBuf(&len, buf);
        canID = CAN.getCanId();
        //Serial.println(canID);
        if(canID == 242) {
            //Serial.println(buf[0]);
            int i = buf[0];
            if(i == 0){
                //Serial.println("close");
                digitalWrite(5, HIGH);
                for(int f = 0; f < open; f++){
                    digitalWrite(3, HIGH);
                    delayMicroseconds(100);
                    digitalWrite(3, LOW);
                    delayMicroseconds(100);
                }
            } else if(i == 1){
                //Serial.println("open");
                digitalWrite(5, LOW);
                for(int f = 0; f < open; f++){
                    digitalWrite(3, HIGH);
                    delayMicroseconds(100);
                    digitalWrite(3, LOW);
                    delayMicroseconds(100);
                }
            }
        }
       
    }
}