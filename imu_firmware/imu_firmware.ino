#include <Wire.h>

#define ENABLE_WIFI

#include "ADXL345.h"
#include "ITG3200.h"
#include "DCM.h"

#ifdef ENABLE_WIFI

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <WiFiAP.h>
#include "esp_system.h"
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include "AsyncUDP.h"


#define WIFI_SSID       "ENTER YOUR NETWORK NAME HERE"
#define WIFI_PASSWORD   "ENTER YOUR NETWORK PASSWORD HERE"
#define SERVER_IP       "ENTER YOUR SERVER IP HERE"
#define UDP_SERVER_PORT 3333

AsyncUDP udp;

#endif

ADXL345* acceletometer;
ITG3200* gyro;
DCM* dcm;

long timeCounter = 0;
long lastTimeCounter = 0;

#ifdef ENABLE_WIFI
void initWiFi() 
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP().toString());
    IPAddress serverIP;
    serverIP.fromString(SERVER_IP);
    if (udp.connect(serverIP, UDP_SERVER_PORT)) 
    {
        Serial.println("UDP connected");
        udp.onPacket([](AsyncUDPPacket packet) 
        {
            // Do somthing with the data
        });
    }
}
#endif


void setup() 
{
	Serial.begin(115200);
	Wire.begin();

    acceletometer = new ADXL345();
    gyro = new ITG3200();
    dcm = new DCM();
	
    timeCounter = millis();
    delay(20);

#ifdef ENABLE_WIFI
    initWiFi();
#endif
}

void loop() 
{
    if ( (millis() - timeCounter) >= 20)  // Main loop runs at 50Hz
    {  
        lastTimeCounter = timeCounter;
        timeCounter = millis();
        float gdt = (timeCounter - lastTimeCounter) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
        if (gdt > 1) 
        {
            gdt = 0;  //keeps dt from blowing up, goes to zero to keep gyros from departing
        }
        dcm->updateGDT(gdt);

        int16_t accData[3];
        int16_t gyroData[4];  

        acceletometer->readData(accData);
        gyro->readData(gyroData);

        dcm->calculate(accData, gyroData);

        // Send data
        char str[512];
        dcm->getDataString(str); 
        Serial.print(str);
#ifdef ENABLE_WIFI        
        udp.print(str);
#endif
    }
}
