#define   VERSION "1.0"

// SIM card PIN number, if any
// #define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]  = "";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT broker details - fill in for your broker
#define MQTT_BROKER   ""
#define MQTT_PORT     1883
#define MQTT_USER     ""
#define MQTT_PASSWORD ""

## Payload ID - set this and create a matching payload doc if you are passing telemetry from the broker to HABHUB
#define PAYLOAD_ID    "CHANGEME"

/// ---------------

#define TINY_GSM_MODEM_SIM7000

// Set serial for debug conso le (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
#define SerialAT Serial1

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon


#include <TinyGsmClient.h>
#include <Ticker.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

Ticker tick;

TinyGsmClient client(modem);
PubSubClient mqtt(client);

unsigned int Counter;
    
#define PIN_TX                  27
#define PIN_RX                  26
#define UART_BAUD               115200
#define PWR_PIN                 4
#define LED_PIN                 12
#define POWER_PIN               25
#define IND_PIN                 36

// OLED
#define OLED
#define OLED_SDA                21
#define OLED_SCL                22
#define OLED_ADDRESS            0x3C
#define SCREEN_WIDTH            128
#define SCREEN_HEIGHT           64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

struct TGPS
{
  float lat      = 0;
  float lon      = 0;
  float gpsspeed = 0;
  float alt      = 0;
  int   vsat     = 0;
  int   usat     = 0;
  float accuracy = 0;
  int   year     = 0;
  int   month    = 0;
  int   day      = 0;
  int   hour     = 0;
  int   minute   = 0;
  int   second   = 0;
};

void setup()
{
   char Temp[32];
   
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  SerialMon.println();
  SerialMon.print("TTGO SIM7000G MQTT Tracker V");
  SerialMon.println(VERSION);
  SerialMon.println("===============================");
  SerialMon.println();

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, false, false))
  {
    SerialMon.println("SSD1306 allocation failed");
  }  
  
  display.clearDisplay();  
  display.setTextColor(WHITE, 0);
  display.setTextSize(1);

  sprintf(Temp, "HAB MQTT Tracker V%s", VERSION);
  DisplayLine(0, Temp);
 
  sprintf(Temp, "APN: %s, ID: %s", apn, PAYLOAD_ID);
  DisplayLine(1, Temp);
 
  // Open modem serial port
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  
  // Onboard LED light, it can be used freely
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // POWER_PIN : This pin controls the power supply of the Modem
  SerialMon.println("Switch Modem ON");
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);

//  while (!StartModem())
//  {
//    SerialMon.println("Failed to restart modem, delaying 10s ...");
//    delay(10000);
//  }
  StartModem();

  SerialMon.println("Initialised Modem OK");

  // Initialise modem and enable GPS within the modem
  SetupModemAndGPS();
}

int StartModem(void)
{
  // PWR_PIN ： This Pin is the PWR-KEY of the Modem
  // The time of active low level impulse of PWRKEY pin to power on module , type 500 ms
  SerialMon.println("Pulse PWR Key");
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(500);    
  digitalWrite(PWR_PIN, LOW);

  // IND_PIN: It is connected to the Modem status Pin, 
  // through which you can know whether the module starts normally.
  pinMode(IND_PIN, INPUT);

  SerialMon.println("LED to flash 1Hz when modem started");
  attachInterrupt(IND_PIN, []() {
      detachInterrupt(IND_PIN);
      // If Modem starts normally, then set the onboard LED to flash once every 1 second
      tick.attach_ms(1000, []() {
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      });
  }, CHANGE);

  DisplayLine(6, "Wait for modem");
  delay(3000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  DisplayLine(6, "Restarting Modem");
  return modem.restart();
}

void SetupModemAndGPS(void)
{
  String name = modem.getModemName();
  delay(500);
  Serial.println("Modem Name: " + name);
  
  String modemInfo = modem.getModemInfo();
  delay(500);
  Serial.println("Modem Info: " + modemInfo);    
  
  // Set SIM7000G GPIO4 HIGH ,Open GPS power
  // CMD:AT+SGPIO=0,4,1,1
  DBG("Power on GPS");
  modem.sendAT("+SGPIO=0,4,1,1");
  
  DBG("Enable GPS");
  modem.enableGPS();    

  #ifdef GSM_PIN
  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
      modem.simUnlock(GSM_PIN);
  }
  #endif

/*
    DBG("Set GSM Mode");
    //Set to GSM mode, please refer to manual 5.11 AT+CNMP Preferred mode selection for more parameters
    String result;
    do {
        result = modem.setNetworkMode(13);
        DBG(result);
        delay(500);
    // } while (result != "OK");
    } while (result != "1");

//    1 CAT-M
//    2 NB-Iot
//    3 CAT-M and NB-IoT
    DBG("Set Preferred Mode");
    do {
        result = modem.setPreferredMode(2);
        DBG(result);
        delay(500);
    // } while (result != "OK");
    } while (result != "1");
*/
}

void loop()
{
  struct TGPS GPS;
  
  EnableGPS();
  
  if (GetPosition(&GPS))
  {
    DisplayLine(6, "Got GPS Lock");
      
    // Check connection and reconnect if needed
    if (ConnectToNetwork())
    {
      char Sentence[100];
      
      ShowNetworkDetails();
  
      if (GetPosition(&GPS))
      {
        int i;
        char Temp[32];
        
        DisplayLine(6, "Got GPS Position");
  
        BuildSentence(GPS, Sentence);
  
        SendMQTTMessage(Sentence);

        for (i=1; i<=10; i++)
        {
           sprintf(Temp, "Delay %d of 10 ...", i);
           DisplayLine(6, Temp);
          delay(1000);
        }
      }
    }
  }
  else
  {
    DisplayLine(6, "No GPS Position yet");

    delay(5000);
  }
}


int ConnectToNetwork(void)
{
  DisplayLine(6, "Waiting for Network");
  if (modem.waitForNetwork())
  {
    DisplayLine(6, "Network present");
    if (modem.isNetworkConnected())
    {
      DBG("GSM Network connected");
      DisplayLine(6, "Connecting to APN");
      if (modem.gprsConnect(apn, gprsUser, gprsPass))
      {
        DisplayLine(6, "Connected to APN");
        return 1;
      }
      else
      {
        DBG("Not connected to APN");

        ShowNetworkDetails();
      }
    }
    else
    {
      DisplayLine(6, "Not connected to GSM");
    }
  }

  return 0;
}
  
void ShowNetworkDetails(void)
{
  char Temp[32];
  bool res = modem.isGprsConnected();
  DBG("GPRS status:", res ? "connected" : "not connected");

  String ccid = modem.getSimCCID();
  DBG("CCID:", ccid);

  String imei = modem.getIMEI();
  DBG("IMEI:", imei);

  String cop = modem.getOperator();
  DBG("Operator:", cop);

  IPAddress local = modem.localIP();
  DBG("Local IP:", local);

  int csq = modem.getSignalQuality();
  DBG("Signal quality:", csq);

  sprintf(Temp, "%d.%d.%d.%d sig %d", local[0], local[1], local[2], local[3], csq); 
  DisplayLine(4, Temp);
}

void EnableGPS(void)
{
  modem.sendAT("+SGPIO=0,4,1,1");

  modem.enableGPS();
}
  

int GetPosition(struct TGPS *GPS)
{
  if (modem.getGPS(&GPS->lat, &GPS->lon, &GPS->gpsspeed, &GPS->alt, &GPS->vsat, &GPS->usat, &GPS->accuracy, &GPS->year, &GPS->month, &GPS->day, &GPS->hour, &GPS->minute, &GPS->second))
  {
    char Temp[32];
    
    modem.getGPSTime(&GPS->year, &GPS->month, &GPS->day, &GPS->hour, &GPS->minute, &GPS->second);

    sprintf(Temp, "%02d:%02d:%02d %.0fm %d sats", GPS->hour, GPS->minute, GPS->second, GPS->alt, GPS->usat);
    DisplayLine(2, Temp);

    sprintf(Temp, "%.5f, %.5f", GPS->lat, GPS->lon);
    DisplayLine(3, Temp);

    return 1;
  }

  return 0;
}

char Hex(int Character)
{
  char HexTable[] = "0123456789ABCDEF";
  
  return HexTable[Character];
}

void BuildSentence(struct TGPS GPS, char *Sentence)
{
  static int Counter=0;
  unsigned int CRC, xPolynomial, i, j, Count;

  sprintf(Sentence, "$$%s,%u,%02d:%02d:%02d,%.5f,%.5f,%.0f,%u", PAYLOAD_ID, ++Counter, GPS.hour, GPS.minute, GPS.second, GPS.lat, GPS.lon, GPS.alt, GPS.usat);

  Count = strlen(Sentence);

  CRC = 0xffff;           // Seed
  xPolynomial = 0x1021;
 
   for (i = 2; i < Count; i++)
   {   // For speed, repeat calculation instead of looping for each bit
      CRC ^= (((unsigned int)Sentence[i]) << 8);
      for (j=0; j<8; j++)
      {
          if (CRC & 0x8000)
              CRC = (CRC << 1) ^ 0x1021;
          else
              CRC <<= 1;
      }
   }

  Sentence[Count++] = '*';
  Sentence[Count++] = Hex((CRC >> 12) & 15);
  Sentence[Count++] = Hex((CRC >> 8) & 15);
  Sentence[Count++] = Hex((CRC >> 4) & 15);
  Sentence[Count++] = Hex(CRC & 15);
  Sentence[Count++] = '\n';  
  Sentence[Count++] = '\0';
      
  DBG(Sentence);
}
  
void SendMQTTMessage(char *Sentence)
{
  static int Count=0;
  char Temp[32];
  
  DBG("Set broker name");
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  DBG("Connect to MQTT broker ...");
  if (mqtt.connect("MQTTTracker", MQTT_USER, MQTT_PASSWORD))
  {
    DBG("Connected to MQTT Broker");  

    DBG("Publish to MQTT broker ...");
    mqtt.publish("telemetry/ttgo", Sentence);

    sprintf(Temp, "MQTT Count: %d", ++Count);
    DisplayLine(5, Temp);
    
    delay(2000);
  
    DBG("Disconnect from MQTT broker ...");
  }
  else
  {
    DBG("Could not connect to MQTT broker");
  }
}

void DisplayLine(int Line, char *Message)
{
  #define ROW_HEIGHT 9
  int y;
  
  DBG(Message);

  y = Line * ROW_HEIGHT;
  
  display.fillRect(0, y, 128, ROW_HEIGHT, 0);

  display.setCursor(0,y);
  display.print(Message);
  display.display();
}
