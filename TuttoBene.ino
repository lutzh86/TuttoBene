
#include "CONFIG.H"
#include "BMSModuleManager.h"
#include <Arduino.h>
#include "SerialConsole.h"
#include "Logger.h"

#include "driver/gpio.h"
#include "driver/twai.h"

#include "BMSUtil.h"
#include "WiFi.h"
#include <WebServer.h>

#include <WiFiUdp.h>
#include <ArduinoOTA.h>


const char* ssid = "wifi";
const char* password =  "password";

String lastcans[100];
int lastcancounter = 0;

twai_message_t msg, inmsg;

IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
WebServer server(80);



int SOC = 100; //State of Charge

BMSModuleManager bms;
SerialConsole console;
EEPROMSettings settings;


int looptime = millis();

char msgString[128]; 

uint16_t SOH = 100; // SOH place holder


int incomingByte = 0;

int x = 0;

int cellspresent = 0;

float ampsecond = 0;
float currentact;




//Balance testing
int balinit = 0;
int balon = 0;

int balcycle = 0;

byte alarm_ve[4], warning_ve[4] = {0, 0, 0, 0}; // for victron can

byte stringToByte(char * src){
    return byte(atoi(src));
}

void loadSettings()
{

  settings.version = EEPROM_VERSION;
  settings.canSpeed = 500000;
  settings.batteryID = 0x01; //in the future should be 0xFF to force it to ask for an address
  settings.OverVSetpoint = 4.2f;
  settings.UnderVSetpoint = 3.4f;
  settings.ChargeVsetpoint = 4.1f;
  settings.DischVsetpoint = 3.4f;
  settings.CellGap = 0.2f; //max delta between high and low cell
  settings.OverTSetpoint = 50.0f;
  settings.UnderTSetpoint = -10.0f;
  settings.DisTSetpoint = 40.0f;
  settings.balanceVoltage = 3.7f;
  settings.balanceHyst = 0.005f;
  settings.logLevel = 2;
  settings.CAP = 234; //battery size in Ah
  settings.Pstrings = 2; // strings in parallel used to divide voltage of pack
  settings.Scells = 16;//Cells in series
 

  settings.socvolt[0] = 3500; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[1] = 10; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[2] = 4000; //Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[3] = 90; //Voltage and SOC curve for voltage based SOC calc
 
}



uint32_t lastUpdate;

void printFrame(twai_message_t message)
{
  Serial.print(message.identifier, HEX);
  if (message.extd) Serial.print(" X ");
  else Serial.print(" S ");   
  Serial.print(message.data_length_code, DEC);
  for (int i = 0; i < message.data_length_code; i++) {
    Serial.print(message.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}


void setup()
{
  WiFi.begin(ssid, password);
  server.on("/", handle_OnConnect);
  server.begin();

      //Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_40, GPIO_NUM_39, TWAI_MODE_NORMAL); //nNROMAL
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    t_config =    {.brp = 8, .tseg_1 = 13, .tseg_2 = 6, .sjw = 3, .triple_sampling = false};
    
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    

    //Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    //Start TWAI driver
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }

/*
    uint32_t alerts_to_enable = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_AND_LOG;
    
    //TWAI_ALERT_ALL | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_RX_DATA | TWAI_ALERT_AND_LOG | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF;


        
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    printf("Alerts reconfigured\n");
    } 
  */
  


  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  

  Serial.begin(115200);
  Serial.println("Starting up!");
  
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  

  
    loadSettings();
 

  Logger::setLoglevel(Logger::Off); //Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4

  lastUpdate = 0;
  bms.setPstrings(settings.Pstrings);
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt, settings.DeltaVolt);
  bms.setBalanceHyst(settings.balanceHyst);


}


void loop()
{

 
  server.handleClient();
  ArduinoOTA.handle();
  

  //int32_t alerts_triggered;
  
  //twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(10));
   
   canread();
  
   


  if (millis() - looptime > 3000)
  {
    looptime = millis();
    bms.getAllVoltTemp();
    
      if (looptime % 354 == 0) { //deacticate balancing all 10 reps
      bms.balanceCells(1,0);//1 is debug
      }
      else {      
      bms.balanceCells(1,1);}

 
      //undervoltage fault (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() < settings.UnderVSetpoint)
     
          
      
      printbmsstat();
      bms.printPackDetails(3);
    
    


   
    //VEcan();

    //sendcommand();

    //cellspresent = bms.seriescells();
    //bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt, settings.DeltaVolt);
   

 
 
  

  }

  



}


  
  



void handle_OnConnect() {
  server.send(200, "text/html", bms.htmlPackDetails(3)); 
}


void alarmupdate()
{
  alarm_ve[0] = 0x00;
  if (settings.OverVSetpoint < bms.getHighCellVolt())
  {
    alarm_ve[0] = 0x04;
  }
  if (bms.getLowCellVolt() < settings.UnderVSetpoint)
  {
    alarm_ve[0] |= 0x10;
  }
  if (bms.getHighTemperature() > settings.OverTSetpoint)
  {
    alarm_ve[0] |= 0x40;
  }
  alarm_ve[1] = 0;
  if (bms.getLowTemperature() < settings.UnderTSetpoint)
  {
    alarm_ve[1] = 0x01;
  }
  alarm_ve[3] = 0;
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
  {
    alarm_ve[3] = 0x01;
  }

 
}



void printbmsstat()
{

  Serial.print("BMS Status : ");


    if (bms.getLowCellVolt() < settings.UnderVSetpoint)
    {
      Serial.print(": UnderVoltage ");
    }

    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      Serial.print(": OverVoltage ");
    }
    if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
    {
      Serial.print(": Cell Imbalance ");
    }
    if (bms.getAvgTemperature() > settings.OverTSetpoint)
    {
      Serial.print(": Over Temp ");
    }
    if (bms.getAvgTemperature() < settings.UnderTSetpoint)
    {
      Serial.print(": Under Temp ");
    }
     
 
  Serial.println();

 
 
}





void updateSOC()
{

    SOC = map(uint16_t(bms.getLowCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);

 // ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
       
   
    ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778 ;
    SOC = ((ampsecond * 0.27777777777778) / (settings.CAP * settings.Pstrings * 1000)) * 100;
 
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778 ; //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
   
   

}

  


void VEcan() //communication with Victron system over CAN
{
 /* if (settings.chargertype == Pylon)
  {
    msg.identifier  = 0x359;
    msg.data_length_code = 8;
    msg.data[0] = 0x00; //protection to be translated later date
    msg.data[1] = 0x00; //protection to be translated later date
    msg.data[2] = 0x00; //protection to be translated later date
    msg.data[3] = 0x00; //protection to be translated later date
    msg.data[4] = 0x01; //number of modules fixed for now
    msg.data[5] = 0x50;
    msg.data[6] = 0x4E;
    msg.data[7] = 0x00;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 

    delay(2);

    msg.identifier  = 0x351;
    msg.data_length_code = 8;
    if (storagemode == 0)
    {
      msg.data[0] = lowByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 10));
      msg.data[1] = highByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 10));
    }
    else
    {
      msg.data[0] = lowByte(uint16_t((settings.StoreVsetpoint * settings.Scells ) * 10));
      msg.data[1] = highByte(uint16_t((settings.StoreVsetpoint * settings.Scells ) * 10));
    }
    msg.data[2] = lowByte(chargecurrent);
    msg.data[3] = highByte(chargecurrent);
    msg.data[4] = lowByte(discurrent );
    msg.data[5] = highByte(discurrent);
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 

    delay(2);

    msg.identifier  = 0x355;
    msg.data_length_code = 8;
   msg.data[0] = lowByte(SOC);
    msg.data[1] = highByte(SOC);
    msg.data[2] = lowByte(SOH) ;//static for now
    msg.data[3] = highByte(SOH); //static for now 
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 

    delay(2);

    msg.identifier  = 0x356;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
    msg.data[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
    msg.data[2] = lowByte(long(currentact / 100));
    msg.data[3] = highByte(long(currentact / 100));
    msg.data[4] = lowByte(int16_t(bms.getAvgTemperature() * 10));
    msg.data[5] = highByte(int16_t(bms.getAvgTemperature() * 10));
    msg.data[6] = 0;
    msg.data[7] = 0;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 


    delay(2);

    msg.identifier  = 0x35C;
    msg.data_length_code = 2;
    msg.data[0] = 0xC0; //fixed charge and discharge enable for verifcation
    msg.data[1] = 0x00;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 

    delay(2);

    msg.identifier  = 0x35E;
    msg.data_length_code = 2;
    msg.data[0] = "T"; //No idea how the naming works
    msg.data[1] = "P"; //No idea how the naming works
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 
  } */
  if (1) // else ..
  {
    msg.identifier  = 0x351;
    msg.data_length_code = 8;

      msg.data[0] = lowByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 10));
      msg.data[1] = highByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 10));
    
  
    //msg.data[2] = lowByte(chargecurrent);
    //msg.data[3] = highByte(chargecurrent);
    //msg.data[4] = lowByte(discurrent );
    //msg.data[5] = highByte(discurrent);
    msg.data[6] = lowByte(uint16_t((settings.DischVsetpoint * settings.Scells) * 10));
    msg.data[7] = highByte(uint16_t((settings.DischVsetpoint * settings.Scells) * 10));
    
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 
    
    msg.identifier  = 0x355;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(SOC);
    msg.data[1] = highByte(SOC);
    msg.data[2] = lowByte(SOH);
    msg.data[3] = highByte(SOH);
    msg.data[4] = lowByte(SOC * 10);
    msg.data[5] = highByte(SOC * 10); 
    msg.data[6] = 0;
    msg.data[7] = 0;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 

    msg.identifier  = 0x356;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
    msg.data[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
    msg.data[2] = lowByte(long(currentact / 100));
    msg.data[3] = highByte(long(currentact / 100));
    msg.data[4] = lowByte(int16_t(bms.getAvgTemperature() * 10));
    msg.data[5] = highByte(int16_t(bms.getAvgTemperature() * 10));
    msg.data[6] = 0;
    msg.data[7] = 0;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 

    delay(2);
    msg.identifier  = 0x35A;
    msg.data_length_code = 8;
    msg.data[0] = alarm_ve[0];//High temp  Low Voltage | High Voltage
    msg.data[1] = alarm_ve[1]; // High Discharge Current | Low Temperature
    msg.data[2] = alarm_ve[2]; //Internal Failure | High Charge current
    msg.data[3] = alarm_ve[3];// Cell Imbalance
    msg.data[4] = warning_ve[0];//High temp  Low Voltage | High Voltage
    msg.data[5] = warning_ve[1];// High Discharge Current | Low Temperature
    msg.data[6] = warning_ve[2];//Internal Failure | High Charge current
    msg.data[7] = warning_ve[3];// Cell Imbalance
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 

    msg.identifier  = 0x35E;
    msg.data_length_code = 8;
    msg.data[0] = 'S';
    msg.data[1] = 'I';
    msg.data[2] = 'M';
    msg.data[3] = 'P';
    msg.data[4] = '3';
    msg.data[5] = '2';
    msg.data[6] = 'B';
    msg.data[7] = 'M';
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 

    delay(2);
    msg.identifier  = 0x370;
    msg.data_length_code = 8;
    msg.data[0] = 'S';
    msg.data[1] = 'H';
    msg.data[2] = 'P';
    msg.data[3] = 'I';
    msg.data[4] = 'G';
    msg.data[5] = 'M';
    msg.data[6] = 'B';
    msg.data[7] = 'H';
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 

    delay(2);
    msg.identifier  = 0x373;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(uint16_t(bms.getLowCellVolt() * 1000));
    msg.data[1] = highByte(uint16_t(bms.getLowCellVolt() * 1000));
    msg.data[2] = lowByte(uint16_t(bms.getHighCellVolt() * 1000));
    msg.data[3] = highByte(uint16_t(bms.getHighCellVolt() * 1000));
    msg.data[4] = lowByte(uint16_t(bms.getLowTemperature() + 273.15));
    msg.data[5] = highByte(uint16_t(bms.getLowTemperature() + 273.15));
    msg.data[6] = lowByte(uint16_t(bms.getHighTemperature() + 273.15));
    msg.data[7] = highByte(uint16_t(bms.getHighTemperature() + 273.15));
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 

    delay(2);
    msg.identifier  = 0x379; //Installed capacity
    msg.data_length_code = 2;
    msg.data[0] = lowByte(uint16_t(settings.Pstrings * settings.CAP));
    msg.data[1] = highByte(uint16_t(settings.Pstrings * settings.CAP));
    /*
        delay(2);
      msg.identifier  = 0x378; //Installed capacity
      msg.data_length_code = 2;
      //energy in 100wh/unit
      msg.data[0] =
      msg.data[1] =
      msg.data[2] =
      msg.data[3] =
      //energy out 100wh/unit
      msg.data[4] =
      msg.data[5] =
      msg.data[6] =
      msg.data[7] =
    */
    delay(2);
    msg.identifier  = 0x372;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(bms.getNumModules());
    msg.data[1] = highByte(bms.getNumModules());
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 
  }
}









void canread()
{

  



 if (twai_receive(&inmsg, pdMS_TO_TICKS(10000)) == ESP_OK) {

  if ((inmsg.identifier & 0x16A95470) == 0x16A95470) {

      lastcancounter++;
  if (lastcancounter > 100) lastcancounter = 0;

   lastcans[lastcancounter] = String(inmsg.identifier, HEX) + " ";
  if (inmsg.extd) lastcans[lastcancounter] += " X ";
  else lastcans[lastcancounter] += " S ";
  lastcans[lastcancounter] += String(inmsg.data_length_code);
  lastcans[lastcancounter] += ": ";
  for (int i = 0; i < inmsg.data_length_code; i++) {

    lastcans[lastcancounter] += " " + String(inmsg.data[i], HEX);    
  }

  }
    
  if (inmsg.identifier < 0x300)//do VW BMS magic if ids are ones identified to be modules
  {
      
      bms.decodecan(inmsg, 0); //do VW BMS if ids are ones identified to be modules
    
  }

  else if ((inmsg.identifier & 0x1FFFFFFF) < 0x1A555440 && (inmsg.identifier & 0x1FFFFFFF) > 0x1A555400)   // Determine if ID is Temperature CAN-ID
  {
 
      bms.decodetemp(inmsg, 0, 1);
      //16A95442 ?? .,..0x16A95471 can confirmation
  }

  else if ((inmsg.identifier & 0x1FFFFFFF) < 0x1A5555FF && (inmsg.identifier & 0x1FFFFFFF) > 0x1A5555EF)   // Determine if ID is Temperature CAN-ID FOR MEB
  {
    bms.decodetemp(inmsg, 0, 2);
   
  }

  else if ((inmsg.identifier & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } 
    // else {printFrame(inmsg);}
}}








  
  void sendcommand()
{
  
twai_message_t message;
int controlid = 0x0BA;

  message.data[0] = 0x00;
  message.data[1] = 0x00;
  message.data[2] = 0x00;
  message.data[3] = 0x00;
  message.data[4] = 0x00;
  message.data[5] = 0x00;
  message.data[6] = 0x00;
  message.data[7] = 0x00;
  message.identifier  = controlid;
  message.data_length_code = 8;
  message.extd = 0;
  message.ss = 1;

   if (twai_transmit(&message, pdMS_TO_TICKS(1000)) != ESP_OK) {Serial.println("CAN SEND ERROR\n");} 
  
  
  message.data[0] = 0x45;
  message.data[1] = 0x01;
  message.data[2] = 0x28;
  message.data[3] = 0x00;
  message.data[4] = 0x00;
  message.data[5] = 0x00;
  message.data[6] = 0x00;
  message.data[7] = 0x30;
  message.identifier  = controlid;
  message.data_length_code = 8;
  message.extd = 0;

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) != ESP_OK) {Serial.println("CAN SEND ERROR\n");} 
  


}
