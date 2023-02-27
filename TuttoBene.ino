
#include "BMSModuleManager.h"
#include "config.h"
#include <Arduino.h>

#include "driver/gpio.h"
#include "driver/twai.h"

#include "WiFi.h"
#include <WebServer.h>

#include <ArduinoOTA.h>
#include <WiFiUdp.h>

#define MAX_MODULE_ADDR 0x3E
#define EEPROM_VERSION 0x12
#define EEPROM_PAGE 0


const char *ssid = "HXT";
const char *password = "4uh7-wlxe-qh75";

String lastcans[10];
int lastcancounter = 0;


uint32_t alerts_triggered;

uint32_t can_processed;
twai_status_info_t twaistatus;
twai_message_t msg, inmsg;

IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);

int SOC,SOC_physical,SOH = 100; // State of Charge


BMSModuleManager bms;
EEPROMSettings settings;

int looptime = millis();
char msgString[128];
int incomingByte = 0;
int x = 0;



float ampsecond = 0;
float currentact;

byte alarm_ve[4], warning_ve[4] = {0, 0, 0, 0}; // for victron can

byte stringToByte(char *src) { return byte(atoi(src)); }

void loadSettings() {

  settings.canSpeed = 500000;
  settings.batteryID =    0x01; // in the future should be 0xFF to force it to ask for an address
  settings.OverVSetpoint = 4.15f;
  settings.UnderVSetpoint = 3.4f;
  settings.ChargeVsetpoint = 4.0f;
  settings.DischVsetpoint = 3.5f;
  settings.CellGap = 0.1f; // max delta between high and low cell
  settings.OverTSetpoint = 45.0f;
  settings.UnderTSetpoint = 5.0f;
  settings.DisTSetpoint = 40.0f;
  settings.balanceVoltage = 3.7f;
  settings.balanceHyst = 0.003f;
  settings.capacity = 234;    // battery size in Ah
  settings.parallel_strings = 2; // strings in parallel used to divide voltage of pack
  settings.series_cells = 16;  // Cells in series
  settings.nominal_cell_voltage = 3659; 
  settings.internal_resistance = 3; //in mOhm
  
  int curve[100] = {3360,3395,3433,3455,3471,3483,3491,3498,3505,3511,3518,3525,3532,3538,3544,3551,3557,3562,3568,3574,3579,3584,3589,3594,3599,3603,3608,3613,3617,3623,3628,
                   3634,3639,3644,3648,3652,3655,3658,3661,3664,3667,3670,3673,3676,3679,3683,3686,3689,3693,3697,3700,3704,3708,3713,3717,3722,3727,3732,3738,3745,3751,3759,3766,
                   3775,3784,3793,3803,3812,3822,3831,3840,3849,3858,3866,3875,3884,3893,3903,3912,3922,3932,3942,3952,3962,3972,3982,3991,4001,4010,4019,4028,4038,4048,4059,4072,
                   4085,4100,4116,4135,4155}; // discharge curve per cell, NMC
  
  for (int i = 0; i < 100; i++)  settings.ocv[i] = curve[i];

}

uint32_t lastUpdate;

void printFrame(twai_message_t message) {
  Serial.print(message.identifier, HEX);
  if (message.extd)
    Serial.print(" X ");
  else
    Serial.print(" S ");
  Serial.print(message.data_length_code, DEC);
  for (int i = 0; i < message.data_length_code; i++) {
    Serial.print(message.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  delay(2000);
  WiFi.begin(ssid, password);
  server.on("/", handle_OnConnect);
  server.begin();

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      GPIO_NUM_40, GPIO_NUM_39, TWAI_MODE_NO_ACK); // nNROMAL
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    printf("Driver installed\n");
  } else {
    printf("Failed to install driver\n");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    printf("Driver started\n");
  } else {
    printf("Failed to start driver\n");
    return;
  }

  uint32_t alerts_to_enable = TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS |
                              TWAI_ALERT_ALL | TWAI_ALERT_RX_DATA |
                              TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED |
                              TWAI_ALERT_BUS_OFF;

  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    printf("Alerts reconfigured\n");
  }

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();

  Serial.begin(115200);
  Serial.println("Starting up!");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  loadSettings();

  lastUpdate = 0;
  bms.setPstrings(settings.parallel_strings);
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt, settings.DeltaVolt);
  bms.setBalanceHyst(settings.balanceHyst);
  looptime = millis();
}

void loop() {

   twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(10));
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {canread(); }

  server.handleClient();
  ArduinoOTA.handle();

  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(10));
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {canread(); }

  if (millis() - looptime > 3000) {
    looptime = millis();
    twai_get_status_info(&twaistatus);
    bms.getAllVoltTemp();

    if (alerts_triggered & TWAI_ALERT_BUS_OFF) {
      twai_initiate_recovery();
    }

    bms.balanceCells(0, (looptime % 7 != 0)); // 1 is debug

    // printbmsstat();
    bms.printPackDetails(3);

    updateSOC();
    // alarmupdate();
    // VEcan();
    // sendcommand();
  }
}

void handle_OnConnect() {
  server.send(200, "text/html", bms.htmlPackDetails(3));
}

void alarmupdate() {
  alarm_ve[0] = 0x00;
  if (settings.OverVSetpoint < bms.getHighCellVolt()) {
    alarm_ve[0] = 0x04;
  }
  if (bms.getLowCellVolt() < settings.UnderVSetpoint) {
    alarm_ve[0] |= 0x10;
  }
  if (bms.getHighTemperature() > settings.OverTSetpoint) {
    alarm_ve[0] |= 0x40;
  }
  alarm_ve[1] = 0;
  if (bms.getLowTemperature() < settings.UnderTSetpoint) {
    alarm_ve[1] = 0x01;
  }
  alarm_ve[3] = 0;
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap) {
    alarm_ve[3] = 0x01;
  }
}

void printbmsstat() {
  Serial.print("Battery SOC:");
  Serial.println(SOC);

  Serial.print("BMS Errors : ");

  if (bms.getLowCellVolt() < settings.UnderVSetpoint) {
    Serial.print(": UnderVoltage ");
  }

  if (bms.getHighCellVolt() > settings.OverVSetpoint) {
    Serial.print(": OverVoltage ");
  }
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap) {
    Serial.print(": Cell Imbalance ");
  }
  if (bms.getAvgTemperature() > settings.OverTSetpoint) {
    Serial.print(": Over Temp ");
  }
  if (bms.getAvgTemperature() < settings.UnderTSetpoint) {
    Serial.print(": Under Temp ");
  }

  Serial.println();
}

void updateSOC() {

  uint16_t lowvoltage = uint16_t(bms.getLowCellVolt() * 1000);

  uint16_t avgvoltage = uint16_t(bms.getAvgCellVolt() * 1000);
  

   for (int i = 0; i < 100; i++) {
    
  if (lowvoltage > settings.ocv[i]) SOC = i+1;
  if (avgvoltage > settings.ocv[i]) SOC_physical = i+1;
  
    
    
    }


  
  
  // map(uint16_t(bms.getLowCellVolt() * 1000), settings.socvolt[0],settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);

  /*  ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) /
     1000);


      ampsecond = (SOC * settings.capacity * settings.parallel_strings * 10) /
     0.27777777777778 ; SOC = ((ampsecond * 0.27777777777778) / (settings.capacity *
     settings.parallel_strings * 1000)) * 100;

      ampsecond = (settings.capacity * settings.parallel_strings * 1000) / 0.27777777777778 ;
     //reset to full, dependant on given capacity. Need to improve with auto
     correction for capcity.
    */
}

void VEcan() // communication with Victron system over CAN
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
     if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK)
   {Serial.println("CAN SEND ERROR\n");}

     delay(2);

     msg.identifier  = 0x351;
     msg.data_length_code = 8;
     if (storagemode == 0)
     {
       msg.data[0] = lowByte(uint16_t((settings.ChargeVsetpoint *
   settings.Scells ) * 10)); msg.data[1] =
   highByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 10));
     }
     else
     {
       msg.data[0] = lowByte(uint16_t((settings.StoreVsetpoint * settings.Scells
   ) * 10)); msg.data[1] = highByte(uint16_t((settings.StoreVsetpoint *
   settings.Scells ) * 10));
     }
     msg.data[2] = lowByte(chargecurrent);
     msg.data[3] = highByte(chargecurrent);
     msg.data[4] = lowByte(discurrent );
     msg.data[5] = highByte(discurrent);
     msg.data[6] = 0x00;
     msg.data[7] = 0x00;
     if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK)
   {Serial.println("CAN SEND ERROR\n");}

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
     if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK)
   {Serial.println("CAN SEND ERROR\n");}

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
     if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK)
   {Serial.println("CAN SEND ERROR\n");}


     delay(2);

     msg.identifier  = 0x35C;
     msg.data_length_code = 2;
     msg.data[0] = 0xC0; //fixed charge and discharge enable for verifcation
     msg.data[1] = 0x00;
     if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK)
   {Serial.println("CAN SEND ERROR\n");}

     delay(2);

     msg.identifier  = 0x35E;
     msg.data_length_code = 2;
     msg.data[0] = "T"; //No idea how the naming works
     msg.data[1] = "P"; //No idea how the naming works
     if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK)
   {Serial.println("CAN SEND ERROR\n");}
   } */
  if (1) // else ..
  {
    msg.identifier = 0x351;
    msg.data_length_code = 8;

    msg.data[0] =
        lowByte(uint16_t((settings.ChargeVsetpoint * settings.series_cells) * 10));
    msg.data[1] =
        highByte(uint16_t((settings.ChargeVsetpoint * settings.series_cells) * 10));

    // msg.data[2] = lowByte(chargecurrent);
    // msg.data[3] = highByte(chargecurrent);
    // msg.data[4] = lowByte(discurrent );
    // msg.data[5] = highByte(discurrent);
    msg.data[6] =
        lowByte(uint16_t((settings.DischVsetpoint * settings.series_cells) * 10));
    msg.data[7] =
        highByte(uint16_t((settings.DischVsetpoint * settings.series_cells) * 10));

    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    msg.identifier = 0x355;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(SOC);
    msg.data[1] = highByte(SOC);
    msg.data[2] = lowByte(SOH);
    msg.data[3] = highByte(SOH);
    msg.data[4] = lowByte(SOC * 10);
    msg.data[5] = highByte(SOC * 10);
    msg.data[6] = 0;
    msg.data[7] = 0;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    msg.identifier = 0x356;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
    msg.data[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
    msg.data[2] = lowByte(long(currentact / 100));
    msg.data[3] = highByte(long(currentact / 100));
    msg.data[4] = lowByte(int16_t(bms.getAvgTemperature() * 10));
    msg.data[5] = highByte(int16_t(bms.getAvgTemperature() * 10));
    msg.data[6] = 0;
    msg.data[7] = 0;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    delay(2);
    msg.identifier = 0x35A;
    msg.data_length_code = 8;
    msg.data[0] = alarm_ve[0];   // High temp  Low Voltage | High Voltage
    msg.data[1] = alarm_ve[1];   // High Discharge Current | Low Temperature
    msg.data[2] = alarm_ve[2];   // Internal Failure | High Charge current
    msg.data[3] = alarm_ve[3];   // Cell Imbalance
    msg.data[4] = warning_ve[0]; // High temp  Low Voltage | High Voltage
    msg.data[5] = warning_ve[1]; // High Discharge Current | Low Temperature
    msg.data[6] = warning_ve[2]; // Internal Failure | High Charge current
    msg.data[7] = warning_ve[3]; // Cell Imbalance
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    msg.identifier = 0x35E;
    msg.data_length_code = 8;
    msg.data[0] = 'S';
    msg.data[1] = 'I';
    msg.data[2] = 'M';
    msg.data[3] = 'P';
    msg.data[4] = '3';
    msg.data[5] = '2';
    msg.data[6] = 'B';
    msg.data[7] = 'M';
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    delay(2);
    msg.identifier = 0x370;
    msg.data_length_code = 8;
    msg.data[0] = 'S';
    msg.data[1] = 'H';
    msg.data[2] = 'P';
    msg.data[3] = 'I';
    msg.data[4] = 'G';
    msg.data[5] = 'M';
    msg.data[6] = 'B';
    msg.data[7] = 'H';
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    delay(2);
    msg.identifier = 0x373;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(uint16_t(bms.getLowCellVolt() * 1000));
    msg.data[1] = highByte(uint16_t(bms.getLowCellVolt() * 1000));
    msg.data[2] = lowByte(uint16_t(bms.getHighCellVolt() * 1000));
    msg.data[3] = highByte(uint16_t(bms.getHighCellVolt() * 1000));
    msg.data[4] = lowByte(uint16_t(bms.getLowTemperature() + 273.15));
    msg.data[5] = highByte(uint16_t(bms.getLowTemperature() + 273.15));
    msg.data[6] = lowByte(uint16_t(bms.getHighTemperature() + 273.15));
    msg.data[7] = highByte(uint16_t(bms.getHighTemperature() + 273.15));
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    delay(2);
    msg.identifier = 0x379; // Installed capacity
    msg.data_length_code = 2;
    msg.data[0] = lowByte(uint16_t(settings.parallel_strings * settings.capacity));
    msg.data[1] = highByte(uint16_t(settings.parallel_strings * settings.capacity));
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
    msg.identifier = 0x372;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(bms.getNumModules());
    msg.data[1] = highByte(bms.getNumModules());
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }
  }
}










void HIGHcan(int addr) // communication HIGHVOLTAGE   CAN
{
  
    msg.identifier = 0x4210 + addr;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
    msg.data[1] = highByte(uint16_t(bms.getPackVoltage() * 100));

    msg.data[2] = lowByte(-3000 + long(currentact / 100));
    msg.data[3] = highByte(-3000 + long(currentact / 100));

    msg.data[4] = lowByte(-100 + int16_t(bms.getAvgTemperature() * 10));
    msg.data[5] = highByte(-100 + int16_t(bms.getAvgTemperature() * 10));
    msg.data[6] = lowByte(SOC);
    msg.data[7] = lowByte(SOH);
   
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    msg.identifier = 0x355;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(SOC);
    msg.data[1] = highByte(SOC);
    msg.data[2] = lowByte(SOH);
    msg.data[3] = highByte(SOH);
    msg.data[4] = lowByte(SOC * 10);
    msg.data[5] = highByte(SOC * 10);
    msg.data[6] = 0;
    msg.data[7] = 0;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    msg.identifier = 0x356;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
    msg.data[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
    msg.data[2] = lowByte(long(currentact / 100));
    msg.data[3] = highByte(long(currentact / 100));
    msg.data[4] = lowByte(int16_t(bms.getAvgTemperature() * 10));
    msg.data[5] = highByte(int16_t(bms.getAvgTemperature() * 10));
    msg.data[6] = 0;
    msg.data[7] = 0;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    delay(2);
    msg.identifier = 0x35A;
    msg.data_length_code = 8;
    msg.data[0] = alarm_ve[0];   // High temp  Low Voltage | High Voltage
    msg.data[1] = alarm_ve[1];   // High Discharge Current | Low Temperature
    msg.data[2] = alarm_ve[2];   // Internal Failure | High Charge current
    msg.data[3] = alarm_ve[3];   // Cell Imbalance
    msg.data[4] = warning_ve[0]; // High temp  Low Voltage | High Voltage
    msg.data[5] = warning_ve[1]; // High Discharge Current | Low Temperature
    msg.data[6] = warning_ve[2]; // Internal Failure | High Charge current
    msg.data[7] = warning_ve[3]; // Cell Imbalance
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    msg.identifier = 0x35E;
    msg.data_length_code = 8;
    msg.data[0] = 'S';
    msg.data[1] = 'I';
    msg.data[2] = 'M';
    msg.data[3] = 'P';
    msg.data[4] = '3';
    msg.data[5] = '2';
    msg.data[6] = 'B';
    msg.data[7] = 'M';
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    delay(2);
    msg.identifier = 0x370;
    msg.data_length_code = 8;
    msg.data[0] = 'S';
    msg.data[1] = 'H';
    msg.data[2] = 'P';
    msg.data[3] = 'I';
    msg.data[4] = 'G';
    msg.data[5] = 'M';
    msg.data[6] = 'B';
    msg.data[7] = 'H';
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    delay(2);
    msg.identifier = 0x373;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(uint16_t(bms.getLowCellVolt() * 1000));
    msg.data[1] = highByte(uint16_t(bms.getLowCellVolt() * 1000));
    msg.data[2] = lowByte(uint16_t(bms.getHighCellVolt() * 1000));
    msg.data[3] = highByte(uint16_t(bms.getHighCellVolt() * 1000));
    msg.data[4] = lowByte(uint16_t(bms.getLowTemperature() + 273.15));
    msg.data[5] = highByte(uint16_t(bms.getLowTemperature() + 273.15));
    msg.data[6] = lowByte(uint16_t(bms.getHighTemperature() + 273.15));
    msg.data[7] = highByte(uint16_t(bms.getHighTemperature() + 273.15));
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }

    delay(2);
    msg.identifier = 0x379; // Installed capacity
    msg.data_length_code = 2;
    msg.data[0] = lowByte(uint16_t(settings.parallel_strings * settings.capacity));
    msg.data[1] = highByte(uint16_t(settings.parallel_strings * settings.capacity));
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
    msg.identifier = 0x372;
    msg.data_length_code = 8;
    msg.data[0] = lowByte(bms.getNumModules());
    msg.data[1] = highByte(bms.getNumModules());
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    if (!twai_transmit(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("CAN SEND ERROR\n");
    }
  }


void canread() {

  
  can_processed++;
  const uint32_t CAN_ID_VW_BMS_MODULE = 0x300;
  const uint32_t CAN_ID_TEMPERATURE_MEASUREMENT = 0x1A555400;
  const uint32_t CAN_ID_TEMPERATURE_MEASUREMENT_END = 0x1A555440;
  const uint32_t CAN_ID_TEMPERATURE_MEASUREMENT_MEB = 0x1A5555F0;
  const uint32_t CAN_ID_STATUS_MEB = 0x16A95470;


  if (twai_receive(&inmsg, pdMS_TO_TICKS(10000)) == ESP_OK) {
  
    
  if (inmsg.identifier < CAN_ID_VW_BMS_MODULE) {bms.decodecan(inmsg, 0);} 
  else

  switch (inmsg.identifier & 0x1FFFFFF0) {

    case CAN_ID_STATUS_MEB:  // store last MEB status for displaying bleed status
         lastcancounter = (lastcancounter + 1) % 10;
         lastcans[lastcancounter] = String(inmsg.identifier, HEX) + " ";
         if (inmsg.extd)  lastcans[lastcancounter] += " X ";
         else             lastcans[lastcancounter] += " S ";
         lastcans[lastcancounter] += String(inmsg.data_length_code);
         lastcans[lastcancounter] += ": ";
         for (int i = 0; i < inmsg.data_length_code; i++)   lastcans[lastcancounter] += " " + String(inmsg.data[i], HEX);
         break;  


    case CAN_ID_TEMPERATURE_MEASUREMENT ... CAN_ID_TEMPERATURE_MEASUREMENT_END:
         bms.decodetemp(inmsg, 0, 1);
         break;

    case CAN_ID_TEMPERATURE_MEASUREMENT_MEB:
         bms.decodetemp(inmsg, 0, 2);
         break;

     default: break;
         
  }

  
    
}


 



    }
