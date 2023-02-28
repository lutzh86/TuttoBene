
#include "BMSModuleManager.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#define MAX_MODULE_ADDR 0x3E

// extern String lastcans[10];
extern EEPROMSettings settings;
extern uint32_t alerts_triggered;
extern uint32_t can_processed;
extern twai_status_info_t twaistatus;
extern int SOC, SOC_physical;
extern int nmc712[100];
extern int nmc712_nominal_voltage;
extern struct canLog canBuffer[100];

uint16_t balance = 0; // bit 0 - 11 are to activate cell balancing 1-12


twai_message_t outmsg;

BMSModuleManager::BMSModuleManager() {
  for (int i = 1; i <= MAX_MODULE_ADDR; i++) {
    modules[i].setExists(false);
    
  }


  isFaulted = false;
  balcnt = 0;
}

bool BMSModuleManager::checkcomms() {
  int g = 0;
  for (int y = 1; y < 63; y++) {
    if (modules[y].isExisting()) {
      g = 1;
      if (modules[y].isReset()) {
        // Do nothing as the counter has been reset
      } else {
        modules[y].setExists(false);
        return false;
      }
    }
    modules[y].setReset(false);
    ;
  }
  if (g == 0) {
    return false;
  }
  return true;
}

void BMSModuleManager::setBalanceHyst(float newVal) { BalHys = newVal; }

void BMSModuleManager::sendcommand() {

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
  message.identifier = controlid;
  message.data_length_code = 8;
  message.extd = 0;
  message.ss = 1;

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) != ESP_OK) {
    Serial.println("CAN SEND ERROR\n");
  }

  message.data[0] = 0x45;
  message.data[1] = 0x01;
  message.data[2] = 0x28;
  message.data[3] = 0x00;
  message.data[4] = 0x00;
  message.data[5] = 0x00;
  message.data[6] = 0x00;
  message.data[7] = 0x30;
  message.identifier = controlid;
  message.data_length_code = 8;
  message.extd = 0;

  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) != ESP_OK) {
    Serial.println("CAN SEND ERROR\n");
  }
}

void BMSModuleManager::balanceCells(int debug, int onoff) {

 
  for (int y = 1; y < 63; y++) {
    if (modules[y].isExisting() == 1) {
      
      balance = 0;
      for (int i = 0; i < 14; i++) {
        // if ((LowCellVolt + BalHys) < modules[y].getCellVoltage(i))
        if ((modules[y].getLowCellV() + BalHys) < modules[y].getCellVoltage(i))

        {

          balance = balance | (1 << i);
        }
      }

      if (onoff == 0) {
        balcnt = 1;
        balance = 0;
      }

      if (balance != 0)
        balcnt = 0;
           

      if (1) { // balance != modules[y].getBalance()

        modules[y].setBalance(balance);

        if (debug == 1) {
          Serial.println();
          Serial.print("Module ");
          Serial.printf("%2d", y);
          Serial.print(" | ");
          Serial.println(balance, HEX);
        }

        for (int i = 0; i < 8; i++) {
          if (bitRead(balance, i) == 1) {
            outmsg.data[i] = 0xFF;
            if (debug == 1) {
              Serial.print("Set cell  ");
              Serial.print(i);
              Serial.println(" bleeding resistor on.");
            }
          } else {
            outmsg.data[i] = 0x00;
          }
        }

       const uint32_t command_bleed_table[] = {
                      0x1A55540A,  // y = 1
                      0x1A55540C,  // y = 2
                      0x1A55540E,  // y = 3
                      0x1A555410,  // y = 4
                      0x1A555412,  // y = 5
                      0x1A555414,  // y = 6
                      0x1A555416,  // y = 7
                      0x1A555418,  // y = 8
                      0x1A55541A,  // y = 9
                      0x1A5554AB,  // y = 10
                      0x1A5554AD,  // y = 11
                      0x1A5554AF   // y = 12
                      };

        if (y >= 1 && y <= 12) outmsg.identifier = command_bleed_table[y-1];


        outmsg.data_length_code = 8;
        outmsg.extd = 1;
        outmsg.ss = 0;
        if (!twai_transmit(&outmsg, pdMS_TO_TICKS(1000)) == ESP_OK) {
          Serial.println("CAN SEND ERROR\n");
        }

        Serial.print("Sent frame to id: ");
        Serial.print(outmsg.identifier, HEX);
        if (outmsg.extd)
          Serial.print(" X ");
        else
          Serial.print(" S ");
        Serial.print("Length:");
        Serial.print(outmsg.data_length_code, DEC);
        Serial.print(" - ");
        for (int i = 0; i < outmsg.data_length_code; i++) {
          Serial.print(outmsg.data[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        delay(1);

        for (int i = 8; i < 14; i++) {
          if (bitRead(balance, i) == 1) {
            outmsg.data[i - 8] = 0xFF;
            Serial.print("Set cell  ");
            Serial.print(i);
            Serial.println(" bleeding resistor on.");
          } else {
            outmsg.data[i - 8] = 0x00;
          }
        }
        // outmsg.data[4] = 0xFE;
        // outmsg.data[5] = 0xFE;
        outmsg.data[6] = 0xFE;
        outmsg.data[7] = 0xFE;

         const uint32_t command_bleed_table_2[] = {
                     0x1A55540B, 0x1A55540D, 0x1A55540F, 0x1A555411,
                     0x1A555413, 0x1A555415, 0x1A555417, 0x1A555419,
                     0x1A55541B, 0x1A5554AC, 0x1A5554AE, 0x1A5554B0
                     };

          if (y >= 1 && y <= 12) {
          // Set the identifier based on the lookup table
          outmsg.identifier = command_bleed_table_2[y - 1];
        }

        
        outmsg.rtr = 0;
        outmsg.data_length_code = 8;
        outmsg.extd = 1;
        outmsg.ss = 0;
        if (!twai_transmit(&outmsg, pdMS_TO_TICKS(1000)) == ESP_OK) {
          Serial.println("CAN SEND ERROR\n");
        }

        Serial.print("Sent frame to id: ");
        Serial.print(outmsg.identifier, HEX);
        if (outmsg.extd)
          Serial.print(" X ");
        else
          Serial.print(" S ");
        Serial.print(outmsg.data_length_code, DEC);
        Serial.print(" ");
        for (int i = 0; i < outmsg.data_length_code; i++) {
          Serial.print(outmsg.data[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
    }
  }
}

int BMSModuleManager::seriescells() {
  spack = 0;
  for (int y = 1; y < 63; y++) {
    if (modules[y].isExisting()) {
      spack = spack + modules[y].getscells();
    }
  }
  return spack;
}

void BMSModuleManager::clearmodules() {
  for (int y = 1; y < 63; y++) {
    if (modules[y].isExisting()) {
      modules[y].clearmodule();
      modules[y].setExists(false);
    }
  }
}

void BMSModuleManager::decodetemp(twai_message_t &msg, int debug, int type) {
  int CMU = 0;
  if (type == 1) {
    CMU = (msg.identifier & 0xFF);
    if (CMU > 10 && CMU < 60) {
      CMU = CMU & 0x0F;
      CMU = (CMU * 0.5) + 1;
    }
    if (CMU > 0 && CMU < 15) {
      modules[CMU].setExists(true);
      modules[CMU].setReset(true);
      modules[CMU].decodetemp(msg, 1);
      if (debug == 1) {
        Serial.println();
        Serial.print(CMU);
        Serial.print(" | Temp Found");
      }
    }
  }
  else if (type == 2) {
    CMU = (msg.identifier & 0x0F);
    if (CMU > -1 && CMU < 15) {
      CMU++;
      if (msg.data[5] !=
          0xDF) // Check module is not initializing OR a "spoof module"
      {
        modules[CMU].setExists(true);
        modules[CMU].setReset(true);
        modules[CMU].decodetemp(msg, 2);
        if (debug == 1) {
          Serial.println();
          Serial.print(CMU);
          Serial.print("|  Temp Found");
        }
      }
    }
  } else
  if (type == 3) {
    CMU = (msg.identifier & 0x0F);
    if (CMU > -1 && CMU < 15) {
      CMU++;
      if (msg.data[5] !=
          0xDF) // Check module is not initializing OR a "spoof module"
      {
        modules[CMU].setExists(true);
        modules[CMU].setReset(true);
        modules[CMU].decodetemp(msg, 3);
        if (debug == 1) {
          Serial.println();
          Serial.print(CMU);
          Serial.print("|  Temp Found");
        }
      }
    }
  }
}

void BMSModuleManager::decodecan(twai_message_t &msg, int debug) {
  int CMU, Id = 0;

   switch (msg.identifier) {
   
  case 0x1B0 ... 0x1B3:
    CMU = 1;
    break;
  case 0x1B4 ... 0x1B7:
    CMU = 2;
    break;
  case 0x1B8 ... 0x1BB:
    CMU = 3;
    break;
  case 0x1BC ... 0x1BF:
    CMU = 4;
    break;
  case 0x1C0 ... 0x1C3:
    CMU = 5;
    break;
  case 0x1C4 ... 0x1C7:
    CMU = 6;
    break;
  case 0x1C8 ... 0x1CB:
    CMU = 7;
    break;
  case 0x1CC ... 0x1CF:
    CMU = 8;
    break;
  case 0x1D0 ... 0x1D3:
    CMU = 9;
    break;
  case 0x1D4 ... 0x1D7:
    CMU = 10;
    break;
  case 0x1D8 ... 0x1DB:
    CMU = 11;
    break;
  case 0x1DC ... 0x1DF:
    CMU = 12;
    break;
  case 0x1E0 ... 0x1E3:
    CMU = 13;
    break;
  case 0x1E4 ... 0x1E7:
    CMU = 14;
    break;
  case 0x1E8 ... 0x1EB:
    CMU = 15;
    break;
  case 0x1EC ... 0x1EF:
    CMU = 16;
    break;

  default:
    return;
    break;
  }


  Id = msg.identifier & 0x3;
  

  
  if (CMU > 0 && CMU < 64) {
    if (Id < 3) {
      if (msg.data[2] != 0xFF && msg.data[5] != 0xFF &&
          msg.data[7] != 0xFF) // Check module is not initializing OR a "spoof module"
      {
        if (debug == 1) {
          Serial.println();
          Serial.print(CMU);
          Serial.print(",");
          Serial.print(Id);
          Serial.print(" | ");
        }
        modules[CMU].setExists(true);
        modules[CMU].setReset(true);
        if (balcnt > 0) {
            modules[CMU].decodecan(Id, msg);
          
      }}
    } else {
      if (msg.data[2] !=
          0xFF) // Check module is not initializing OR a "spoof module"
      {
        if (debug == 1) {
          Serial.println();
          Serial.print(CMU);
          Serial.print(",");
          Serial.print(Id);
          Serial.print(" | ");
        }
        modules[CMU].setExists(true);
        modules[CMU].setReset(true);
        if (balcnt > 0) {
          modules[CMU].decodecan(Id, msg);
          
      }
      }
    }
  }
}

void BMSModuleManager::getAllVoltTemp() {
  packVolt = 0.0f;

  for (int x = 1; x <= MAX_MODULE_ADDR; x++) {
    if (modules[x].isExisting()) {

      Serial.printf("Module %i exists. Reading voltage and temperature values",
                    x);
      Serial.printf("Module voltage: %f", modules[x].getModuleVoltage());
      Serial.printf("Lowest Cell V: %f     Highest Cell V: %f",
                    modules[x].getLowCellV(), modules[x].getHighCellV());
      Serial.printf("Temp1: %f       Temp2: %f", modules[x].getTemperature(0),
                    modules[x].getTemperature(1));
      packVolt += modules[x].getModuleVoltage();
    }
  }

  packVolt = packVolt / Pstring;

}

float BMSModuleManager::getLowCellVolt() {
  LowCellVolt = 5.0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++) {
    if (modules[x].isExisting()) {
      if (modules[x].getLowCellV() < LowCellVolt)
        LowCellVolt = modules[x].getLowCellV();
    }
  }
  return LowCellVolt;
}

float BMSModuleManager::getHighCellVolt() {
  HighCellVolt = 0.0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++) {
    if (modules[x].isExisting()) {
      if (modules[x].getHighCellV() > HighCellVolt)
        HighCellVolt = modules[x].getHighCellV();
    }
  }
  return HighCellVolt;
}

float BMSModuleManager::getPackVoltage() { return packVolt; }

int BMSModuleManager::getNumModules() { return numFoundModules; }



void BMSModuleManager::setPstrings(int Pstrings) { Pstring = Pstrings; }

float BMSModuleManager::getAvgTemperature() {
  float avg = 0.0f;
  lowTemp = 999.0f;
  highTemp = -999.0f;
  int y = 0; // counter for modules below -70 (no sensors connected)
  for (int x = 0; x <= MAX_MODULE_ADDR; x++) {
    if (modules[x].isExisting()) {
      if (modules[x].getAvgTemp() > -70) {
        avg += modules[x].getAvgTemp();
        if (modules[x].getHighTemp() > highTemp) {
          highTemp = modules[x].getHighTemp();
        }
        if (modules[x].getLowTemp() < lowTemp) {
          lowTemp = modules[x].getLowTemp();
        }
      } else {
        y++;
      }
    }
  }
  avg = avg / (float)(numFoundModules - y);

  return avg;
}

float BMSModuleManager::getHighTemperature() { return highTemp; }

float BMSModuleManager::getLowTemperature() { return lowTemp; }

float BMSModuleManager::getAvgCellVolt() {
  numFoundModules = 0;
  float avg = 0.0f;
  for (int x = 0; x <= MAX_MODULE_ADDR; x++) {
    if (modules[x].isExisting()) {
      if (modules[x].getAverageV() > 0) {
        avg += modules[x].getAverageV();
        numFoundModules++;
      }
    }
  }
  avg = avg / (float)numFoundModules;

  return avg;
}

void BMSModuleManager::printPackSummary() {
  uint8_t faults = 0;
  uint8_t alerts = 0;
  uint8_t COV = 0;
  uint8_t CUV = 0;

  Serial.printf("Modules: %i  Cells: %i  Voltage: %fV   Avg Cell Voltage: %fV  "
                "   Avg Temp: %fC ",
                numFoundModules, seriescells(), getPackVoltage(),
                getAvgCellVolt(), getAvgTemperature());
  Serial.println();
  for (int y = 0; y < 63; y++) {
    if (modules[y].isExisting()) {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      Serial.printf("Module #%i", y);

      Serial.printf("  Voltage: %fV   (%fV-%fV)     Temperatures: (%fC-%fC)",
                    modules[y].getModuleVoltage(), modules[y].getLowCellV(),
                    modules[y].getHighCellV(), modules[y].getLowTemp(),
                    modules[y].getHighTemp());
      if (faults > 0) {
        Serial.print("  MODULE IS FAULTED:");
        if (faults & 1) {
          Serial.print("    Overvoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 14; i++) {
            if (COV & (1 << i)) {
              Serial.print(i + 1);
              Serial.print(" ");
            }
          }
          Serial.println();
        }
        if (faults & 2) {
          Serial.print("    Undervoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 14; i++) {
            if (CUV & (1 << i)) {
              Serial.print(i + 1);
              Serial.print(" ");
            }
          }
          Serial.println();
        }
        if (faults & 4) {
          Serial.println("    CRC error in received packet");
        }
        if (faults & 8) {
          Serial.println("    Power on reset has occurred");
        }
        if (faults & 0x10) {
          Serial.println("    Test fault active");
        }
        if (faults & 0x20) {
          Serial.println("    Internal registers inconsistent");
        }
      }
      if (alerts > 0) {
        Serial.println("  MODULE HAS ALERTS:");
        if (alerts & 1) {
          Serial.println("    Over temperature on TS1");
        }
        if (alerts & 2) {
          Serial.println("    Over temperature on TS2");
        }
        if (alerts & 4) {
          Serial.println("    Sleep mode active");
        }
        if (alerts & 8) {
          Serial.println("    Thermal shutdown active");
        }
        if (alerts & 0x10) {
          Serial.println("    Test Alert");
        }
        if (alerts & 0x20) {
          Serial.println("    OTP EPROM Uncorrectable Error");
        }
        if (alerts & 0x40) {
          Serial.println("    GROUP3 Regs Invalid");
        }
        if (alerts & 0x80) {
          Serial.println("    Address not registered");
        }
      }
      if (faults > 0 || alerts > 0)
        Serial.println();
    }
  }
}

String BMSModuleManager::htmlPackDetails(int digits) {

  String ptr = "<!DOCTYPE html> <html>\n";

  ptr += "<head><meta http-equiv=\"refresh\" content=\"2; URL=/\"></head>";

  int cellNum = 0;

  ptr += "Modules:" + String(numFoundModules);
  ptr += " Cells: " + String(seriescells());
  ptr += " Voltage: " + String(getPackVoltage());
  ptr += " Avg Cell Voltage: " + String(getAvgCellVolt());
  ptr += " Low Cell Voltage: " + String(getLowCellVolt());
  ptr += " High Cell Voltage: " + String(getHighCellVolt());
  ptr += " Avg Temp: " + String(getAvgTemperature()) + "&deg;C";

  ptr += "<p>SOC useable:" + String(SOC) + "%";
  ptr += "<p>SOC physical:" + String(SOC_physical) + "%";
  ptr +=         "<p>Full capacity:" + String(settings.series_cells * settings.parallel_strings * settings.capacity * settings.nominal_cell_voltage * 0.000001 ) + "kWh";
  ptr += "<p>Useable left capacity:" + String(float(SOC * 0.01) * settings.series_cells * settings.parallel_strings * settings.capacity * settings.nominal_cell_voltage * 0.001 * 0.001) + "kWh";


  
  ptr += "<p><table>";

  

  int header = 0;

  for (int y = 1; y < 63; y++) {
    if (modules[y].isExisting()) {
      // faults = modules[y].getFaults();
      // alerts = modules[y].getAlerts();
      // COV = modules[y].getCOVCells();
      // CUV = modules[y].getCUVCells();

      if (header == 0)

      {
        ptr += "<tr><td></td><td>Voltage</td><td>Drift</td>";
        for (int i = 0; i < 14; i++)

        {
          if (modules[y].getCellVoltage(i) > 0) {

            ptr += ("<td>Cell");
            ptr += String(cellNum++);
            ptr += "</td>";
          }
        }
        ptr += "<td>Temperature</td></tr>";
        cellNum = 0;
        header = 1;
      }

      ptr += "<tr><td>Module #";
      ptr += (y < 10 ? "0" : "") + String(y);
      ptr += "</td><td><b>";
      ptr += String(modules[y].getModuleVoltage(), digits) + "V</b></td><td>";
      ptr += String((modules[y].getHighCellV() - modules[y].getLowCellV()) *
                    1000) +
             "mV</td>";

      for (int i = 0; i < 14; i++) {

        if (modules[y].getCellVoltage(i) > 0) {

          ptr += ("<td>");
          cellNum++;
          // ptr += (cellNum < 10 ? "0" : "") + String(cellNum);
          // ptr += (": ");
          if (modules[y].getLowCellV() == modules[y].getCellVoltage(i)) {
            ptr += "<u style=\"color:blue\";>";
            ptr += String(modules[y].getCellVoltage(i), digits);
            ptr += "V</u>";
          } else if (modules[y].getHighCellV() ==
                     modules[y].getCellVoltage(i)) {
            ptr += "<u style=\"color:red\";>";
            ptr += String(modules[y].getCellVoltage(i), digits);
            ptr += "V</u>";
          }

          else {
            ptr += String(modules[y].getCellVoltage(i), digits);
            ptr += "V";
          }

          if (bitRead(modules[y].getBalance(), i) == 1) {
            ptr += ("&darr; ");
          } else {
            ptr += ("&nbsp; ");
          }
          ptr += "</td>";
        }
      }

      if (modules[y].getType() == 1) {
        ptr += ("<td>");
        ptr += String(modules[y].getTemperature(0), 2);
        ptr += ("&deg;C,  ");
        ptr += String(modules[y].getTemperature(1), 2);
        ptr += ("&deg;C,  ");
        ptr += String(modules[y].getTemperature(2));
        ptr += ("&deg;C</td>");

      } else {
        ptr += ("<td>");
        ptr += String(modules[y].getTemperature(0), 2);
        ptr += ("&deg;C</td>");
   ptr += ("<td>");
        ptr += String(modules[y].getTemperature(1), 2);
        ptr += ("&deg;C</td>");
 

      }
      ptr += "</tr>";
    }
  }

  ptr += "</table>";
  ptr += "<br><b>CAN STATUS:</b> (";

  ptr += alerts_triggered;
  ptr += ")";

  if (alerts_triggered & TWAI_ALERT_BUS_OFF)
    ptr += "TWAI_ALTER_BUS_OFF,";
  if (alerts_triggered & TWAI_ALERT_RX_DATA)
    ptr += "TWAI_ALERT_RX_DATA,";
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
    ptr += "TWAI_ALERT_BUS_ERROR,";
  if (alerts_triggered & TWAI_ALERT_TX_FAILED)
    ptr += "TWAI_ALERT_TX_FAILED,";
  if (alerts_triggered & TWAI_ALERT_ERR_PASS)
    ptr += "TWAI_ALERT_ERR_PASS,";
  if (alerts_triggered & TWAI_ALERT_ABOVE_ERR_WARN)
    ptr += "TWAI_ALERT_ABOVE_ERR_WARN,";
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL)
    ptr += "TWAI_ALERT_RX_QUEUE_FULL,";

  ptr += "<br>actual state:";

  if (twaistatus.state & TWAI_STATE_RUNNING)
    ptr += " RUNNING ";
  if (twaistatus.state & TWAI_STATE_STOPPED)
    ptr += " STOPPED ";
  if (twaistatus.state & TWAI_STATE_BUS_OFF)
    ptr += " BUS OFF ";
  // if (twaistatus.state & TWAI_STATE_RECOVERING) ptr +=" (RECOVERING) ";

  ptr += "<br>Bus error count:";
  ptr += String(twaistatus.bus_error_count);

  ptr += "<br>TX buffered:";
  ptr += String(twaistatus.msgs_to_tx);

  ptr += "<br>TX errors:";
  ptr += String(twaistatus.tx_error_counter);

  ptr += "<br>RX buffered:";
  ptr += String(twaistatus.msgs_to_rx);

  ptr += "<br>RX processed:";
  ptr += String(can_processed);

  ptr += "<br>RX missed:";
  ptr += String(twaistatus.rx_missed_count);

  ptr += "<br>RX overrun:";
  ptr += String(twaistatus.rx_overrun_count);

  ptr += "<p><br>\n";

  /*

  for (int i = 0; i < 10; i++) {
    ptr += lastcans[i] + "<br>";
  }

 */


 for (int i = 0; i < 100; i++) {

  if (canBuffer[i].identifier) {
  ptr += String(canBuffer[i].identifier,HEX)+ " : ";
  for (int a = 0; a < 8; a++)   ptr += " " + ((String(canBuffer[i].data[a],HEX).length() < 2 ) ? ( "0" + String(canBuffer[i].data[a],HEX)) : String(canBuffer[i].data[a],HEX));
  ptr += "<br>";} else break;
         
}
  
  


  ptr += "<br></body>\n";

  ptr += "</html>\n";
  return ptr;
}

void BMSModuleManager::printPackDetails(int digits) {
  // int faults = 0;  int alerts= 0;  int COV = 0;  int CUV = 0;

  int cellNum = 0;

  Serial.printf("Modules: %i Cells: %i Strings: %i  Voltage: %fV   Avg Cell "
                "Voltage: %fV  Low Cell Voltage: %fV   High Cell Voltage: %fV "
                "Delta Voltage: %lfmV   Avg Temp: %fC ",
                numFoundModules, seriescells(), Pstring, getPackVoltage(),
                getAvgCellVolt(), LowCellVolt, HighCellVolt,
                (HighCellVolt - LowCellVolt) * 1000, getAvgTemperature());

  for (int y = 0; y < 63; y++) {
    if (modules[y].isExisting()) {
      // faults = modules[y].getFaults();
      // alerts = modules[y].getAlerts();
      // COV = modules[y].getCOVCells();
      // CUV = modules[y].getCUVCells();

      Serial.print("Module #");
      Serial.printf("%2d", y);
      Serial.print(" ");
      Serial.print(modules[y].getModuleVoltage(), digits);
      Serial.print("V");
      for (int i = 0; i < 14; i++) {

        if (modules[y].getCellVoltage(i) > 0) {

          Serial.print(" Cell");
          cellNum++;
          Serial.printf("%2d", cellNum);
          Serial.print(": ");
          Serial.print(modules[y].getCellVoltage(i), digits);
          Serial.print("V");

          if (bitRead(modules[y].getBalance(), i) == 1) {
            Serial.print("↓ ");
          } else {
            Serial.print("  ");
          }
        }
      }

      if (modules[y].getType() == 1) {
        Serial.print(" Temp 1: ");
        Serial.print(modules[y].getTemperature(0));
        Serial.print("C Temp 2: ");
        Serial.print(modules[y].getTemperature(1));
        Serial.print("C Temp 3: ");
        Serial.print(modules[y].getTemperature(2));
        Serial.println("C");

      } else {
        Serial.print(" Temp 1: ");
        Serial.print(modules[y].getTemperature(0));
        Serial.println("C");
      }
    }
  }
}
