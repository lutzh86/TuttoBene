#include "CONFIG.H"
#include "BMSModuleManager.h"
#include "BMSUtil.h"
#include "Logger.h"
#include "driver/gpio.h"
#include "driver/twai.h"


#define MAX_MODULE_ADDR     0x3E

extern EEPROMSettings settings;
extern String lastcans[100];

twai_message_t outmsg;

 


BMSModuleManager::BMSModuleManager()
{
  for (int i = 1; i <= MAX_MODULE_ADDR; i++) {
    modules[i].setExists(false);
    modules[i].setAddress(i);
  }
  lowestPackVolt = 1000.0f;
  highestPackVolt = 0.0f;
  lowestPackTemp = 200.0f;
  highestPackTemp = -100.0f;
  isFaulted = false;
  
  balcnt = 0;//counter to stop balancing for cell measurement
}

bool BMSModuleManager::checkcomms()
{
  int g = 0;
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      g = 1;
      if (modules[y].isReset())
      {
        //Do nothing as the counter has been reset
      }
      else
      {
        modules[y].setExists(false);
        return false;
      }
    }
    modules[y].setReset(false);
    modules[y].setAddress(y);
  }
  if ( g == 0)
  {
    return false;
  }
  return true;
}

void BMSModuleManager::setBalanceHyst(float newVal)
{
  BalHys = newVal;
  //Serial.println();
  //Serial.println(BalHys, 3);
}

void BMSModuleManager::balanceCells(int debug, int onoff)
{

  uint16_t balance = 0; //bit 0 - 11 are to activate cell balancing 1-12
 
     
      for (int y = 1; y < 63; y++)
      {
        if (modules[y].isExisting() == 1)
        {
          balance = 0;
          for (int i = 0; i < 13; i++)
          {
           //if ((LowCellVolt + BalHys) < modules[y].getCellVoltage(i))
         if ((modules[y].getLowCellV() + BalHys) < modules[y].getCellVoltage(i))
                  
            
            {
              
               balance = balance | (1 << i);
            }
           
          }

          if (onoff == 0) {balance = 0; }

          if (balance != modules[y].getBalance()) {
            
          
          modules[y].setBalance(balance);
          
          
          if (debug == 1)
          {
            Serial.println();
            Serial.print("Module ");
            Serial.printf("%2d", y);            
            Serial.print(" | ");
            Serial.println(balance, HEX);

          }


          for (int i = 0; i < 8; i++)
          {
            if (bitRead(balance, i) == 1)
            {
              outmsg.data[i] = 0x08;
              if (debug == 1)
          {
               Serial.print("Set cell  ");
               Serial.print(i);
               Serial.println(" bleeding resistor on.");
            }}
            else
            {
              outmsg.data[i] = 0x00;
            }
          }

          switch (y)
          {
            case (1):
              outmsg.identifier  = 0x1A55540A;
              break;
            case (2):
              outmsg.identifier  = 0x1A55540C;
              break;
            case (3):
              outmsg.identifier  = 0x1A55540E;
              break;
            case (4):
              outmsg.identifier  = 0x1A555410;
              break;
            case (5):
              outmsg.identifier  = 0x1A555412;
              break;
            case (6):
              outmsg.identifier  = 0x1A555414;
              break;
            case (7):
              outmsg.identifier  = 0x1A555416;
              break;
            case (8):
              outmsg.identifier  = 0x1A555418;
              break;
            case (9):
              outmsg.identifier  = 0x1A55541A;
              break;
            case (10):
              outmsg.identifier  = 0x1A5554AB;
              break;
            case (11):
              outmsg.identifier  = 0x1A5554AD;
              break;
            case (12):
              outmsg.identifier  = 0x1A5554AF;
              break;

            default:
              break;
          }

        

          outmsg.data_length_code = 8;
          outmsg.extd = 1;
          outmsg.ss = 1;
          if (!twai_transmit(&outmsg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 

         
          Serial.print("Sent frame to id: ");
          Serial.print(outmsg.identifier, HEX);
          if (outmsg.extd) Serial.print(" X ");
          else Serial.print(" S ");   
          Serial.print(outmsg.data_length_code, DEC);
           Serial.print(" ");
  for (int i = 0; i < outmsg.data_length_code; i++) {
    Serial.print(outmsg.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

          delay(1);

          for (int i = 8; i < 12; i++)
          {
            if (bitRead(balance, i) == 1)
            {
               outmsg.data[i - 8] = 0x08;
               Serial.print("Set cell  ");
               Serial.print(i);
               Serial.println(" bleeding resistor on.");
            }
            else
            {
              outmsg.data[i - 8] = 0x00;
            }
          }
          outmsg.data[4] = 0xFE;
          outmsg.data[5] = 0xFE;
          outmsg.data[6] = 0xFE;
          outmsg.data[7] = 0xFE;

          switch (y)
          {
            case (1):
              outmsg.identifier  = 0x1A55540B;
              break;
            case (2):
              outmsg.identifier  = 0x1A55540D;
              break;
            case (3):
              outmsg.identifier  = 0x1A55540F;
              break;
            case (4):
              outmsg.identifier  = 0x1A555411;
              break;
            case (5):
              outmsg.identifier  = 0x1A555413;
              break;
            case (6):
              outmsg.identifier  = 0x1A555415;
              break;
            case (7):
              outmsg.identifier  = 0x1A555417;
              break;
            case (8):
              outmsg.identifier  = 0x1A555419;
              break;
            case (9):
              outmsg.identifier  = 0x1A55541B;
              break;
            case (10):
              outmsg.identifier  = 0x1A5554AC;
              break;
            case (11):
              outmsg.identifier  = 0x1A5554AE;
              break;
            case (12):
              outmsg.identifier  = 0x1A5554B0;
              break;

            default:
              break;
          }
           outmsg.rtr = 0;
          outmsg.data_length_code = 8;
          outmsg.extd = 1;
          outmsg.ss = 1;
          if (!twai_transmit(&outmsg, pdMS_TO_TICKS(1000)) == ESP_OK) {Serial.println("CAN SEND ERROR\n");} 

         
          Serial.print("Sent frame to id: ");
          Serial.print(outmsg.identifier, HEX);
          if (outmsg.extd) Serial.print(" X ");
          else Serial.print(" S ");   
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




int BMSModuleManager::seriescells()
{
  spack = 0;
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      spack = spack + modules[y].getscells();
    }
  }
  return spack;
}

void BMSModuleManager::clearmodules()
{
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      modules[y].clearmodule();
      modules[y].setExists(false);
      modules[y].setAddress(y);
    }
  }
}

void BMSModuleManager::decodetemp(twai_message_t &msg, int debug, int type)
{
  int CMU = 0;
  if (type == 1)
  {
    CMU = (msg.identifier & 0xFF);
    if (CMU > 10 && CMU < 60)
    {
      CMU = CMU & 0x0F;
      CMU = (CMU * 0.5) + 1;
    }
    if (CMU > 0 && CMU < 15)
    {
      modules[CMU].setExists(true);
      modules[CMU].setReset(true);
      modules[CMU].decodetemp(msg, 1);
      if (debug == 1)
      {
        Serial.println();
        Serial.print(CMU);
        Serial.print(" | Temp Found");
      }
    }
  }
  if (type == 2)
  {
    CMU = (msg.identifier & 0x0F);
    if (CMU > 0 && CMU < 15)
    {
      CMU++;
      if (msg.data[5] != 0xDF) //Check module is not initializing OR a "spoof module"
      {
        modules[CMU].setExists(true);
        modules[CMU].setReset(true);
        modules[CMU].decodetemp(msg, 2);
        if (debug == 1)
        {
          Serial.println();
          Serial.print(CMU);
          Serial.print("|  Temp Found");
        }
      }
    }
  }
}

void BMSModuleManager::decodecan(twai_message_t &msg, int debug)
{
  int CMU, Id = 0;
  
    switch (msg.identifier)
    {
      ///////////////// one extender increment//////////

      case (0x1D0):
        CMU = 9;
        Id = 0;
        break;
      case (0x1D1):
        CMU = 9;
        Id = 1;
        break;
      case (0x1D2):
        CMU = 9;
        Id = 2;
        break;
      case (0x1D3):
        CMU = 9;
        Id = 3;
        break;

      case (0x1D4):
        CMU = 10;
        Id = 0;
        break;
      case (0x1D5):
        CMU = 10;
        Id = 1;
        break;
      case (0x1D6):
        CMU = 10;
        Id = 2;
        break;
      case (0x1D8):
        CMU = 11;
        Id = 0;
        break;
      case (0x1D9):
        CMU = 11;
        Id = 1;
        break;
      case (0x1DA):
        CMU = 11;
        Id = 2;
        break;
      case (0x1DC):
        CMU = 12;
        Id = 0;
        break;
      case (0x1DD):
        CMU = 12;
        Id = 1;
        break;
      case (0x1DE):
        CMU = 12;
        Id = 2;
        break;

      case (0x1E0):
        CMU = 13;
        Id = 0;
        break;
      case (0x1E1):
        CMU = 13;
        Id = 1;
        break;
      case (0x1E2):
        CMU = 13;
        Id = 2;
        break;

      case (0x1E4):
        CMU = 14;
        Id = 0;
        break;
      case (0x1E5):
        CMU = 14;
        Id = 1;
        break;
      case (0x1E6):
        CMU = 14;
        Id = 2;
        break;

      case (0x1E8):
        CMU = 15;
        Id = 0;
        break;
      case (0x1E9):
        CMU = 15;
        Id = 1;
        break;
      case (0x1EA):
        CMU = 15;
        Id = 2;
        break;

      case (0x1EC):
        CMU = 16;
        Id = 0;
        break;
      case (0x1ED):
        CMU = 16;
        Id = 1;
        break;
      case (0x1EE):
        CMU = 16;
        Id = 2;
        break;


      ///////////////////////standard ids////////////////


      case (0x1B0):
        CMU = 1;
        Id = 0;
        break;
      case (0x1B1):
        CMU = 1;
        Id = 1;
        break;
      case (0x1B2):
        CMU = 1;
        Id = 2;
        break;
      case (0x1B3):
        CMU = 1;
        Id = 3;
        break;

      case (0x1B4):
        CMU = 2;
        Id = 0;
        break;
      case (0x1B5):
        CMU = 2;
        Id = 1;
        break;
      case (0x1B6):
        CMU = 2;
        Id = 2;
        break;
      case (0x1B7):
        CMU = 2;
        Id = 3;
        break;

      case (0x1B8):
        CMU = 3;
        Id = 0;
        break;
      case (0x1B9):
        CMU = 3;
        Id = 1;
        break;
      case (0x1BA):
        CMU = 3;
        Id = 2;
        break;
      case (0x1BB):
        CMU = 3;
        Id = 3;
        break;

      case (0x1BC):
        CMU = 4;
        Id = 0;
        break;
      case (0x1BD):
        CMU = 4;
        Id = 1;
        break;
      case (0x1BE):
        CMU = 4;
        Id = 2;
        break;
      case (0x1BF):
        CMU = 4;
        Id = 3;
        break;

      case (0x1C0):
        CMU = 5;
        Id = 0;
        break;
      case (0x1C1):
        CMU = 5;
        Id = 1;
        break;
      case (0x1C2):
        CMU = 5;
        Id = 2;
        break;
      case (0x1C3):
        CMU = 5;
        Id = 3;
        break;

      case (0x1C4):
        CMU = 6;
        Id = 0;
        break;
      case (0x1C5):
        CMU = 6;
        Id = 1;
        break;
      case (0x1C6):
        CMU = 6;
        Id = 2;
        break;
      case (0x1C7):
        CMU = 6;
        Id = 3;
        break;

      case (0x1C8):
        CMU = 7;
        Id = 0;
        break;
      case (0x1C9):
        CMU = 7;
        Id = 1;
        break;
      case (0x1CA):
        CMU = 7;
        Id = 2;
        break;
      case (0x1CB):
        CMU = 7;
        Id = 3;
        break;

      case (0x1CC):
        CMU = 8;
        Id = 0;
        break;
      case (0x1CD):
        CMU = 8;
        Id = 1;
        break;
      case (0x1CE):
        CMU = 8;
        Id = 2;
        break;
      case (0x1CF):
        CMU = 8;
        Id = 3;
        break;

      default:
        return;
        break;
    }
    if (CMU > 0 && CMU < 64)
    {
      if (Id < 3)
      {
        if (msg.data[2] != 0xFF && msg.data[5] != 0xFF && msg.data[7] != 0xFF) //Check module is not initializing OR a "spoof module"
        {
          if (debug == 1)
          {
            Serial.println();
            Serial.print(CMU);
            Serial.print(",");
            Serial.print(Id);
            Serial.print(" | ");
          }
          modules[CMU].setExists(true);
          modules[CMU].setReset(true);
          modules[CMU].decodecan(Id, msg);
        }
      }
      else
      {
        if (msg.data[2] != 0xFF) //Check module is not initializing OR a "spoof module"
        {
          if (debug == 1)
          {
            Serial.println();
            Serial.print(CMU);
            Serial.print(",");
            Serial.print(Id);
            Serial.print(" | ");
          }
          modules[CMU].setExists(true);
          modules[CMU].setReset(true);
          modules[CMU].decodecan(Id, msg);
        }
      }
    }
  
}


void BMSModuleManager::getAllVoltTemp()
{
  packVolt = 0.0f;

  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      Logger::debug("");
      Logger::debug("Module %i exists. Reading voltage and temperature values", x);
      Logger::debug("Module voltage: %f", modules[x].getModuleVoltage());
      Logger::debug("Lowest Cell V: %f     Highest Cell V: %f", modules[x].getLowCellV(), modules[x].getHighCellV());
      Logger::debug("Temp1: %f       Temp2: %f", modules[x].getTemperature(0), modules[x].getTemperature(1));
      packVolt += modules[x].getModuleVoltage();
      if (modules[x].getLowTemp() < lowestPackTemp) lowestPackTemp = modules[x].getLowTemp();
      if (modules[x].getHighTemp() > highestPackTemp) highestPackTemp = modules[x].getHighTemp();
    }
  }

  packVolt = packVolt / Pstring;
  if (packVolt > highestPackVolt) highestPackVolt = packVolt;
  if (packVolt < lowestPackVolt) lowestPackVolt = packVolt;

 
}

float BMSModuleManager::getLowCellVolt()
{
  LowCellVolt = 5.0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getLowCellV() <  LowCellVolt)  LowCellVolt = modules[x].getLowCellV();
    }
  }
  return LowCellVolt;
}

float BMSModuleManager::getHighCellVolt()
{
  HighCellVolt = 0.0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getHighCellV() >  HighCellVolt)  HighCellVolt = modules[x].getHighCellV();
    }
  }
  return HighCellVolt;
}

float BMSModuleManager::getPackVoltage()
{
  return packVolt;
}

int BMSModuleManager::getNumModules()
{
  return numFoundModules;
}

float BMSModuleManager::getLowVoltage()
{
  return lowestPackVolt;
}

float BMSModuleManager::getHighVoltage()
{
  return highestPackVolt;
}

void BMSModuleManager::setBatteryID(int id)
{
  batteryID = id;
}

void BMSModuleManager::setPstrings(int Pstrings)
{
  Pstring = Pstrings;
}

void BMSModuleManager::setSensors(int sensor, float Ignore, float VoltDelta)
{
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      modules[x].settempsensor(sensor);
      modules[x].setIgnoreCell(Ignore);
      modules[x].setDelta(VoltDelta);
    }
  }
}

float BMSModuleManager::getAvgTemperature()
{
  float avg = 0.0f;
  lowTemp = 999.0f;
  highTemp = -999.0f;
  int y = 0; //counter for modules below -70 (no sensors connected)
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getAvgTemp() > -70)
      {
        avg += modules[x].getAvgTemp();
        if (modules[x].getHighTemp() > highTemp)
        {
          highTemp = modules[x].getHighTemp();
        }
        if (modules[x].getLowTemp() < lowTemp)
        {
          lowTemp = modules[x].getLowTemp();
        }
      }
      else
      {
        y++;
      }
    }
  }
  avg = avg / (float)(numFoundModules - y);

  return avg;
}

float BMSModuleManager::getHighTemperature()
{
  return highTemp;
}

float BMSModuleManager::getLowTemperature()
{
  return lowTemp;
}

float BMSModuleManager::getAvgCellVolt()
{
  numFoundModules = 0;
  float avg = 0.0f;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getAverageV() > 0)
      {
        avg += modules[x].getAverageV();
        numFoundModules++;
      }
    }
  }
  avg = avg / (float)numFoundModules;

  return avg;
}

void BMSModuleManager::printPackSummary()
{
  uint8_t faults = 0;
  uint8_t alerts = 0;
  uint8_t COV = 0;
  uint8_t CUV = 0;

  Logger::console("");
  Logger::console("");
  Logger::console("");
  Logger::console("Modules: %i  Cells: %i  Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules, seriescells(),
                  getPackVoltage(), getAvgCellVolt(), getAvgTemperature());
  Logger::console("");
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      Logger::console("                               Module #%i", y);

      Logger::console("  Voltage: %fV   (%fV-%fV)     Temperatures: (%fC-%fC)", modules[y].getModuleVoltage(),
                      modules[y].getLowCellV(), modules[y].getHighCellV(), modules[y].getLowTemp(), modules[y].getHighTemp());
      if (faults > 0)
      {
        Logger::console("  MODULE IS FAULTED:");
        if (faults & 1)
        {
          Serial.print("    Overvoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 12; i++)
          {
            if (COV & (1 << i))
            {
              Serial.print(i + 1);
              Serial.print(" ");
            }
          }
          Serial.println();
        }
        if (faults & 2)
        {
          Serial.print("    Undervoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 12; i++)
          {
            if (CUV & (1 << i))
            {
              Serial.print(i + 1);
              Serial.print(" ");
            }
          }
          Serial.println();
        }
        if (faults & 4)
        {
          Logger::console("    CRC error in received packet");
        }
        if (faults & 8)
        {
          Logger::console("    Power on reset has occurred");
        }
        if (faults & 0x10)
        {
          Logger::console("    Test fault active");
        }
        if (faults & 0x20)
        {
          Logger::console("    Internal registers inconsistent");
        }
      }
      if (alerts > 0)
      {
        Logger::console("  MODULE HAS ALERTS:");
        if (alerts & 1)
        {
          Logger::console("    Over temperature on TS1");
        }
        if (alerts & 2)
        {
          Logger::console("    Over temperature on TS2");
        }
        if (alerts & 4)
        {
          Logger::console("    Sleep mode active");
        }
        if (alerts & 8)
        {
          Logger::console("    Thermal shutdown active");
        }
        if (alerts & 0x10)
        {
          Logger::console("    Test Alert");
        }
        if (alerts & 0x20)
        {
          Logger::console("    OTP EPROM Uncorrectable Error");
        }
        if (alerts & 0x40)
        {
          Logger::console("    GROUP3 Regs Invalid");
        }
        if (alerts & 0x80)
        {
          Logger::console("    Address not registered");
        }
      }
      if (faults > 0 || alerts > 0) Serial.println();
    }
  }
}

String BMSModuleManager::htmlPackDetails(int digits)
{

  String ptr = "<!DOCTYPE html> <html>\n";


  int cellNum = 0;


  ptr += "Modules:" + String(numFoundModules);
  ptr += " Cells: " + String(seriescells());
  ptr += " Voltage: "+  String(getPackVoltage());
  ptr += " Avg Cell Voltage: " + String(getAvgCellVolt());
  ptr += " Low Cell Voltage: " + String(LowCellVolt);
  ptr += " High Cell Voltage: " + String(HighCellVolt);
  ptr += " Avg Temp: " + String(getAvgTemperature()) + "&deg;C";
  ptr += "<p><table>";

  int header = 0;
  
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      //faults = modules[y].getFaults();
      //alerts = modules[y].getAlerts();
      //COV = modules[y].getCOVCells();
      //CUV = modules[y].getCUVCells();
     
  
      if (header == 0)
      
      { 
        ptr += "<tr><td></td><td>Voltage</td><td>Drift</td>";
        for (int i = 0; i < 13; i++)
      
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
      ptr += String((modules[y].getHighCellV() - modules[y].getLowCellV())*1000) + "mV</td>";

      
      for (int i = 0; i < 13; i++)
      {
        
        if (modules[y].getCellVoltage(i) > 0) { 
        
        ptr += ("<td>");
        cellNum++;
        //ptr += (cellNum < 10 ? "0" : "") + String(cellNum);
        //ptr += (": ");
        if (modules[y].getLowCellV() == modules[y].getCellVoltage(i)) { ptr += "<u style=\"color:blue\";>"; ptr += String(modules[y].getCellVoltage(i), digits);  ptr += "V</u>";}
        else if (modules[y].getHighCellV() == modules[y].getCellVoltage(i)) { ptr += "<u style=\"color:red\";>"; ptr += String(modules[y].getCellVoltage(i), digits);  ptr += "V</u>";}
        
        else {ptr += String(modules[y].getCellVoltage(i), digits); ptr += "V";}
        
        
         if (bitRead(modules[y].getBalance(), i) == 1) { ptr += ("&darr; ");}  else { ptr += ("&nbsp; ");}
        ptr += "</td>";
        }
      }
     
      if (modules[y].getType() == 1)
      {
        ptr += ("<td>");
        ptr += String(modules[y].getTemperature(0), 2);
        ptr += ("&deg;C,  ");
        ptr += String(modules[y].getTemperature(1), 2);
        ptr += ("&deg;C,  ");
        ptr += String(modules[y].getTemperature(2));
        ptr += ("&deg;C</td>");

      }
      else
      {
        ptr += ("<td>");
        ptr += String(modules[y].getTemperature(0),2);
        ptr += ("&deg;C</td>");
      }
      ptr += "</tr>";
    }
  }




  

   ptr +="</table></body>\n";

   for  (int i = 0; i < 100; i++)
      {
        ptr += lastcans[i] + "<br>";
        }

   
  ptr +="</html>\n";
  return ptr;
  
  }

void BMSModuleManager::printPackDetails(int digits)
{
  //int faults = 0;  int alerts= 0;  int COV = 0;  int CUV = 0;
 

  int cellNum = 0;

  Logger::console("");
  Logger::console("Modules: %i Cells: %i Strings: %i  Voltage: %fV   Avg Cell Voltage: %fV  Low Cell Voltage: %fV   High Cell Voltage: %fV Delta Voltage: %zmV   Avg Temp: %fC ", numFoundModules, seriescells(),
                  Pstring, getPackVoltage(), getAvgCellVolt(), LowCellVolt, HighCellVolt, (HighCellVolt - LowCellVolt) * 1000, getAvgTemperature());
  Logger::console("");
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      //faults = modules[y].getFaults();
      //alerts = modules[y].getAlerts();
      //COV = modules[y].getCOVCells();
      //CUV = modules[y].getCUVCells();

      Serial.print("Module #");
      Serial.printf("%2d", y);
      
      Serial.print(" ");
      Serial.print(modules[y].getModuleVoltage(), digits);
      Serial.print("V");
      for (int i = 0; i < 13; i++)
      {
        
        if (modules[y].getCellVoltage(i) > 0) { 
        
        Serial.print(" Cell");
        cellNum++;
        Serial.printf("%2d", cellNum);
        Serial.print(": ");
        Serial.print(modules[y].getCellVoltage(i), digits);
        Serial.print("V");
        
         if (bitRead(modules[y].getBalance(), i) == 1) {Serial.print("↓ ");}  else {Serial.print("  ");}
        }
      }
     
      if (modules[y].getType() == 1)
      {
        Serial.print(" Temp 1: ");
        Serial.print(modules[y].getTemperature(0));
        Serial.print("C Temp 2: ");
        Serial.print(modules[y].getTemperature(1));
        Serial.print("C Temp 3: ");
        Serial.print(modules[y].getTemperature(2));
        Serial.println("C");

      }
      else
      {
        Serial.print(" Temp 1: ");
        Serial.print(modules[y].getTemperature(0));
        Serial.println("C");
      }
    }
  }
}
