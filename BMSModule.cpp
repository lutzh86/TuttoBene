#include "BMSModule.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#define MAX_MODULE_ADDR 0x3E
#define IGNORE_CELL_LOW 0.5
#define IGNORE_CELL_HIGH 5.0




BMSModule::BMSModule() {
  for (int i = 0; i < 14; i++) {
    cellVolt[i] = 0.0f;

  }
  balance = 0;
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  temperatures[1] = 0.0f;
  temperatures[2] = 0.0f;

  balstat = 0;
  exists = false;
  reset = false;
  
  type = 1;
}

void BMSModule::clearmodule() {
  for (int i = 0; i < 14; i++) {
    cellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  temperatures[1] = 0.0f;
  temperatures[2] = 0.0f;
  balstat = 0;
  exists = false;
  reset = false;
}

void BMSModule::decodetemp(twai_message_t &msg, int y) {
 if (y == 1) // 0x00 in byte 2 means its an MEB message
{
  type = 1;
  if (msg.data[7] == 0xFD && msg.data[2] != 0xFD)
  {
    temperatures[0] = (msg.data[2] * 0.5) - 40;
  }
  else if (msg.data[0] < 0xDF)
  {
    temperatures[0] = (msg.data[0] * 0.5) - 43;
    balstat = msg.data[2] + (msg.data[3] << 8);
    temperatures[1] = (msg.data[4] < 0xF0) ? ((msg.data[4] * 0.5) - 43) : 0;
    temperatures[2] = (msg.data[5] < 0xF0) ? ((msg.data[5] * 0.5) - 43) : 0;
  }
  else
  {
    temperatures[0] = (msg.data[3] * 0.5) - 43;
    temperatures[1] = (msg.data[4] < 0xF0) ? ((msg.data[4] * 0.5) - 43) : 0;
    temperatures[2] = (msg.data[5] < 0xF0) ? ((msg.data[5] * 0.5) - 43) : 0;
  }
}
else
{
  uint16_t temp_val = ((msg.data[5] & 0x0F) << 4) | ((msg.data[4] & 0xF0) >> 4);
  switch (y)
  {
    case 2:
      type = 2;
      temperatures[0] = (temp_val * 0.5) - 40; // MEB Bits 36-44
      break;
    case 3:
      type = 3;
      temperatures[1] = (temp_val * 0.5) - 40; // MEB Bits 36-44
      break;
  }
}
}


void BMSModule::decodecan(int Id, twai_message_t &msg) {

  static const int idToIndex[] = {0, 4, 8, 12};
  int index = idToIndex[Id];

  cellVolt[index++] = (uint16_t(msg.data[1] >> 4) + uint16_t(msg.data[2] << 4) + 1000) * 0.001;
  cellVolt[index++] = (msg.data[3] + uint16_t((msg.data[4] & 0x0F) << 8) + 1000) * 0.001;
  
  if (Id != 3) {
    cellVolt[index++] = (uint16_t(msg.data[5] << 4) + uint16_t(msg.data[4] >> 4) + 1000) * 0.001;
    cellVolt[index] = (msg.data[6] + uint16_t((msg.data[7] & 0x0F) << 8) + 1000) * 0.001;
 }

  

 
}




uint16_t BMSModule::getBalance() { return balance; }


float BMSModule::getCellVoltage(int cell) {
  if (cell < 0 || cell > 13)
    return 0.0f;
  return cellVolt[cell];
}

float BMSModule::getLowCellV() {
  float lowVal = 10.0f;
  for (int i = 0; i < 14; i++)
    if (cellVolt[i] < lowVal && cellVolt[i] > IGNORE_CELL_LOW)
      lowVal = cellVolt[i];
  return lowVal;
}

float BMSModule::getHighCellV() {
  float hiVal = 0.0f;
  for (int i = 0; i < 14; i++)
    if (cellVolt[i] > IGNORE_CELL_LOW && cellVolt[i] < IGNORE_CELL_HIGH) {
      if (cellVolt[i] > hiVal)
        hiVal = cellVolt[i];
    }
  return hiVal;
}

float BMSModule::getAverageV() {
  int x = 0;
  float avgVal = 0.0f;
  for (int i = 0; i < 14; i++) {
    if (cellVolt[i] > IGNORE_CELL_LOW && cellVolt[i] < IGNORE_CELL_HIGH) {
      x++;
      avgVal += cellVolt[i];
    }
  }

  scells = x;
  avgVal /= x;

  if (scells == 0) {
    avgVal = 0;
  }

  return avgVal;
}

int BMSModule::getscells() { return scells; }


float BMSModule::getLowTemp() {
  if (type == 1) {
    if (sensor == 0) {
      if (getAvgTemp() > 0.5) {
        if (temperatures[0] > 0.5) {
          if (temperatures[0] < temperatures[1] &&
              temperatures[0] < temperatures[2]) {
            return (temperatures[0]);
          }
        }
        if (temperatures[1] > 0.5) {
          if (temperatures[1] < temperatures[0] &&
              temperatures[1] < temperatures[2]) {
            return (temperatures[1]);
          }
        }
        if (temperatures[2] > 0.5) {
          if (temperatures[2] < temperatures[1] &&
              temperatures[2] < temperatures[0]) {
            return (temperatures[2]);
          }
        }
      }
    } else {
      return temperatures[sensor - 1];
    }
  } else {
    return temperatures[0];
  }
  return -88;
}

float BMSModule::getHighTemp() {
  if (type == 1) {
    if (sensor == 0) {
      return (temperatures[0] < temperatures[1]) ? temperatures[1]
                                                 : temperatures[0];
    } else {
      return temperatures[sensor - 1];
    }
  } else {
    return temperatures[0];
  }
  return -88;
}

float BMSModule::getAvgTemp() {
  if (type == 1) {
    if (sensor == 0) {
      if ((temperatures[0] + temperatures[1] + temperatures[2]) / 3.0f > 0.5) {
        if (temperatures[0] > 0.5 && temperatures[1] > 0.5 &&
            temperatures[2] > 0.5) {
          return (temperatures[0] + temperatures[1] + temperatures[2]) / 3.0f;
        }
        if (temperatures[0] < 0.5 && temperatures[1] > 0.5 &&
            temperatures[2] > 0.5) {
          return (temperatures[1] + temperatures[2]) / 2.0f;
        }
        if (temperatures[0] > 0.5 && temperatures[1] < 0.5 &&
            temperatures[2] > 0.5) {
          return (temperatures[0] + temperatures[2]) / 2.0f;
        }
        if (temperatures[0] > 0.5 && temperatures[1] > 0.5 &&
            temperatures[2] < 0.5) {
          return (temperatures[0] + temperatures[1]) / 2.0f;
        }
        if (temperatures[0] > 0.5 && temperatures[1] < 0.5 &&
            temperatures[2] < 0.5) {
          return (temperatures[0]);
        }
        if (temperatures[0] < 0.5 && temperatures[1] > 0.5 &&
            temperatures[2] < 0.5) {
          return (temperatures[1]);
        }
        if (temperatures[0] < 0.5 && temperatures[1] < 0.5 &&
            temperatures[2] > 0.5) {
          return (temperatures[2]);
        }
        if (temperatures[0] < 0.5 && temperatures[1] < 0.5 &&
            temperatures[2] < 0.5) {
          return (-80);
        }
      }
    } else {
      return temperatures[sensor - 1];
    }
  } else {
    return temperatures[0];
  }
  return -88;
}

float BMSModule::getModuleVoltage() {
  moduleVolt = 0;
  for (int i = 0; i < 14; i++) {
    if (cellVolt[i] > IGNORE_CELL_LOW && cellVolt[i] < IGNORE_CELL_HIGH) {
      moduleVolt = moduleVolt + cellVolt[i];
    }
  }
  return moduleVolt;
}

float BMSModule::getTemperature(int temp) {
  if (temp < 0 || temp > 2)
    return 0.0f;
  return temperatures[temp];
}

int BMSModule::getType() { return type; }
bool BMSModule::isExisting() { return exists; }
bool BMSModule::isReset() { return reset; }
void BMSModule::setExists(bool ex) { exists = ex; }
void BMSModule::setReset(bool ex) { reset = ex; }
void BMSModule::setBalance(uint16_t value) { balance = value; }
