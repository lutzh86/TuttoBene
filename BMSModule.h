#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"

class BMSModule {
public:
  BMSModule();
  void decodecan(int Id, twai_message_t &msg);
  void decodetemp(twai_message_t &msg, int y);
  void clearmodule();
  int getscells();
  float getCellVoltage(int cell);
  uint16_t getBalance();

  float getLowCellV();
  float getHighCellV();
  float getAverageV();
  float getLowTemp();
  float getHighTemp();
  float getAvgTemp();
  float getModuleVoltage();
  float getTemperature(int temp);
  uint8_t getFaults();
  uint8_t getAlerts();
  uint8_t getCOVCells();
  uint8_t getCUVCells();
  int getType();
  int getBalStat();
  bool isExisting();
  bool isReset();
  void copy();
  void setBalance(uint16_t balance);
  void setReset(bool ex);
  void setExists(bool ex);
  void settempsensor(int tempsensor);
  void setDelta(float ex);

private:
  float cellVolt[14]; // calculated as 16 bit value * 6.250 / 16383 = volts
  float moduleVolt;      // calculated as 16 bit value * 33.333 / 16383 = volts
  float temperatures[3]; // Don't know the proper scaling at this point
  float lowestTemperature;
  float highestTemperature;
  float VoltDelta;
  bool exists;
  bool reset;
  int alerts;
  int faults;
  int COVFaults;
  int CUVFaults;
  int sensor;
  uint16_t balance;
  int scells;
  uint32_t balstat;
  uint32_t lasterror;
  uint8_t cmuerror;
  uint32_t timeout;
  int type;
};
