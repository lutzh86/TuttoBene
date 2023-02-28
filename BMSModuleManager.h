
#include "Arduino.h"
#include "BMSModule.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#define MAX_MODULE_ADDR 0x3E


typedef struct {  
  float OverVSetpoint;
  float UnderVSetpoint;
  float ChargeVsetpoint;
  float DischVsetpoint; 
  float OverTSetpoint;
  float UnderTSetpoint;
  float DisTSetpoint;  
  float CellGap;  
  float balanceVoltage;
  float balanceHyst;  
  int series_cells;
  int parallel_strings;
  int capacity;
  int DischargeTaper;
  int chargecurrentmax;
  int discurrentmax; 
  int nominal_cell_voltage; 
  int internal_resistance; //in mOhm
  int ocv[100];
} EEPROMSettings;



class BMSModuleManager {
public:
  BMSModuleManager();
  int seriescells();
  void clearmodules();
  void sendcommand();
  void decodecan(twai_message_t &msg, int debug);
  void decodetemp(twai_message_t &msg, int debug, int type);
  void getAllVoltTemp();
  void setPstrings(int Pstrings);
  void setUnderVolt(float newVal);
  void setOverVolt(float newVal);
  void setOverTemp(float newVal);
  void setBalanceV(float newVal);
  void setBalanceHyst(float newVal);
  void balanceCells(int debug, int onoff);
  float getPackVoltage();
  float getAvgTemperature();
  float getHighTemperature();
  float getLowTemperature();
  float getAvgCellVolt();
  float getLowCellVolt();
  float getHighCellVolt();
  float getHighVoltage();
  float getLowVoltage();
  void printPackSummary();
  void printPackDetails(int digits);
  String htmlPackDetails(int digits);
  int getNumModules();
  bool checkcomms();

private:
  float packVolt; 
  int Pstring;
  float LowCellVolt;
  float HighCellVolt;
  float highTemp;
  float lowTemp;
  float BalHys;
  BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as
                                          // we've configured for.

  int numFoundModules; // The number of modules that seem to exist
  bool isFaulted;
  bool balancing;
  uint8_t balcnt;
  int spack;

};
