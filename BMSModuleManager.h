
#include "Arduino.h"
#include "BMSModule.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#define MAX_MODULE_ADDR 0x3E


typedef struct {
  uint8_t version;
  uint8_t checksum;
  uint32_t canSpeed;
  uint8_t batteryID; 
  uint8_t logLevel;
  float OverVSetpoint;
  float UnderVSetpoint;
  float ChargeVsetpoint;
  float DischVsetpoint;
  float ChargeHys;
  float StoreVsetpoint;
  float WarnOff;
  float OverTSetpoint;
  float UnderTSetpoint;
  float ChargeTSetpoint;
  float DisTSetpoint;
  float WarnToff;
  float CellGap;
  uint8_t IgnoreTemp;
  float IgnoreVolt;
  float balanceVoltage;
  float balanceHyst;
  float DeltaVolt;
  int series_cells;
  int parallel_strings;
  int capacity;
  int chargecurrentmax;
  int chargecurrentend;
  int discurrentmax;
  int invertcur;
  int cursens;
  int voltsoc;
  uint16_t offset1;
  uint16_t offset2;
  int32_t changecur;
  float convhigh;
  float convlow;
  int Pretime;
  int conthold;
  int Precurrent;
  int Serialexp;
  int ESSmode;
  int gaugelow;
  int gaugehigh;
  int ncur;
  int chargertype;
  int chargerspd;
  uint16_t UnderDur;
  uint16_t CurDead;
  float DisTaper;
  bool ChargerDirect;
  uint8_t tripcont;
  uint16_t triptime;
  float DischHys;
  int16_t chargecurrentcold;
  int curcan;
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
  void readSetpoints();
  void setBatteryID(int id);
  void setPstrings(int Pstrings);
  void setUnderVolt(float newVal);
  void setOverVolt(float newVal);
  void setOverTemp(float newVal);
  void setBalanceV(float newVal);
  void setBalanceHyst(float newVal);
  void setSensors(int sensor, float Ignore, float VoltDelta);
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
  /*
    void processCANMsg(CAN_FRAME &frame);
  */
  void printAllCSV(unsigned long timestamp, float current, int SOC);
  void printPackSummary();
  void printPackDetails(int digits);
  String htmlPackDetails(int digits);
  int getNumModules();
  bool checkcomms();

private:
  float packVolt; // All modules added together
  int Pstring;
  float LowCellVolt;
  float HighCellVolt;
  float lowestPackVolt;
  float highestPackVolt;
  float lowestPackTemp;
  float highestPackTemp;
  float highTemp;
  float lowTemp;
  float BalHys;
  BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as
                                          // we've configured for.
  int batteryID;
  int numFoundModules; // The number of modules that seem to exist
  bool isFaulted;
  bool balancing;
  uint8_t balcnt;
  int spack;
  /*
    void sendBatterySummary();
    void sendModuleSummary(int module);
    void sendCellDetails(int module, int cell);
  */
};
