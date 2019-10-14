#ifndef Stc1000p_h
#define Stc1000p_h

#include <Arduino.h>

enum class Stc1000pRunMode {
  TH,
  PR0,
  PR1,
  PR2,
  PR3,
  PR4,
  PR5
};

class Stc1000p
{
  public:
    Stc1000p(int pin, int input_mode = INPUT);

    bool writeEeprom(const unsigned char address, const uint16_t value);
    bool writeEepromFloat(const unsigned char address, const float value);
    bool writeEepromBool(const unsigned char address, const bool value);
    bool readEeprom(const unsigned char address, int *value);
    bool readEepromFloat(const unsigned char address, float *value);
    bool readEepromBool(const unsigned char address, bool *value);
    bool readCommand(unsigned char command, int *value);
    bool readCommandFloat(unsigned char command, float *value);
    bool readCommandBool(unsigned char command, bool *value);

    bool readTemperature(float *value);
    bool readHeating(bool *value);
    bool readCooling(bool *value);
    bool readSetpoint(float *value);
    bool readHysteresis(float *value);
    bool readTemperatureCorrection(float *value);
    bool readSetpointAlarm(float *value);
    bool readCurrentStep(int *value);
    bool readCurrentDuration(int *value);
    bool readCoolingDelay(int *value);
    bool readHeatingDelay(int *value);
    bool readRamping(bool *value);
    bool readRunMode(Stc1000pRunMode *value);
    bool readProfileSetpoint(int program, int step, float *value);
    bool readProfileDuration(int program, int step, int *value);

    bool writeSetpoint(float value);
    bool writeHysteresis(float value);
    bool writeTemperatureCorrection(float value);
    bool writeSetpointAlarm(float value);
    bool writeCurrentStep(int value);
    bool writeCurrentDuration(int value);
    bool writeCoolingDelay(int value);
    bool writeHeatingDelay(int value);
    bool writeRamping(bool value);
    bool writeRunMode(Stc1000pRunMode value);
    bool writeProfileSetpoint(int program, int step, float value);
    bool writeProfileDuration(int program, int step, int value);
    
    bool reboot();

  private:
    int _pin;
    int _input_mode;

    void _writeBit(unsigned const char data);
    unsigned char _readBit();
    void _writeByte(unsigned const char data);
    unsigned char _readByte();
};

#endif
