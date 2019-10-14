#include <Arduino.h>
#include <Stc1000p.h>

/*
 * STC1000+, improved firmware and Arduino based firmware uploader for the STC-1000 dual stage thermostat.
 *
 * Copyright 2014 Mats Staffansson
 *
 * STC1000+ is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * STC1000+ is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with STC1000+.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#define COM_READ_EEPROM   0x20
#define COM_WRITE_EEPROM  0xE0
#define COM_READ_TEMP     0x01
#define COM_READ_COOLING  0x02
#define COM_READ_HEATING  0x03
#define COM_ACK           0x9A
#define COM_NACK          0x66

#define EEADR_PROFILE_SETPOINT(profile, stp)  (((profile)*19) + ((stp)<<1))
#define EEADR_PROFILE_DURATION(profile, stp)  (EEADR_PROFILE_SETPOINT(profile, stp) + 1)
#define EEADR_SETTINGS_FIRST                  EEADR_PROFILE_SETPOINT(6, 0)
#define EEADR_SETTINGS(name)                  (EEADR_SETTINGS_FIRST + int(name))
#define EEADR_POWER_ON                        127

enum class Settings {
  SP,
  HY,
  TC,
  SA,
  ST,
  DH,
  CD,
  HD,
  RP,
  RN
};

Stc1000p::Stc1000p(int pin, int input_mode) {
  _pin = pin;
  _input_mode = input_mode;
}

void Stc1000p::_writeBit(unsigned const char data) {
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, HIGH);
  delayMicroseconds(7);
  if(!data){
    pinMode(_pin, _input_mode);
    digitalWrite(_pin, LOW);
  }
  delayMicroseconds(400);
  pinMode(_pin, _input_mode);
  digitalWrite(_pin, LOW);
  delayMicroseconds(100);
}

unsigned char Stc1000p::_readBit() {
  unsigned char data;

  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, HIGH);
  delayMicroseconds(7);
  pinMode(_pin, _input_mode);
  digitalWrite(_pin, LOW);
  delayMicroseconds(200);
  data = digitalRead(_pin);
  delayMicroseconds(300);

  return data;
}

void Stc1000p::_writeByte(unsigned const char data) {
  unsigned char i;
  
  for(i=0;i<8;i++){
    _writeBit(((data << i) & 0x80));
  }
  delayMicroseconds(500);
}

unsigned char Stc1000p::_readByte() {
  unsigned char i, data;
  
  for(i=0;i<8;i++){
    data <<= 1;
    if(_readBit()){
      data |= 1;
    }
  }
  delayMicroseconds(500);

  return data;
}

bool Stc1000p::writeEeprom(const unsigned char address, const uint16_t value) {
  unsigned char ack;
  _writeByte(COM_WRITE_EEPROM);
  _writeByte(address);
  _writeByte(((unsigned char)(value >> 8)));
  _writeByte((unsigned char)value);
  _writeByte(COM_WRITE_EEPROM ^ address ^ ((unsigned char)(value >> 8)) ^ ((unsigned char)value));
  delay(6); // Longer delay needed here for EEPROM write to finish, but must be shorter than 10ms
  ack = _readByte();
  return ack == COM_ACK; 
}

bool Stc1000p::writeEepromFloat(const unsigned char address, const float value) {
  return writeEeprom(address, int(value*10));
}

bool Stc1000p::writeEepromBool(const unsigned char address, const bool value) {
  return writeEeprom(address, value);
}

bool Stc1000p::readEeprom(const unsigned char address, int *value) {
  unsigned char xorsum;
  unsigned char ack;
  uint16_t data;

  _writeByte(COM_READ_EEPROM);
  _writeByte(address);
  data = _readByte();
  data = (data << 8) | _readByte();
  xorsum = _readByte();
  ack = _readByte();
  if(ack == COM_ACK && xorsum == (COM_READ_EEPROM ^ address ^ ((unsigned char)(data >> 8)) ^ ((unsigned char)data))) {
    *value = (int16_t)data;
    return true;
  }
  return false;
}

bool Stc1000p::readEepromFloat(const unsigned char address, float *value) {
  int data;
  if( !readEeprom(address, &data) ) {
    return false;
  }
  *value = float(data)/10;
  return true;
}

bool Stc1000p::readEepromBool(const unsigned char address, bool *value) {
  int data;
  if( !readEeprom(address, &data) ) {
    return false;
  }
  *value = bool(data);
  return true;
}

bool Stc1000p::readCommand(const unsigned char command, int *value) {
  unsigned char xorsum;
  unsigned char ack;
  unsigned int data;

  _writeByte(command);
  data = _readByte();
  data = (data << 8) | _readByte();
  xorsum = _readByte();
  ack = _readByte();
  
  if(ack == COM_ACK && xorsum == (command ^ ((unsigned char)(data >> 8)) ^ ((unsigned char)data))) {
    *value = (int)data;
    return true;
  }
  return false;
}

bool Stc1000p::readCommandFloat(const unsigned char command, float *value) {
  int data;
  if( !readCommand(command, &data) ) {
    return false;
  }
  *value = float(data)/10;
  return true;
}

bool Stc1000p::readCommandBool(const unsigned char command, bool *value) {
  int data;
  if( !readCommand(command, &data) ) {
    return false;
  }
  *value = bool(data);
  return true;
}

bool Stc1000p::readTemperature(float *value) {
  return readCommandFloat(COM_READ_TEMP, value);
}

bool Stc1000p::readHeating(bool *value) {
  return readCommandBool(COM_READ_HEATING, value);
}

bool Stc1000p::readCooling(bool *value) {
  return readCommandBool(COM_READ_COOLING, value);
}

bool Stc1000p::readSetpoint(float *value) {
  return readEepromFloat(EEADR_SETTINGS(Settings::SP), value);
}

bool Stc1000p::readHysteresis(float *value) {
  return readEepromFloat(EEADR_SETTINGS(Settings::HY), value);
}

bool Stc1000p::readTemperatureCorrection(float *value) {
  return readEepromFloat(EEADR_SETTINGS(Settings::TC), value);
}

bool Stc1000p::readSetpointAlarm(float *value) {
  return readEepromFloat(EEADR_SETTINGS(Settings::SA), value);
}

bool Stc1000p::readCurrentStep(int *value) {
  return readEeprom(EEADR_SETTINGS(Settings::ST), value);
}

bool Stc1000p::readCurrentDuration(int *value) {
  return readEeprom(EEADR_SETTINGS(Settings::DH), value);
}

bool Stc1000p::readCoolingDelay(int *value) {
  return readEeprom(EEADR_SETTINGS(Settings::CD), value);
}

bool Stc1000p::readHeatingDelay(int *value) {
  return readEeprom(EEADR_SETTINGS(Settings::HD), value);
}

bool Stc1000p::readRamping(bool *value) {
  return readEepromBool(EEADR_SETTINGS(Settings::RP), value);
}

bool Stc1000p::readRunMode(Stc1000pRunMode *value) {
  int data;
  if( !readEeprom(EEADR_SETTINGS(Settings::RN), &data) ) {
    return false;
  }
  
  switch(data) {
    case 0:
      *value = Stc1000pRunMode::PR0;
      break;
    case 1:
      *value = Stc1000pRunMode::PR1;
      break;
    case 2:
      *value = Stc1000pRunMode::PR2;
      break;
    case 3:
      *value = Stc1000pRunMode::PR3;
      break;
    case 4:
      *value = Stc1000pRunMode::PR4;
      break;
    case 5:
      *value = Stc1000pRunMode::PR5;
      break;
    case 6:
      *value = Stc1000pRunMode::TH;
      break;
    default:
      return false;
  }
  
  return true;
}

bool Stc1000p::readProfileSetpoint(int profile, int step, float *value) {
  return readEepromFloat(EEADR_PROFILE_SETPOINT(profile, step), value);
}

bool Stc1000p::readProfileDuration(int profile, int step, int *value) {
  return readEeprom(EEADR_PROFILE_DURATION(profile, step), value);
}

bool Stc1000p::writeSetpoint(float value) {
  return writeEepromFloat(EEADR_SETTINGS(Settings::SP), value);
}

bool Stc1000p::writeHysteresis(float value) {
  return writeEepromFloat(EEADR_SETTINGS(Settings::HY), value);
}

bool Stc1000p::writeTemperatureCorrection(float value) {
  return writeEepromFloat(EEADR_SETTINGS(Settings::TC), value);
}

bool Stc1000p::writeSetpointAlarm(float value) {
  return writeEepromFloat(EEADR_SETTINGS(Settings::SA), value);
}

bool Stc1000p::writeCurrentStep(int value) {
  return writeEeprom(EEADR_SETTINGS(Settings::ST), value);
}

bool Stc1000p::writeCurrentDuration(int value) {
  return writeEeprom(EEADR_SETTINGS(Settings::DH), value);
}

bool Stc1000p::writeCoolingDelay(int value) {
  return writeEeprom(EEADR_SETTINGS(Settings::CD), value);
}

bool Stc1000p::writeHeatingDelay(int value) {
  return writeEeprom(EEADR_SETTINGS(Settings::HD), value);
}

bool Stc1000p::writeRamping(bool value) {
  return writeEepromBool(EEADR_SETTINGS(Settings::RP), value);
}

bool Stc1000p::writeRunMode(Stc1000pRunMode value) {
  int data = 0;
  
  switch(value) {
    case Stc1000pRunMode::PR0:
      data = 0;
      break;
    case Stc1000pRunMode::PR1:
      data = 1;
      break;
    case Stc1000pRunMode::PR2:
      data = 2;
      break;
    case Stc1000pRunMode::PR3:
      data = 3;
      break;
    case Stc1000pRunMode::PR4:
      data = 4;
      break;
    case Stc1000pRunMode::PR5:
      data = 5;
      break;
    case Stc1000pRunMode::TH:
      data = 6;
      break;
    default:
      return false;
  }
  
  if( value != Stc1000pRunMode::TH ) {
    float sp;
    int dh;
    if( !readProfileSetpoint(data, 0, &sp) ) {
      return false;
    }
    if( !writeSetpoint(sp) ) {
      return false;
    }
    if( !readProfileDuration(data, 0, &dh) ) {
      return false;
    }
    if( !writeCurrentDuration(dh) ) {
      return false;
    }
    if( !writeCurrentStep(0) ) {
      return false;
    }
  }

  return writeEeprom(EEADR_SETTINGS(Settings::RN), data);
}

bool Stc1000p::writeProfileSetpoint(int profile, int step, float value) {
  return writeEepromFloat(EEADR_PROFILE_SETPOINT(profile, step), value);
}

bool Stc1000p::writeProfileDuration(int profile, int step, int value) {
  return writeEeprom(EEADR_PROFILE_DURATION(profile, step), value);
}

bool Stc1000p::reboot() {
  bool res = true;
  res = res && writeEepromBool(EEADR_POWER_ON, false);
  res = res && writeEepromBool(EEADR_POWER_ON, true);
  return res;
}

