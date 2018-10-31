/*
        ADS1256.h - Arduino Library for communication with Texas Instrument
        ADS1256 ADC
        Modified by Zhenyi Ye
*/

#include "ADS1256.h"


ADS1256::ADS1256(float clockspdMhz, float vref, bool useResetPin) {
  // Set DRDY as input
  pinMode(PINDEX_DRDY, INPUT);
  // Set CS as output
  pinMode(PINDEX_CS, OUTPUT);

  if (useResetPin) {
    // set RESETPIN as output
    pinMode(PINDEX_RESET, OUTPUT);
    // pull RESETPIN high
    digitalWrite(PINDEX_RESET, HIGH);
  }

  // Voltage Reference
  _VREF = vref;

  // Default conversion factor
  _conversionFactor = 1.0;

  // Default _pga
  _pga = 1;

  // Initilize DRDY_state
  DRDY_state = HIGH;

  // Start SPI on a quarter of ADC clock speed
  SPI.begin();
  SPI.beginTransaction(
      SPISettings(clockspdMhz * 1000000 / 4, MSBFIRST, SPI_MODE1));
  
}


void ADS1256::writeRegister(unsigned char reg, unsigned char wdata) {
    
  waitDRDY();
  CSON();
  delayMicroseconds(10);
  SPI.transfer(WREG | reg);
  SPI.transfer(0x0);
  SPI.transfer(wdata);
  delayMicroseconds(10);        // t11 delay (4*tCLKIN) after WREG command,
                                // 16Mhz avr clock is approximately twice
                                // faster that 7.68 Mhz ADS1256 master clock
  CSOFF();

  if (wdata != readRegister(reg)) {   //Check if write was succesfull
    Serial.print("Write to Register 0x");
    Serial.print(reg, HEX);
    Serial.println(" failed!");
  }
  else {
    Serial.println("success");
  }
}


unsigned char ADS1256::readRegister(unsigned char reg) {
  unsigned char readValue;
  CSON();
  delayMicroseconds(10);
  SPI.transfer(RREG | reg);
  SPI.transfer(0x0);
  delayMicroseconds(10);  // t6 delay (50*tCLKIN), 16Mhz avr clock is
                                      // approximately twice faster that 7.68 Mhz
                                      // ADS1256 master clock
  readValue = SPI.transfer(NOP);
  delayMicroseconds(10);  // t11 delay
  CSOFF();

  return readValue;
}


void ADS1256::sendCommand(unsigned char reg) {
  waitDRDY();
  CSON();
  delayMicroseconds(10);
  SPI.transfer(reg);
  delayMicroseconds(10);  // t11
  CSOFF();
}


void ADS1256::setConversionFactor(float val) { _conversionFactor = val; }


void ADS1256::readTest() {
  unsigned char _highByte, _midByte, _lowByte;
  CSON();
  SPI.transfer(RDATA);
  delayMicroseconds(10);  // t6 delay

  _highByte = SPI.transfer(WAKEUP);
  _midByte = SPI.transfer(WAKEUP);
  _lowByte = SPI.transfer(WAKEUP);

  CSOFF();
}


float ADS1256::readChannel() {
  
  attachInterrupt(PINDEX_DRDY, DRDY_Interrupt, FALLING);

  unsigned char _highByte, _midByte, _lowByte;
  long value;

  waitDRDY();
  CSON();
  delayMicroseconds(10);
  SPI.transfer(RDATA);
  delayMicroseconds(7);  // t6 delay
  _highByte = SPI.transfer(NOP);
  _midByte = SPI.transfer(NOP);
  _lowByte = SPI.transfer(NOP);
  CSOFF();

  detachInterrupt(PINDEX_DRDY);
  
  value = ((long)_highByte << 16) + ((long)_midByte << 8) + ((long)_lowByte);
  if (value & 0x00800000) {
    value |= 0xff000000;
  }

  return (((float)value / 0x7FFFFF) * ((2 * _VREF) / (float)_pga)) *
         _conversionFactor;
}

float ADS1256::readChannel(int avg) {
  float sum_data = 0.0;
  
  for(int i=0; i<avg; i++){
    sum_data += readChannel();
  }

  return sum_data / avg;
}

// Channel switching for single ended mode. Negative input channel are
// automatically set to AINCOM
void ADS1256::setChannel(byte channel) { setChannel(channel, -1); }

// Channel Switching for differential mode. Use -1 to set input channel to
// AINCOM
void ADS1256::setChannel(byte AIN_P, byte AIN_N) {
  unsigned char MUX_CHANNEL;
  unsigned char MUXP;
  unsigned char MUXN;

  switch (AIN_P) {
    case 0:
      MUXP = ADS1256_MUXP_AIN0;
      break;
    case 1:
      MUXP = ADS1256_MUXP_AIN1;
      break;
    case 2:
      MUXP = ADS1256_MUXP_AIN2;
      break;
    case 3:
      MUXP = ADS1256_MUXP_AIN3;
      break;
    case 4:
      MUXP = ADS1256_MUXP_AIN4;
      break;
    case 5:
      MUXP = ADS1256_MUXP_AIN5;
      break;
    case 6:
      MUXP = ADS1256_MUXP_AIN6;
      break;
    case 7:
      MUXP = ADS1256_MUXP_AIN7;
      break;
    default:
      MUXP = ADS1256_MUXP_AINCOM;
  }

  switch (AIN_N) {
    case 0:
      MUXN = ADS1256_MUXN_AIN0;
      break;
    case 1:
      MUXN = ADS1256_MUXN_AIN1;
      break;
    case 2:
      MUXN = ADS1256_MUXN_AIN2;
      break;
    case 3:
      MUXN = ADS1256_MUXN_AIN3;
      break;
    case 4:
      MUXN = ADS1256_MUXN_AIN4;
      break;
    case 5:
      MUXN = ADS1256_MUXN_AIN5;
      break;
    case 6:
      MUXN = ADS1256_MUXN_AIN6;
      break;
    case 7:
      MUXN = ADS1256_MUXN_AIN7;
      break;
    default:
      MUXN = ADS1256_MUXN_AINCOM;
  }

  MUX_CHANNEL = MUXP | MUXN;

  attachInterrupt(PINDEX_DRDY, DRDY_Interrupt, FALLING);

  writeRegister(MUX, MUX_CHANNEL);
  //now we need to sync
  //need to delay by 4x SPI clock = 2.35 uS (t1)
  //to be safe 5 uS
  delayMicroseconds(5);
  sendCommand(SYNC);

  //again delay by t1
  delayMicroseconds(5);
  sendCommand(WAKEUP);
  delayMicroseconds(1);

  detachInterrupt(PINDEX_DRDY);

}

void ADS1256::begin(unsigned char drate, unsigned char gain, bool buffenable) {
  attachInterrupt(PINDEX_DRDY, DRDY_Interrupt, FALLING);
  _pga = 1 << gain;
  sendCommand(
      SDATAC);  // send out SDATAC command to stop continous reading mode.
  writeRegister(DRATE, drate);  // write data rate register
  uint8_t bytemask = B00000111;
  uint8_t adcon = readRegister(ADCON);
  uint8_t byte2send = (adcon & ~bytemask) | gain;
  writeRegister(ADCON, byte2send);
  if (buffenable) {
    uint8_t status = readRegister(STAT);
    bitSet(status, 1);
    writeRegister(STAT, status);
  }
  sendCommand(SELFCAL);  // perform self calibration
  waitDRDY();  // wait ADS1256 to settle after self calibration
  delay(100);
  // attachInterrupt to data_ready pin
  detachInterrupt(PINDEX_DRDY);
}


inline void ADS1256::CSON() {
  GPOC = (1 << PINDEX_CS);
}  // digitalWrite(_CS, LOW); }

inline void ADS1256::CSOFF() {
  GPOS = (1 << PINDEX_CS);
}  // digitalWrite(_CS, HIGH); }

void ADS1256::waitDRDY() {
  while (DRDY_state) {
    continue;
  }
  noInterrupts();
  DRDY_state = HIGH;
  interrupts();
}

//Interrupt function
void DRDY_Interrupt() {
  DRDY_state = LOW;
}