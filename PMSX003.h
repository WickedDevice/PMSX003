#ifndef __WICKED_DEVICE_PMSX003__
#define __WICKED_DEVICE_PMSX003__

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <stdint.h>

class PMSX003 {
public:
  PMSX003(HardwareSerial * serial);
  PMSX003(SoftwareSerial * swserial);

  boolean begin(void);                                           // puts sensor into passive mode
  boolean getSample(float * pm1p0, float * pm2p5, float * pm10); // returns only data fields 4, 5, and 6
  boolean getFields(uint16_t * buf);                             // returns 13 data field values into buf
  boolean getRaw(uint8_t * buf);                                 // returns 32 bytes into buf

private:
  uint8_t response[32]; // temporary storage for response

  HardwareSerial * hwserial;
  SoftwareSerial * swserial;

  void resetSerial();
  void endSerial();
  uint8_t read();
  void write(uint8_t * value, int size);
  void write(uint8_t value);

  boolean pmsx003ValidResponse(uint8_t * response);
  uint16_t pmsx003GetValue(uint8_t * response, uint8_t field_index);
  boolean pmsx003ConsumeResponse(uint8_t * response);
  boolean pmsx003SendRequest(uint8_t * request);
  void clearPMSX003SerialInput(void);
};

#endif
