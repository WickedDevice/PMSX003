#include "PMSX003.h"

#define PMSX003_DELAY_BETWEEN_WRITE_BURSTS (80)

PMSX003::PMSX003(HardwareSerial * hwserial){
  this->swserial = NULL;
  this->hwserial = hwserial;
}

PMSX003::PMSX003(SoftwareSerial * swserial){
  this->swserial = swserial;
  this->hwserial = NULL;
}

void PMSX003::setSerial(HardwareSerial * serial) {
  this->swserial = NULL;
  this->hwserial = serial;
}

void PMSX003::setSerial(SoftwareSerial * serial) {
  this->swserial = serial;
  this->hwserial = NULL;
}

boolean PMSX003::begin(void){
  const uint8_t init_message[] = { 0x42, 0x4d, 0xe1, 0x00, 0x00, 0x01, 0x70 }; // precompute checksum

  resetSerial();

  for(uint8_t ii = 0; ii < 7; ii++){
    write(init_message[ii]);
  }

  if(pmsx003ConsumeResponse(response, 8)){
    // there's a complete response at least
    if(pmsx003ValidResponse(response, 8)){
      // Serial.println("Valid Response");   
      // there's a valid response, we have a winner
      endSerial();
      delay(PMSX003_DELAY_BETWEEN_WRITE_BURSTS);
      return true;
    }
    else {
      // Serial.println("Invalid Checksum");
    }
  }
  else {
    // Serial.println("Timeout");
  }
  // Serial.println("Invalid Response");
  // for(uint8_t jj = 0; jj < 8; jj++){
  //   Serial.print(jj);
  //   Serial.print(": 0x");
  //   if(response[jj] < 16){
  //     Serial.print("0");
  //   }
  //   Serial.println(response[jj], HEX);
  // }
  endSerial();
  delay(PMSX003_DELAY_BETWEEN_WRITE_BURSTS);
  return false;
}

void PMSX003::clearPMSX003SerialInput(void){
  Stream * s = swserial ? (Stream *) swserial : (Stream *) hwserial;
  if(s){
    while(s->available()){
      s->read();
    }
  }
}

void PMSX003::resetSerial(){
  if(swserial){
    swserial->begin(9600);
  }
  else if(hwserial){
    hwserial->begin(9600);
  }

  clearPMSX003SerialInput();

  memset(response, 0, 32);
}

void PMSX003::endSerial(){
  if(swserial){
    swserial->end();
  }
  else if(hwserial){
    hwserial->end();
  }
}

uint8_t PMSX003::read(){
  if(swserial){
    return swserial->read();
  }
  else if(hwserial){
    return hwserial->read();
  }
}

void PMSX003::write(uint8_t value){
  if(swserial){
    swserial->write(value);
  }
  else if(hwserial){
    hwserial->write(value);
  }
}

void PMSX003::write(uint8_t * value, int size){
  if(swserial){
    swserial->write(value, size);
  }
  else if(hwserial){
    hwserial->write(value, size);
  }
}

boolean PMSX003::getRaw(uint8_t * buff){
  uint8_t readPMSX003[] = { 0x42, 0x4d, 0xe2, 0x00, 0x00, 0x01, 0x71 };  // precompute checksum

  resetSerial();


  for(uint8_t ii = 0; ii < 7; ii++){
    if(pmsx003SendRequest(readPMSX003)){
      // there is the beginning of a response at least
      if(pmsx003ConsumeResponse(response)){
        // there's a complete response at least
        if(pmsx003ValidResponse(response)){
          // there's a valid response, we have a winner
          if(buff != NULL){
            memcpy(buff, response, 32);
          }

          endSerial();

          delay(PMSX003_DELAY_BETWEEN_WRITE_BURSTS);
          return true;
        }
      }
    }

    // attempt to re-synchronize
    // Serial.println(F("Info: Attempting to resynchronize with PMSX003 sensor"));
    for(uint8_t jj = 0; jj <= ii; jj++){
      // send out a variable number of padding bytes in order to brute-force resynchronize the sensor
      // eventually the request message should be received by the sensor correctly
      write(0xff);
    }

  }

  // Serial.print(F("Warning: Failed to collect PMSX003 Data"));
  endSerial();
  delay(PMSX003_DELAY_BETWEEN_WRITE_BURSTS);
  return false;
}

boolean PMSX003::getSample(float * pm1p0, float * pm2p5, float * pm10){
  uint8_t numWrites = 0;
  if(getRaw(NULL)){ // has a side effect of loading response buffer
    if(pm1p0 != NULL){
      *pm1p0 = pmsx003GetValue(response, 4);
      numWrites++;
    }
    if(pm2p5 != NULL){
      *pm2p5 = pmsx003GetValue(response, 5);
      numWrites++;
    }
    if(pm10 != NULL){
      *pm10 = pmsx003GetValue(response, 6);
      numWrites++;
    }

    return (numWrites > 0);
  }

  return false;
}

boolean PMSX003::getFields(uint16_t * buf){
  if(buf == NULL){
    return false;
  }

  if(getRaw(NULL)){ // has a side effect of loading response buffer
    for(uint8_t ii = 1; ii <= 13; ii++){
      *buf++ = pmsx003GetValue(response, ii);
    }
    return true;
  }

  return false;
}

// sends the request command
// returns true the sensor starts responding within 50ms
boolean PMSX003::pmsx003SendRequest(uint8_t * request){
  // before sending the command, take some steps
  // to ensure that that the input buffer is empty
  Stream * s = swserial ? (Stream *) swserial : (Stream *) hwserial;

  delay(50);
  clearPMSX003SerialInput();

  write(request, 7);
  unsigned long start = millis();
  const long timeout_interval = 100;

  while(1){
    unsigned long currentMillis = millis();

    if(s->available()){
      // exit with true as soon as you get some bytes back
      return true;
    }

    if(currentMillis - start >= timeout_interval) {
      // exit with false if you get a timeout
      // Serial.println("TIMEOUT #1");
      return false;
    }
  }

  return false;
}

boolean PMSX003::pmsx003ConsumeResponse(uint8_t * response){
  return pmsx003ConsumeResponse(response, 32);
}

boolean PMSX003::pmsx003ConsumeResponse(uint8_t * response, uint8_t length){
  // expect a response that starts with 0x42
  // expect a response that is length bytes long
  // expect a response to complete within 100ms
  uint8_t bytes_received = 0;
  Stream * s = swserial ? (Stream *) swserial : (Stream *) hwserial;
  unsigned long start = millis();
  const long timeout_interval = 100;
  while(bytes_received < length){
    unsigned long currentMillis = millis();
    if(currentMillis - start >= timeout_interval) {
      // Serial.println("TIMEOUT #2");
      // Serial.print(bytes_received);
      // Serial.print(" < ");
      // Serial.println(length);
      return false;
    }

    if(s->available()){
      uint8_t value = read();
      if((bytes_received == 0) && (value != 0x42)){ // reject bytes until synchronized
        continue;
      }
      else{
        if(response){
          response[bytes_received] = value;
        }
        bytes_received++;
      }
    }
  }

  return true;
}

uint16_t PMSX003::pmsx003GetValue(uint8_t * response, uint8_t field_index){
  uint8_t idx = 2 + (2 * field_index); // idx 4 = data field 1 (not zero-based)
  uint8_t high = response[idx];     // high byte for value is 4th byte in packet in the packet
  uint8_t low  = response[idx + 1]; // low byte for value is 5th byte in the packet

  uint16_t ret = high;
  ret <<= 8;   // shift the low byte into the high byte
  ret |= low;  // mask in the low byte
  return ret;  // return the result
}

boolean PMSX003::pmsx003ValidResponse(uint8_t * response){
  return pmsx003ValidResponse(response, 32);
}

boolean PMSX003::pmsx003ValidResponse(uint8_t * response, uint8_t length){
  // sum of the first length - 2 characters should equal the last field_index
  uint16_t sum = 0;
  for(uint8_t ii = 0; ii < length - 2; ii++){
    sum += response[ii];
    // Serial.print("SUM[");
    // Serial.print(ii);
    // Serial.print("]:");
    // Serial.println(sum);
  }

  // calculate the last "field index" based on the length
  // skip two bytes (the header) and divide by 2
  uint8_t field_index = ((length - 2) / 2) - 1;
  uint16_t reportedChecksum = pmsx003GetValue(response, field_index);
  if(reportedChecksum != sum){
    // Serial.println("CHECKSUM ERROR!");
    // Serial.print(reportedChecksum, HEX);
    // Serial.print(" != ");
    // Serial.println(sum, HEX);
    return false;
  }

  return true;
}
