#include <SoftwareSerial.h>
#include <PMSX003.h>

int enable_pin = 18; // will be 17 on 3.1 shield (bugfix)

SoftwareSerial pmsx003Serial(A7, A5);  // RX, TX
// SoftwareSerial pmsx003Serial(A4, A5);  // RX, TX
PMSX003 pmsx003(&pmsx003Serial);

void setup(){
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, HIGH);
  delay(100);

  pmsx003.begin();
}

void loop(){
  uint16_t fields[13] = {0};

  Serial.print(millis());
  Serial.print("\t");
  if(pmsx003.getFields(&fields[0])){
    for(uint8_t ii = 0; ii < sizeof(fields) / sizeof(fields[0]); ii++){
      Serial.print(fields[ii]);
      Serial.print("\t");
    }
    Serial.println();
  }
  else{
    Serial.println("Failed to Get Sample");
    digitalWrite(enable_pin, LOW);
    delay(100);
    digitalWrite(enable_pin, HIGH);
    delay(100);
  }

  delay(2000);
}
