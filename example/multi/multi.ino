#include <SoftwareSerial.h>
#include <PMSX003.h>

int enable_pin = 18; // will be 17 on 3.1 shield (bugfix)

SoftwareSerial pmsx003Serial_2(A7, A5);  // RX, TX
SoftwareSerial pmsx003Serial_1(A4, A5);  // RX, TX
PMSX003 pmsx003_1(&pmsx003Serial_1);
PMSX003 pmsx003_2(&pmsx003Serial_2);

void setup(){
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, HIGH);
  delay(100);

  pmsx003_1.begin();
  pmsx003_2.begin();
}

void loop(){
  uint16_t fields[13] = {0};

  Serial.print(millis());
  Serial.print("\t1\t");
  if(pmsx003_1.getFields(&fields[0])){
    for(uint8_t ii = 0; ii < sizeof(fields) / sizeof(fields[0]); ii++){
      Serial.print(fields[ii]);
      Serial.print("\t");
    }
    Serial.println();
  }
  else{
    Serial.println("Failed to Get Sample From #1");
    digitalWrite(enable_pin, LOW);
    delay(100);
    digitalWrite(enable_pin, HIGH);
    delay(100);
  }

  Serial.print(millis());
  Serial.print("\t2\t");
  memset(fields, 0, 26);
  if(pmsx003_2.getFields(&fields[0])){
    for(uint8_t ii = 0; ii < sizeof(fields) / sizeof(fields[0]); ii++){
      Serial.print(fields[ii]);
      Serial.print("\t");
    }
    Serial.println();
  }
  else{
    Serial.println("Failed to Get Sample From #2");
    digitalWrite(enable_pin, LOW);
    delay(100);
    digitalWrite(enable_pin, HIGH);
    delay(100);
  }

  delay(2000);
}
