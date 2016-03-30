#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial sSerialGPS(11, 12);

Adafruit_GPS GPS(&sSerialGPS);
#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean);
void altDetect(float oldAlt, float alt, int altBreakPoint[]);

String message = "";
boolean sent = false;
int altBreakPoint[5] = {0, 0, 0, 0, 0};
float gpsAlt[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
int offset = 0;
int abpOffset = 0; // for finding the altBreakPoint
float oldAlt, alt;
boolean falling = false;
boolean forOldAlt = true;
boolean checkAlt = false;
char sampleCode = '\0';
boolean sampleStatus = false;
char rbStatus = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Wire.begin();
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  
  delay(1000);
  sSerialGPS.println(PMTK_Q_RELEASE);
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop() {
  int t = millis();

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if(GPS.fix) {
    Serial.println("Fix Found");

    if((millis() / 1000) % 30 == 0) {
      Wire.beginTransmission(8);
      message += GPS.latitudeDegrees;
      Wire.write(message.c_str());
      Wire.write(' ');
      message.remove(0); // clear message
      message += GPS.longitudeDegrees;
      Wire.write(message.c_str());
      Wire.write(' ');
      message.remove(0); // clear message
      message += GPS.altitude;
      Wire.write(message.c_str());
      Wire.write(' ');
      message.remove(0); // clear message
      Wire.write(falling);
      Wire.write(' ');
      Wire.write(sampleCode);
      Wire.write(' ');
      Wire.write(sampleStatus);
      Wire.write('\0');
      Wire.endTransmission();
    }

    // for Altitude Average
    if((millis() / 1000) % 2) {
      if(offset < 5) {
        gpsAlt[offset] = GPS.altitude;
        offset++;
      } else { // i.e. offset is 5
        float total = 0;
        for(int i = 0; i < 5; i++) {
          total += gpsAlt[i];
        }
        float average = total / 5.0;

        if(!falling) {
          // get ready to call altDetect
          if(forOldAlt) {
            oldAlt = average;
            forOldAlt = false;
          } else { //i.e. for alt
            alt = average;
            forOldAlt = true;
            checkAlt = true;
          }
  
          // call altDetect
          if(checkAlt) {
            altDetect(oldAlt, alt, altBreakPoint);
            checkAlt = false;
          }
        } else {
          if(average < altBreakPoint[abpOffset]) {
            Wire.beginTransmission(9);
            char sampleCode = '0' + abpOffset;
            Wire.write(sampleCode);
            Wire.endTransmission();
            abpOffset++;
          }
        }
        
      }
    }
    
  } else {
    
  }

  Wire.requestFrom(8, 1);
  while(Wire.available()) {
    rbStatus = Wire.read();
  }
  Wire.requestFrom(9, 2);
  while(Wire.available()) {
    sampleCode = Wire.read();
    sampleStatus = Wire.read();
  }

  Serial.print(rbStatus);
  Serial.print(sampleCode);
  Serial.println(sampleStatus);
  t = millis() - t;
  Serial.print("Loop Time: ");
  Serial.print(t);
  Serial.println("ms");
  delay(1000);
  digitalWrite(LED_BUILTIN, (digitalRead(LED_BUILTIN) == HIGH)? LOW : HIGH);
}

void altDetect (float oldAlt, float alt, int altBreakPoint[]){
  int stepSize = 0;
  int deltaAlt = alt - oldAlt;
  if (deltaAlt < 0) {
    stepSize = alt/5;
    for(int y = 0; y < 5; y++){
      altBreakPoint[5 - y] = {alt - ((y + 1) * stepSize)}; // stores break points in decsending order
    }
    falling = true; 
  }  
}    
