#include "esp_timer.h"

//GLOBAL CONSTANTS
const uint8_t RX_PIN = 32;
const float WITHIN_AVERAGE_RANGE = 0.1;


//GLOBAL VARIABLES
uint8_t rxFrame[262];


//rxPinChanged() ISR VARIABLES
int64_t lastRXTransitionTime = 0;

enum syncClk {NoSync, Bit1, Bit2, Bit3, Bit4, Bit5, Bit6, Bit7, Bit8};
enum syncClk syncClkState = NoSync;
int64_t noSyncPeriodAvgBuf[7];

void setup() {
  // put your setup code here, to run once:
  pinMode(RX_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(RX_PIN), rxPinChanged, CHANGE);
}

void loop() {
}

void rxPinChanged() {
  //Calculate Time since last transition
  int64_t CurrentTime = esp_timer_get_time();
  int64_t TimeSinceLastTransitionInUS = CurrentTime - lastRXTransitionTime;
  lastRXTransitionTime = CurrentTime;

  //Get current pin voltage state
  bool pinVoltageState = digitalRead(RX_PIN);

  //Sync clock on preambule
  switch (syncClkState)
  {
    case NoSync:
      if(pinVoltageState == true){
        syncClkState = NoSync;
      }
      else{
        syncClkState = Bit1;
      }
      break;
    case Bit1:
      if(pinVoltageState == true){
        noSyncPeriodAvgBuf[0] = TimeSinceLastTransitionInUS;
        syncClkState = Bit2;
      }
      else{
        syncClkState = Bit1;
      }
      break;
    case Bit2:
      if(pinVoltageState == true){
        syncClkState = NoSync;
      }
      else{
        if(withinAverageRange(TimeSinceLastTransitionInUS, 1)){
          noSyncPeriodAvgBuf[1] = TimeSinceLastTransitionInUS;
          syncClkState = Bit3;
        }
        else{
          syncClkState = Bit1;
        }
      }
      break;
    case Bit3:
      if(pinVoltageState == true){
        if(withinAverageRange(TimeSinceLastTransitionInUS, 2)){
          noSyncPeriodAvgBuf[2] = TimeSinceLastTransitionInUS;
          syncClkState = Bit4;
        }
        else{
          syncClkState = NoSync;
        }
      }
      else{
        syncClkState = Bit1;
      }
      break;
    case Bit4:
      if(pinVoltageState == true){
        syncClkState = NoSync;
      }
      else{
        if(withinAverageRange(TimeSinceLastTransitionInUS, 2)){
          noSyncPeriodAvgBuf[2] = TimeSinceLastTransitionInUS;
          syncClkState = Bit4;
        }
        else{
          syncClkState = NoSync;
        }
      }
      break;
    case Bit5:
      
    case Bit6:
      
    case Bit7:
      
    case Bit8:
      
  }
}

bool withinAverageRange(int64_t comparedInt, uint8_t BitNumber){
  int64_t average = 0
  for(uint8_t i=0; i < BitNumber; i++){
    average += noSyncPeriodAvgBuf[i];
  }
  average = average / BitNumber;

  int64_t range = average * WITHIN_AVERAGE_RANGE;
  int64_t averagePlusRange = average + range;
  int64_t averageMinusRange = average - range;

  if(comparedInt >= averageMinusRange && comparedInt <= averagePlusRange){
    return true;
  }
  else{
    return false;
  }
}
