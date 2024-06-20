#include "esp_timer.h"
#include <vector>

//GLOBAL CONSTANTS
const uint8_t RX_PIN = 32;
const float WITHIN_AVERAGE_RANGE = 0.1;


//GLOBAL VARIABLES
uint8_t rxFrame[262];


//TOTS VARIABLES
enum class State {
  IDLE,
  PREAMBLE,
  START,
  HEADER_TYPE,
  HEADER_LENGTH,
  DATA,
  CONTROL,
  END
};

State currentState = State::IDLE;
State nextState; 


//rxPinChanged() ISR VARIABLES
int64_t lastRXTransitionTime = 0;

enum syncClk {NoSync, Bit1, Bit2, Bit3, Bit4, Bit5, Bit6, Bit7, Bit8};
enum syncClk syncClkState = NoSync;
int64_t noSyncPeriodAvgBuf[7];


void readFrame();
const int lenghtFrame = 17;
uint8_t rxFrame[lenghtFrame] = {85,126,0,5,1,1,1,1,0,0,0,0,0,0,0,0,0}; 

void readFrame() {
    uint8_t lenghtData;
    std::vector<uint8_t> data;
    for (int i = 0; i < lenghtFrame; i++) {

        switch (currentState) {
            case State::IDLE:
                // Serial.println("State: IDLE");
                i = 2;
                nextState = State::HEADER_LENGTH;
                break;

            case State::PREAMBLE:
                // Serial.println("State: PREAMBLE");
                // Add relevant logic for PREAMBLE
                nextState = State::START; // Example transition
                break;

            case State::START:
                // Serial.println("State: START");
                // Add relevant logic for START
                nextState = State::HEADER_TYPE; // Example transition
                break;

            case State::HEADER_TYPE:
                // Serial.println("State: HEADER_TYPE");
                // Add relevant logic for HEADER_TYPE
                nextState = State::HEADER_LENGTH; // Example transition
                break;

            case State::HEADER_LENGTH:
                // Serial.println("State: HEADER_LENGTH");
                lenghtData = rxFrame[i];
                nextState = State::DATA;
                break;

            case State::DATA:
                // Serial.println("State: DATA");
                if (i < 4 + lenghtData - 1) {
                    data.push_back(rxFrame[i]);
                    nextState = State::DATA;
                } else {
                    data.push_back(rxFrame[i]);
                    nextState = State::CONTROL;
                }
                break;

            case State::CONTROL:
                // Serial.println("State: CONTROL");
                // do stuff here
                nextState = State::END; // Example transition
                break;

            case State::END:
                // Serial.println("State: END");
                Serial.println(i);
                i = lenghtFrame; // Ending the loop
                nextState = State::IDLE;
                break;
        }
        currentState = nextState;
    }
    printVector(data);
}

void printVector(std::vector< uint8_t > data) {
  Serial.printf("<");
  for(int i = 0; i < data.size(); i++) {
    Serial.printf("%d,", data[i]);
  }
  Serial.println(">");
}



void setup() {
  Serial.begin(115200);
  pinMode(RX_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(RX_PIN), rxPinChanged, CHANGE);
}

void loop() {
  readFrame();
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
