#include "esp_timer.h"
#include <vector>

//GLOBAL CONSTANTS
const uint8_t RX_PIN = 32;
const float WITHIN_RANGE = 0.1;


//GLOBAL VARIABLES
uint8_t rxFrame[262];
bool rxFrameReadyFlag = false;


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
enum syncClk {NoSync = 0, Bit1 = 1, Bit2 = 2, Bit3 = 3, Bit4 = 4, Bit5 = 5, Bit6 = 6, Bit7 = 7, Bit8 = 8, Bit9 = 9, LowVoltageInSync = 10, HighVoltageInSync = 11, LowVoltageHalf = 12, HighVoltageHalf = 13};
uint8_t syncClkState = NoSync;
int64_t noSyncPeriodAvgBuf[7];
int64_t halfPeriod;
enum lastSymbol {HalfPeriod, Period, Error};
uint16_t currentBitPos;


void readFrame();
const int lenghtFrame = 17;
//uint8_t rxFrame[lenghtFrame] = {85,126,0,5,1,1,1,1,0,0,0,0,0,0,0,0,0}; 

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
  enum lastSymbol currentLastSymbol;

  //Sync clock on preambule
  switch (syncClkState)
  {
    case NoSync:
      syncClkState = rxErrorHandling(pinVoltageState);
      break;
    case Bit1:
      if(pinVoltageState){
        noSyncPeriodAvgBuf[0] = TimeSinceLastTransitionInUS;
        syncClkState = Bit2;
      }
      else{
        syncClkState = rxErrorHandling(pinVoltageState);
      }
      break;
    case Bit2:
      syncClkState = nextSyncClkState(TimeSinceLastTransitionInUS, pinVoltageState, Bit2);
      break;
    case Bit3:
      syncClkState = nextSyncClkState(TimeSinceLastTransitionInUS, pinVoltageState, Bit3);
      break;
    case Bit4:
      syncClkState = nextSyncClkState(TimeSinceLastTransitionInUS, pinVoltageState, Bit4);
      break;
    case Bit5:
      syncClkState = nextSyncClkState(TimeSinceLastTransitionInUS, pinVoltageState, Bit5);
      break;
    case Bit6:
      syncClkState = nextSyncClkState(TimeSinceLastTransitionInUS, pinVoltageState, Bit6);
      break;
    case Bit7:
      syncClkState = nextSyncClkState(TimeSinceLastTransitionInUS, pinVoltageState, Bit7);
      break;
    case Bit8:
      syncClkState = nextSyncClkState(TimeSinceLastTransitionInUS, pinVoltageState, Bit8);
      break;
    case Bit9:
      halfPeriod = bufferAverage((uint8_t)Bit9) / 2;
      rxFrame[0] = 0b01010101;
      currentBitPos = 9;

      currentLastSymbol = getLastSymbol(TimeSinceLastTransitionInUS, pinVoltageState, true);
      if(currentLastSymbol == HalfPeriod){
        syncClkState = HighVoltageHalf;
      }
      else if(currentLastSymbol == Period){
        if(addBitAndCheckEndOfFrame(false)){
          syncClkState = rxErrorHandling(pinVoltageState);
        }
        else{
          syncClkState = HighVoltageInSync;
        }
      }
      else{
        syncClkState = rxErrorHandling(pinVoltageState);
      }
      break;
    case LowVoltageInSync:
      currentLastSymbol = getLastSymbol(TimeSinceLastTransitionInUS, pinVoltageState, true);
      if(currentLastSymbol == HalfPeriod){
        syncClkState = HighVoltageHalf;
      }
      else if(currentLastSymbol == Period){
        if(addBitAndCheckEndOfFrame(false)){
          syncClkState = rxErrorHandling(pinVoltageState);
        }
        else{
          syncClkState = HighVoltageInSync;
        }
      }
      else{
        syncClkState = rxErrorHandling(pinVoltageState);
      }
      break;
    case HighVoltageInSync:
      currentLastSymbol = getLastSymbol(TimeSinceLastTransitionInUS, pinVoltageState, false);
      if(currentLastSymbol == HalfPeriod){
        syncClkState = LowVoltageHalf;
      }
      else if(currentLastSymbol == Period){
        if(addBitAndCheckEndOfFrame(true)){
          syncClkState = rxErrorHandling(pinVoltageState);
        }
        else{
          syncClkState = LowVoltageInSync;
        }
      }
      else{
        syncClkState = rxErrorHandling(pinVoltageState);
      }
      break;
    case LowVoltageHalf:
      currentLastSymbol = getLastSymbol(TimeSinceLastTransitionInUS, pinVoltageState, true);
      if(currentLastSymbol == HalfPeriod){
        if(addBitAndCheckEndOfFrame(false)){
          syncClkState = rxErrorHandling(pinVoltageState);
        }
        else{
          syncClkState = HighVoltageInSync;
        }
      }
      else if(currentLastSymbol == Period){
        syncClkState = rxErrorHandling(pinVoltageState);
      }
      else{
        syncClkState = rxErrorHandling(pinVoltageState);
      }
      break;
    case HighVoltageHalf:
      currentLastSymbol = getLastSymbol(TimeSinceLastTransitionInUS, pinVoltageState, false);
      if(currentLastSymbol == HalfPeriod){
        if(addBitAndCheckEndOfFrame(true)){
          syncClkState = rxErrorHandling(pinVoltageState);
        }
        else{
          syncClkState = LowVoltageInSync;
        }
      }
      else if(currentLastSymbol == Period){
        syncClkState = rxErrorHandling(pinVoltageState);
      }
      else{
        syncClkState = rxErrorHandling(pinVoltageState);
      }
      break;
  }
}

uint8_t nextSyncClkState(int64_t TimeSinceLastTransition, bool currentVoltage, uint8_t BitNumber){
  bool expectedVoltage;
  if(BitNumber == 2 || BitNumber == 4 || BitNumber == 6 || BitNumber == 8){
    expectedVoltage = false;
  }
  else{
    expectedVoltage = true;
  }

  if(expectedVoltage){
    if(currentVoltage){
      if(withinAverageRange(TimeSinceLastTransition, BitNumber)){
        noSyncPeriodAvgBuf[BitNumber-1] = TimeSinceLastTransition;
        return BitNumber+1;
      }
      else{
        return NoSync;
      }
    }
    else{
      return Bit1;
    }
  }
  else{
    if(currentVoltage){
      return NoSync;
    }
    else{
      if(withinAverageRange(TimeSinceLastTransition, BitNumber)){
        noSyncPeriodAvgBuf[BitNumber-1] = TimeSinceLastTransition;
        return BitNumber+1;
      }
      else{
        return Bit1;
      }
    }
  }
}

uint8_t rxErrorHandling(bool currentVoltage){
  if(currentVoltage){
    return NoSync;
  }
  else{
    return Bit1;
  }
}

bool withinAverageRange(int64_t comparedInt, uint8_t BitNumber){
  int64_t average = bufferAverage(BitNumber);
  return withinRange(average, comparedInt);
}

bool withinRange(int64_t selectedInt, int64_t comparedInt){
  int64_t range = selectedInt * WITHIN_RANGE;
  int64_t selectedPlusRange = selectedInt + range;
  int64_t selectedMinusRange = selectedInt - range;

  if(comparedInt >= selectedMinusRange && comparedInt <= selectedPlusRange){
    return true;
  }
  else{
    return false;
  }
}

int64_t bufferAverage(uint8_t BitNumber){
  int64_t average = 0;
  for(uint8_t i=0; i < BitNumber; i++){
    average += noSyncPeriodAvgBuf[i];
  }
  return average / BitNumber;
}

enum lastSymbol getLastSymbol(int64_t TimeSinceLastTransition, bool currentVoltage, bool expectedVoltage){
  int64_t period = halfPeriod * 2;

  if(currentVoltage != expectedVoltage){
    return Error;
  }

  if(withinRange(halfPeriod, TimeSinceLastTransition)){
    return HalfPeriod;
  }
  else if(withinRange(period, TimeSinceLastTransition)){
    return Period;
  }
  else{
    return Error;
  }
}

bool addBitAndCheckEndOfFrame(bool bitValue){
  uint16_t bytePos = (currentBitPos - 1) / 8;
  uint8_t bitByteOffset = (currentBitPos - 1) % 8;

  if(bitValue){
    rxFrame[bytePos] |= 1 << bitByteOffset;
  }
  else{
    rxFrame[bytePos] &= ~(1 << bitByteOffset);
  }

  currentBitPos++;

  if(currentBitPos > 262*8){
    return true;
  }
  bytePos = (currentBitPos - 1) / 8;
  bitByteOffset = (currentBitPos - 1) % 8;
  if(currentBitPos == 0 && rxFrame[bytePos-1] == 0b01111110 && bytePos > 3){
    if(bytePos == rxFrame[3] + 7){
      rxFrameReadyFlag = true;
      return true;
    }
  }
  return false;
}