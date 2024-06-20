#include "esp_timer.h"
#include <vector>
#include "CRC16.h"

//GLOBAL CONSTANTS
const int RX_PIN = 13;
const int TX_PIN = 33;
const float WITHIN_RANGE = 0.1;
bool receivedBit = false;
bool checkPeriod = false;
unsigned long lastChangeTime = 0;
unsigned long sendTaskTime = 0;

const int HALF_PERIOD = 500;            // en micro
const int THRESHOLD_PERIOD = 280;       // en micro
const int TIME_BETWEEN_BITS = 5000;     // en micro
const TickType_t xDelay = TIME_BETWEEN_BITS / 1000 * portTICK_PERIOD_MS; // en millis

//GLOBAL VARIABLES
uint8_t* rxFrame;
bool rxFrameReadyFlag = false;

std::vector<uint8_t> receivedBytes;    // Vector to store received bytes
std::vector<int> buffer;               // Buffer to accumulate received bits

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

void receivePulse();
void readFrame();
std::vector<uint8_t> createFrame();
const int lenghtFrame = 17;
//uint8_t rxFrame[lenghtFrame] = {85,126,0,5,1,1,1,1,0,0,0,0,0,0,0,0,0}; 

void readFrame() {
    Serial.println("readFrame");
    uint8_t lenghtData;
    CRC16 crc;
    std::vector<uint8_t> data;
    uint16_t crcVal;
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
                if (i == 4 + lenghtData - 1){
                  crcVal = rxFrame[i] << 8;
                  nextState = State::CONTROL;
                  break;
                }
                else if(i == 4 + lenghtData){
                  crcVal |= rxFrame[i] & 0b11111111;

                  for(int j = 0; j < lenghtData; j++){
                  crc.add(data[j]);
                  }
                  if(crc.calc() == crcVal){
                    currentState = State::END;
                  }
                  else {
                    Serial.println("CRC ERROR");
                    nextState = State::END;
                    break;
                  }
                }
                Serial.println("error line 95 and 100");
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

std::vector<uint8_t> createFrame(uint8_t *payload, uint8_t size){
  // uint8_t payload[8] = {1,1,1,1,1,1,1,1};
  CRC16 crc;
  crc.add(payload, size);
  uint16_t crcVal = crc.calc();

  std::vector<uint8_t> frame{};
  frame.push_back(0b01010101); //preable
  frame.push_back(0b01111110); //start
  frame.push_back(0b00000000); //type/flag
  frame.push_back(size); //size
  for(int i = 0; i < size; i++){
    frame.push_back(payload[i]);
  }
  frame.push_back((crcVal >> 8) & 0b11111111);  // CRC
  frame.push_back(crcVal & 0b11111111);         // CRC
  frame.push_back(0b01111110);
  return frame;
}

void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  

  // attachInterrupt(digitalPinToInterrupt(RX_PIN), rxPinChanged, CHANGE);

  attachInterrupt(digitalPinToInterrupt(RX_PIN), receivePulse, CHANGE);
  
  xTaskCreate(send, "Send Trame", 2048, NULL, 2,  NULL);

  xTaskCreate(receive, "receive Trame", 2048, NULL, 3,  NULL);
}

void loop() {
}

void IRAM_ATTR receivePulse() {
  // Serial.println("interupt");
  receivedBit = true;
}

void send(void *pvParameters){
  Serial.println("send");
  const uint8_t size = 4;
  uint8_t payloadArray[size] = {
    0b11011101, // 221
    0b11011111, // 223
    0b00011101, // 29
    0b00000111, // 7
  };
  
  std::vector<uint8_t> message1 = createFrame(payloadArray, size);   // success
  // Serial.println("message : ");
  // printVector(message1);
  sendTaskTime = micros();
  for(int i = 0; i < message1.size(); ++i) {
    sendByte(message1[i]);
  }

  for (;;) {
    // Serial.println("loop");
    vTaskDelay(xDelay);
  }
}

void sendByte(uint8_t bits) {
  int val;
  for(int i = 7; i >= 0; i--) {
    val = (bits >> i) & 0x01;
    sendPulse(val);
    vTaskDelay(xDelay);
  }
}

void sendPulse(int value) { // 0, 1
  digitalWrite(TX_PIN, value);
  delayMicroseconds(HALF_PERIOD);

  digitalWrite(TX_PIN, !value);
  delayMicroseconds(HALF_PERIOD);
}

void receive(void *pvParameters) {
    Serial.println("taskreceive");
    int periodElapse;
    int rxVal;
    buffer.push_back(0);
    for (;;) {
        auto currentTime = micros();
        periodElapse = currentTime - lastChangeTime;

        if (!checkPeriod && (periodElapse >= HALF_PERIOD * 2 + TIME_BETWEEN_BITS - THRESHOLD_PERIOD) &&
            (periodElapse <= HALF_PERIOD * 2 + TIME_BETWEEN_BITS + THRESHOLD_PERIOD)) {
            checkPeriod = true;
        }

        if (receivedBit) {
            // Serial.println(buffer.size());

            receivedBit = false;
            rxVal = digitalRead(RX_PIN);

            if (buffer.size() != 0) {
                if (checkPeriod) {
                    if (periodElapse >= HALF_PERIOD - THRESHOLD_PERIOD) {
                        buffer.push_back(!rxVal); // Add inverted rxVal to buffer
                    }
                } else {
                    checkPeriod = true;
                }
            } else { // First bit
                buffer.push_back(0); // Add first bit to buffer
            }
            lastChangeTime = currentTime;
        }
        // Check if a complete byte (8 bits) has been accumulated
        if (buffer.size() >= 11*8) {
            processBuffer();
            buffer.clear();
        }
            // uint8_t currentByte = 0;
            // for (int i = 0; i < 8; ++i) {
            //     currentByte |= buffer[i] << (7 - i);
            // }
            // receivedBytes.push_back(currentByte);
            // printVector(receivedBytes);
            // buffer.clear(); // Clear buffer after forming a byte
        

        // if (receivedBytes.size() == 11){
        //   rxFrame = receivedBytes.data();
        //   readFrame();
        //   receivedBytes.clear();
        // }

        vTaskDelay(1); // Delay before checking again
    }
}

void processBuffer() {
    // Assuming each byte is 8 bits
    // Convert bits in buffer to bytes
    for (size_t i = 0; i < buffer.size(); i += 8) {
        uint8_t byte = 0;
        for (int j = 0; j < 8; ++j) {
            byte |= buffer[i + j] << (7 - j);
           
        }
      receivedBytes.push_back(byte);
    }
  printVector(receivedBytes);
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
      else syncClkState = rxErrorHandling(pinVoltageState);
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
      halfPeriod = bufferAverage(Bit9) / 2;
      rxFrame[0] = 0b01010101;
      currentBitPos = 9;

      currentLastSymbol = getLastSymbol(TimeSinceLastTransitionInUS, pinVoltageState, true);
      if(currentLastSymbol == HalfPeriod)   syncClkState = HighVoltageHalf;
      else if(currentLastSymbol == Period)  syncClkState = addBitAndCheckEndOfFrame(false) ? rxErrorHandling(pinVoltageState) : HighVoltageInSync;
      else                                  syncClkState = rxErrorHandling(pinVoltageState);
      break;
    case LowVoltageInSync:
      currentLastSymbol = getLastSymbol(TimeSinceLastTransitionInUS, pinVoltageState, true);
      if(currentLastSymbol == HalfPeriod)   syncClkState = HighVoltageHalf;
      else if(currentLastSymbol == Period)  syncClkState = addBitAndCheckEndOfFrame(false) ? rxErrorHandling(pinVoltageState) : HighVoltageInSync;
      else                                  syncClkState = rxErrorHandling(pinVoltageState);
      break;
    case HighVoltageInSync:
      currentLastSymbol = getLastSymbol(TimeSinceLastTransitionInUS, pinVoltageState, false);
      if(currentLastSymbol == HalfPeriod)   syncClkState = LowVoltageHalf;
      else if(currentLastSymbol == Period)  syncClkState = addBitAndCheckEndOfFrame(true) ? rxErrorHandling(pinVoltageState) : LowVoltageInSync;
      else                                  syncClkState = rxErrorHandling(pinVoltageState);
      break;
    case LowVoltageHalf:
      currentLastSymbol = getLastSymbol(TimeSinceLastTransitionInUS, pinVoltageState, true);
      if(currentLastSymbol == HalfPeriod)   syncClkState = addBitAndCheckEndOfFrame(false) ? rxErrorHandling(pinVoltageState) : HighVoltageInSync;
      else                                  syncClkState = rxErrorHandling(pinVoltageState);
      break;
    case HighVoltageHalf:
      currentLastSymbol = getLastSymbol(TimeSinceLastTransitionInUS, pinVoltageState, false);
      if(currentLastSymbol == HalfPeriod)   syncClkState = addBitAndCheckEndOfFrame(true) ? rxErrorHandling(pinVoltageState) : LowVoltageInSync;
      else                                  syncClkState = rxErrorHandling(pinVoltageState);
      break;
  }
}

uint8_t nextSyncClkState(int64_t TimeSinceLastTransition, bool currentVoltage, uint8_t BitNumber){
  bool expectedVoltage;
  if(BitNumber == 2 || BitNumber == 4 || BitNumber == 6 || BitNumber == 8)  expectedVoltage = false;
  else                                                                      expectedVoltage = true;

  if(currentVoltage == expectedVoltage && withinRange(bufferAverage(BitNumber), TimeSinceLastTransition)){
    noSyncPeriodAvgBuf[BitNumber-1] = TimeSinceLastTransition;
    return BitNumber+1;
  }
  else return rxErrorHandling(currentVoltage);
}

uint8_t rxErrorHandling(bool currentVoltage){
  if(currentVoltage)  return NoSync;
  else                return Bit1;
}

//Check if comparedInt is within range of selectedInt
bool withinRange(int64_t selectedInt, int64_t comparedInt){
  int64_t range = selectedInt * WITHIN_RANGE;
  int64_t selectedPlusRange = selectedInt + range;
  int64_t selectedMinusRange = selectedInt - range;

  if(comparedInt >= selectedMinusRange && comparedInt <= selectedPlusRange) return true;
  else                                                                      return false;
}

//Calculate average from buffer from 0 to BitNumber elements
int64_t bufferAverage(uint8_t BitNumber){
  int64_t average = 0;
  for(uint8_t i=0; i < BitNumber; i++){
    average += noSyncPeriodAvgBuf[i];
  }
  return average / BitNumber;
}

enum lastSymbol getLastSymbol(int64_t TimeSinceLastTransition, bool currentVoltage, bool expectedVoltage){
  int64_t period = halfPeriod * 2;

  //Detect missed transition and error out
  if(currentVoltage != expectedVoltage) return Error;

  //Detect last symbol (half-period or period)
  if(withinRange(halfPeriod, TimeSinceLastTransition))  return HalfPeriod;
  else if(withinRange(period, TimeSinceLastTransition)) return Period;
  else                                                  return Error;
}

bool addBitAndCheckEndOfFrame(bool bitValue){
  //Calculate bit position
  uint16_t bytePos = (currentBitPos - 1) / 8;
  uint8_t bitByteOffset = (currentBitPos - 1) % 8;

  //Add bit to Frame buffer
  if(bitValue)  rxFrame[bytePos] |= 1 << bitByteOffset;
  else          rxFrame[bytePos] &= ~(1 << bitByteOffset);

  //Update bit position
  currentBitPos++;

  //Recalculate bit position
  bytePos = (currentBitPos - 1) / 8;
  bitByteOffset = (currentBitPos - 1) % 8;

  //Detect bad frame (too long)
  if(currentBitPos > 262*8) return true;

  //Check if end of valid frame
  if(currentBitPos == 0 && rxFrame[bytePos-1] == 0b01111110 && bytePos > 3){
    if(bytePos == rxFrame[3] + 7){
      rxFrameReadyFlag = true;
      return true;
    }
  }
  return false;
}