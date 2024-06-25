// Include necessary libraries for timer and vector operations
#include "esp_timer.h"
#include <vector>
#include "CRC16.h"

// Define constants for different case use scenarios
#define NORMAL 0
#define TOOLONG 1
#define WRONGCRC 2
#define MAXSIZE 3
#define NOSTART 4
#define NOEND 5

const int SITUATION = MAXSIZE;

// Global constants for pin configuration and timing
unsigned long start_time;
const int RX_PIN = 13;
const int TX_PIN = 33;
const float WITHIN_RANGE = 0.1;
bool receivedBit = false;
bool checkPeriod = false;
unsigned long lastChangeTime = 0;
unsigned long sendTaskTime = 0;

const int HALF_PERIOD = 500;            // Half period in microseconds
const int THRESHOLD_PERIOD = 280;       // Threshold period in microseconds
const int TIME_BETWEEN_BITS = 5000;     // Time between bits in microseconds
const TickType_t xDelay = TIME_BETWEEN_BITS / 1000 * portTICK_PERIOD_MS; // Time delay in milliseconds

// Global variables for frame reception and buffer handling
bool rxFrameReadyFlag = false;

std::vector<uint8_t> receivedBytes;    // Vector to store received bytes
std::vector<int> buffer;               // Buffer to accumulate received bits

// Define states for the state machine
enum class State {
  IDLE,
  PREAMBLE,
  START,
  HEADER_TYPE,
  HEADER_LENGTH,
  DATA,
  CONTROL,
  END,
  ERROR
};

State currentState = State::PREAMBLE;
State nextState; 

// Variables for ISR and synchronization
int64_t lastRXTransitionTime = 0;
enum syncClk {NoSync = 0, Bit1 = 1, Bit2 = 2, Bit3 = 3, Bit4 = 4, Bit5 = 5, Bit6 = 6, Bit7 = 7, Bit8 = 8, Bit9 = 9, LowVoltageInSync = 10, HighVoltageInSync = 11, LowVoltageHalf = 12, HighVoltageHalf = 13};
uint8_t syncClkState = NoSync;
int64_t noSyncPeriodAvgBuf[7];
int64_t halfPeriod;
enum lastSymbol {HalfPeriod, Period, Error};
uint16_t currentBitPos;

// Function declarations
void receivePulse();
void readFrame();
std::vector<uint8_t> createFrame();
int lenghtFrame;
uint8_t * rxFrame; 

// Function to read the frame
void readFrame() {
    // Uncomment for debugging: printUint8Array(rxFrame, 11);
    uint8_t lenghtData;
    CRC16 crc;
    std::vector<uint8_t> data;
    uint16_t crcVal;
    unsigned long finalTime;
    for (int i = 0; i < lenghtFrame; i++) {
        // Uncomment for debugging: Serial.println(rxFrame[i]);
        switch (currentState) {
            case State::IDLE:
                // Uncomment for debugging: Serial.println("State: IDLE");
                nextState = State::PREAMBLE;
                break;

            case State::PREAMBLE:
                // Uncomment for debugging: Serial.println("State: PREAMBLE");
                // Add relevant logic for PREAMBLE
                nextState = State::START; // Example transition
                break;

            case State::START:
                // Uncomment for debugging: Serial.println("State: Start");
                // Uncomment for debugging: Serial.println(rxFrame[i]);
                if (rxFrame[i] != 126){
                  Serial.println("ERROR NO START");
                  nextState = State::ERROR;
                  break;
                } else {
                  nextState = State::HEADER_TYPE; // Example transition
                  break;
                }

            case State::HEADER_TYPE:
                // Uncomment for debugging: Serial.println("State: HEADER_TYPE");
                // Add relevant logic for HEADER_TYPE
                nextState = State::HEADER_LENGTH; // Example transition
                break;

            case State::HEADER_LENGTH:
                // Uncomment for debugging: Serial.println("State: HEADER_LENGTH");
                lenghtData = rxFrame[i];
                if (lenghtData > 80){
                  Serial.println("ERROR : PAYLOAD TOO LONG");
                  nextState = State::ERROR;
                  break;
                } else {
                  nextState = State::DATA;
                  break;
                }

            case State::DATA:
                // Uncomment for debugging: Serial.println("State: DATA");
                if (i < 4 + lenghtData - 1) {
                    data.push_back(rxFrame[i]);
                    nextState = State::DATA;
                } else {
                    data.push_back(rxFrame[i]);
                    nextState = State::CONTROL;
                }
                break;

            case State::CONTROL:
                // Uncomment for debugging: Serial.println(rxFrame[i]);
                // Uncomment for debugging: Serial.println(i);
                if (i == 4 + lenghtData){
                  crcVal = rxFrame[i] << 8;
                  nextState = State::CONTROL;
                  break;
                } else if (i == 4 + lenghtData + 1){
                  crcVal |= rxFrame[i] & 0b11111111;

                  for(int j = 0; j < lenghtData; j++){
                    crc.add(data[j]);
                  }
                  if (crc.calc() == crcVal){
                    // Uncomment for debugging: Serial.println("CRC GOOD");
                    nextState = State::END;
                    break;
                  } else {
                    Serial.println("CRC ERROR OR PAYLOAD LONGER THAN EXPECTED");
                    nextState = State::ERROR;
                    break;
                  }
                }
                Serial.println("error line 95 and 100");
                break;

            case State::END:
                // Uncomment for debugging: Serial.println(i);
                // Uncomment for debugging: Serial.println(lenghtData);
                finalTime = micros() - start_time;
                if (i == lenghtData + 6){
                  if(rxFrame[i] != 126){
                    Serial.println("ERROR NO END");
                  } else {
                    Serial.println("CORRECT END");
                    Serial.printf(" - time %d - ", finalTime);
                  }
                }
                // Uncomment for debugging: Serial.println("State: END");
                // Uncomment for debugging: Serial.println(i);
                i = lenghtFrame; // Ending the loop
                nextState = State::IDLE;
                break;

            case State::ERROR:
              nextState = State::ERROR;
              break;
        }
        currentState = nextState;
    }
    if (currentState != State::ERROR){
      printVector(data);
    }
}

// Function to print a vector of uint8_t
void printVector(std::vector< uint8_t > data) {
  Serial.printf("<");
  for(int i = 0; i < data.size(); i++) {
    Serial.printf("%d,", data[i]);
  }
  Serial.println(">");
}

// Function to create a frame from payload
std::vector<uint8_t> createFrame(uint8_t *payload, uint8_t size){
  CRC16 crc;
  crc.add(payload, size);
  uint16_t crcVal = crc.calc();
  
  std::vector<uint8_t> frame{};
  frame.push_back(0b01010101); // Preamble
  frame.push_back(0b01111110); // Start
  frame.push_back(0b00000000); // Type/flag
  frame.push_back(size); // Size
  for(int i = 0; i < size; i++){
    frame.push_back(payload[i]);
  }
  frame.push_back((crcVal >> 8) & 0b11111111);  // CRC
  frame.push_back(crcVal & 0b11111111);         // CRC
  frame.push_back(0b01111110); // End
  return frame;
}

// Setup function to initialize serial communication and pin modes
void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);

  // Attach interrupts for pin change detection
  attachInterrupt(digitalPinToInterrupt(RX_PIN), receivePulse, CHANGE);
  
  // Create tasks for sending and receiving frames
  xTaskCreate(send, "Send Trame", 2048, NULL, 2,  NULL);
  xTaskCreate(receive, "receive Trame", 2048, NULL, 3,  NULL);
}

void loop() {
  // Empty loop as tasks handle communication
}

// ISR for receiving pulses
void IRAM_ATTR receivePulse() {
  receivedBit = true;
}

// Convert a vector to a uint8_t array
uint8_t * vector2Array(std::vector<uint8_t> vector){
  uint8_t * temp = vector.data();
  return temp;
}

// Task to send frames
void send(void *pvParameters){
  uint8_t size;
  Serial.println("send");
  std::vector<uint8_t> message1;

  // Switch case for different scenarios
  switch (SITUATION){
    case NORMAL:
    {
      size = 4;
      uint8_t payloadArray[size] = { 42, 56 , 73, 1};
      message1 = createFrame(payloadArray, size);
      break;
    }
    case TOOLONG:
    {
      size = 82;
      uint8_t payloadArray[size];
      for(int i = 0; i < size; i++){
        payloadArray[i] = 44;
      }
      message1 = createFrame(payloadArray, size);
      break;
    }
    case WRONGCRC:
      {
      size = 4;
      uint8_t payloadArray[size] = { 42, 56 , 73, 1};
      message1 = createFrame(payloadArray, size);
      message1[8] += 3;
      message1[9] += 1;
      break;
      }
    case MAXSIZE:
    {
      size = 80;
      uint8_t payloadArray[size];
      for(int i = 0; i < size; i++){
        payloadArray[i] = 24;
      }
      message1 = createFrame(payloadArray, size);
      break;
    }
    case NOSTART:
    {
      size = 4;
      uint8_t payloadArray[size] = { 42, 56 , 73, 1};
      message1 = createFrame(payloadArray, size);
      message1[1] = 0b00000000; 
      break;
    }
    case NOEND:
    {
      size = 4;
      uint8_t payloadArray[size] = { 42, 56 , 73, 1};
      message1 = createFrame(payloadArray, size);
      message1[10] = 0b00000000; 
      break;
    }
    default:
    {
      Serial.println("ERROR CHOSE CASE");
      break;
    }
  }

  sendTaskTime = micros();
  for(int i = 0; i < message1.size(); ++i) {
    sendByte(message1[i]);
  }

  for (;;) {
    vTaskDelay(xDelay);
  }
}

// Function to send a byte
void sendByte(uint8_t bits) {
  int val;
  for(int i = 7; i >= 0; i--) {
    val = (bits >> i) & 0x01;
    sendPulse(val);
    vTaskDelay(xDelay);
  }
}

// Function to send a pulse
void sendPulse(int value) {
  digitalWrite(TX_PIN, value);
  delayMicroseconds(HALF_PERIOD);
  digitalWrite(TX_PIN, !value);
  delayMicroseconds(HALF_PERIOD);
}

// Task to receive frames
void receive(void *pvParameters) {
    Serial.println("taskreceive");
    int periodElapse;
    int rxVal;
    int FrameLenght; 
    uint8_t dataLenght = 0;
    bool finished = false;
    buffer.push_back(0);
    bool first = true;
    for (;;) {
        auto currentTime = micros();
        periodElapse = currentTime - lastChangeTime;
        if (!checkPeriod && (periodElapse >= HALF_PERIOD * 2 + TIME_BETWEEN_BITS - THRESHOLD_PERIOD) &&
            (periodElapse <= HALF_PERIOD * 2 + TIME_BETWEEN_BITS + THRESHOLD_PERIOD)) {
            checkPeriod = true;
        }
        if (receivedBit) {
            if (first){
              first = false;
              start_time = micros();
            }
            receivedBit = false;
            rxVal = digitalRead(RX_PIN);
            if (checkPeriod) {
              if (periodElapse >= HALF_PERIOD - THRESHOLD_PERIOD) {
                buffer.push_back(!rxVal); // Add inverted rxVal to buffer
              }
            } else {
              checkPeriod = true;
            }
            lastChangeTime = currentTime;
        }
        // Check if a complete byte (8 bits) has been accumulated
        if (buffer.size() == 33){
          for(int index = 23; index < 32; index++){
            dataLenght |= (buffer[index] << 7 - index + 24);
          }
          FrameLenght = 7 + dataLenght;
        }

        if (buffer.size() == FrameLenght * 8 && !finished) {
            finished = true;
            processBuffer();
            buffer.clear();
            lenghtFrame = FrameLenght;
            readFrame();
        }
        vTaskDelay(1); // Delay before checking again
    }
}

// Function to process the buffer and convert bits to bytes
void processBuffer() {
    for (size_t i = 0; i < buffer.size(); i += 8) {
        uint8_t byte = 0;
        for (int j = 0; j < 8; ++j) {
            byte |= buffer[i + j] << (7 - j);
        }
      receivedBytes.push_back(byte);
    }
    rxFrame = receivedBytes.data();
}

// Function to print an array of uint8_t
void printUint8Array(const uint8_t* array, size_t length) {
  for (size_t i = 0; i < length; i++) {
    Serial.print(array[i]);
    if (i < length - 1) {
      Serial.print(", ");  // Print a comma and space between values
    }
  }
  Serial.println();  // Move to the next line after printing all values
}
