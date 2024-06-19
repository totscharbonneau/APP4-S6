#include <vector>

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
}

void loop() {
  readFrame();
}
