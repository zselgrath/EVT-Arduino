#include "VCU.h"
#include "VCUTasks.cpp"
//#include "CanMessages.cpp"

//#define Serial 0x0

// Reads ADC values for both APPS sensors as well as the BSE and appends them to the oversampling array

void onReceive(int packetSize) {
  // received a packet
  Serial.print("\nReceived ");

  if (CAN.packetExtended()) {
    Serial.print("extended ");
  }

  if (CAN.packetRtr()) {
    // Remote transmission request, packet contains no data
    Serial.print("RTR ");
  }

  Serial.print("packet with id 0x");
  Serial.print(CAN.packetId(), HEX);

  int currentByte = 0;
  byte bytes [packetSize];

  if (CAN.packetRtr()) {
    Serial.print(" and requested length ");
    Serial.println(CAN.packetDlc());
  } else {
    Serial.print(" and length ");
    Serial.println(packetSize);
    // only print packet data for non-RTR packets
    while (CAN.available()) {
      bytes[currentByte] = ((byte)CAN.read());
      currentByte++;
      Serial.print(bytes[currentByte]);
      Serial.print(" ");
    }
    Serial.println();
  }

  if(bytes[0] == (byte)0xA8){
    Serial.println("RPM Received: " + String(0 + bytes[1] + (bytes[2] >> 8)));
  }
  Serial.println("B0: " + String(bytes[0]));

  Serial.println();
};

void VCU::init() {
    // BEGIN CRITICAL SECTION. BE CAREFUL MODIFYING THIS CODE. 
    this->setSafe();
    this->writePinStates(); // TODO: Determine if this is necessary before pinModes. Default states may be low.

    // Safety-critical pins
    pinMode(SDC_RELAY_PIN, OUTPUT);
    pinMode(PRECHARGE_PIN, OUTPUT);
    pinMode(DISCHARGE_PIN, OUTPUT);
    pinMode(CHARGED_ACTIVE_PIN, OUTPUT);
    pinMode(MOTOR_CONTROLLER_ENABLE_PIN, OUTPUT);
    this->writePinStates();

    // CAN Setup
    CAN.setPins(CAN_CS_PIN, CAN_INT_PIN);
    if (!CAN.begin(250E3)) {
      while(1){
        Serial.println("Starting CAN failed!");
        delay(100);
      }
    }else{
      Serial.println("CAN started");
    }

    // Set torque to 0 and disable the motor controller
    this->disableMotorController();

    // Register the CANbus receive callback
    CAN.onReceive(onReceive);

    // END CRITICAL SECTION

    if (!Serial) {
      Serial.begin(115200); // We do not wait for Serial to be initialized
    }

    // Sensor input pins
    analogReadResolution(ADC_READ_RESOLUTION);
    pinMode(APPS_1_PIN, INPUT);
    pinMode(APPS_2_PIN, INPUT);
    pinMode(BSE_PIN, INPUT_PULLDOWN);
    pinMode(SDC_NEGATIVE_CURRENT_PIN, INPUT);
    pinMode(SDC_POSITIVE_CURRENT_PIN, INPUT);

    // 5-way Switch Pins
    pinMode(S_UP_PIN, INPUT);
    pinMode(S_DOWN_PIN, INPUT);
    pinMode(S_LEFT_PIN, INPUT);
    pinMode(S_RIGHT_PIN, INPUT);
    pinMode(S_CENTER_PIN, INPUT);
    
    //Teensy3Clock.set(__DATE__);
//  Teensy3Clock.set(DateTime(F(__DATE__), F(__TIME__)));
//  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    //RTC.adjust(DateTime(__DATE__, __TIME__));

    // Declare tasks which should be executed
    tasks[0] = new UpdateThrottleAnalogValues(this->apps1samples, this->apps2samples, this->bseSamples);
    tasks[1] = new VehicleStateTask(this);
    tasks[2] = new SaveDataToSD();
    tasks[3] = new ValidateShutdownCircuitCurrent(this->sdcpSamples, this->sdcnSamples);
    tasks[4] = new PrintStatus(this);
    tasks[5] = new DoSpecificDebugThing(this);
    tasks[6] = new ProcessSerialInput(this);
    tasks[7] = new UpdateIMU(&(this->imuAccel), &(this->imuMag), &(this->imuGyro), &(this->imuTemperature));
    tasks[8] = new PrintWew();
    tasks[9] = new PrintLad();

    // Default variable initialization
    this->maxAdcValue = pow(2, ADC_READ_RESOLUTION);
    this->oversamplingLocation = 0;
    for (int i = 0; i < OVERSAMPLING_BUFFER_SIZE; i++) {
        this->apps1samples[i] = 0;
        this->apps2samples[i] = 0;
        this->bseSamples[i] = 0;
    }
    this->loopsCompleted = 0;
}

void VCU::setSafe() {
    this->sdcActive = false;
    this->prechargeActive = false;
    this->dischargeActive = true;
    this->chargedActive = false;
    this->motorControllerActive = false;
}

void VCU::setDangerous() {
//    this->sdcActive = true;
//    this->prechargeActive = true;
//    this->dischargeActive = false;
//    this->chargedActive = true;
//    this->motorControllerActive = true;
}

void VCU::writePinStates() {
    digitalWrite(SDC_RELAY_PIN, this->sdcActive);
    digitalWrite(PRECHARGE_PIN, this->prechargeActive);
    digitalWrite(DISCHARGE_PIN, this->dischargeActive);
    digitalWrite(CHARGED_ACTIVE_PIN, this->chargedActive);
    digitalWrite(MOTOR_CONTROLLER_ENABLE_PIN, this->motorControllerActive);
}

void VCU::vcuLoop() {
    for (int i = 0; i < TASKS_ARRAY_SIZE; i++) {
        volatile long currentTime = micros(); // Putting this inside the for loop may make sense
        if (tasks[i]->shouldExecute(currentTime)) {
            tasks[i]->execute();
        }
    }
    this->loopsCompleted++;
}


float VCU::getTorqueRegisterValue(){
  float toReturn = this->mapf(this->getApps1Float(), 0.185, 0.410, 0, -32000);
  if(toReturn < -31000){
    toReturn = -31000;
  }else if(toReturn > -10){
    toReturn = 0;
  }
  return toReturn;
}

void VCU::requestStartup() {
  requestTransition(CarState::TransitionType::ENABLE);
}

void VCU::requestShutdown() {
  requestTransition(CarState::TransitionType::DISABLE);
}

void VCU::requestTransition(CarState::TransitionType transitionType){
  CarState::NewStateType state = currentCarState->handleUserInput(*this, transitionType);
  if(state != CarState::NewStateType::SAME_STATE){
    noInterrupts();
    if(state == CarState::NewStateType::PRECHARGE_STATE){
      long timePassedSinceLUTR = millis() - lastUserTransitionRequest;
      if(timePassedSinceLUTR < MINIMUM_TRANSITION_DELAY){
        return; // It has not been long enough since the last transition
      }
      lastUserTransitionRequest = millis();
      delete currentCarState;
      currentCarState = new PrechargeState();
      currentCarState->enter(*this);
    }else if(state == CarState::NewStateType::OFF_STATE){
      delete currentCarState;
      currentCarState = new OffState();
      currentCarState->enter(*this);
    }
    // NOTE: We do not allow user transitions to ON_STATE. That can only be done by the precharge state update method, if its verification passes. 
    interrupts();
  }
};

void VCU::sendMotorControllerMessage(unsigned char *bytes, int messageLength) {
//    if (messageLength > 0) {
//        Serial.print("MC-CAN Sending: ");
//        for (int i = 0; i < messageLength; i++) {
//            Serial.print(String(bytes[i]) + ", ");
//        }
//        Serial.println("");
//    } else {
//        Serial.println("CAN Sending with no bytes?");
//    }

    CAN.beginPacket(canMotorControllerAddress); // ???
    for (int i = 0; i < messageLength; i++) {
        CAN.write(bytes[i]);
    }
    CAN.endPacket();
}

void VCU::setTorqueValue(int value) {
    unsigned char canTorquePacket[3] = {0x90, (byte)value, (byte)(value>>8)};
    this->sendMotorControllerMessage(canTorquePacket, 3);
}

void VCU::disableMotorController() {
  this->setTorqueValue(0);
  this->sendMotorControllerMessage(this->canArrDisableMotorController, 3);
}

void VCU::enableMotorController() //DO NOT USE, DANGEROUS TO OVERRIDE LIKE THIS
{
  this->setTorqueValue(0);
  this->sendMotorControllerMessage(this->canArrDisableMotorController, 0);
}
void VCU::requestRPM() {
  this->sendMotorControllerMessage(this->canRequestRPM, 3);
}

void VCU::requestMotorPosition() {
    this->sendMotorControllerMessage(this->canRequestMotorPosition, 3);
}


int VCU::getApps1() {
    int sum = 0;
    for (int apps1sample : apps1samples) {
        sum += apps1sample;
    }
    return sum / OVERSAMPLING_BUFFER_SIZE;
}

float VCU::getApps1Float() {
    return VCU::mapf(this->getApps1(), 0, this->maxAdcValue, 0.0f, 1.0f);
}
float VCU::getApps2Float() {
    return VCU::mapf(this->getApps2(), 0, this->maxAdcValue, 0.0f, 1.0f);
}

int VCU::getApps2() {
    int sum = 0;
    for (int apps2sample : apps2samples) {
        sum += apps2sample;
    }
    return sum / OVERSAMPLING_BUFFER_SIZE;
}

int VCU::getBse() {
    int sum = 0;
    for (int bseSample : bseSamples) {
        sum += bseSample;
    }
    return sum / OVERSAMPLING_BUFFER_SIZE;
}

int VCU::getSDCCurrentPos() {
    int sum = 0;
    for (int sdcpSample : sdcpSamples) {
        sum += sdcpSample;
    }
    return sum / NONCRITICAL_OVERSAMPLING_BUFFER_SIZE;
}

int VCU::getSDCCurrentNeg() {
    int sum = 0;
    for (int sdcnSample : sdcnSamples) {
        sum += sdcnSample;
    }
    return sum / NONCRITICAL_OVERSAMPLING_BUFFER_SIZE;
}


String VCU::getSwitchStates() {
    String result = "";
    if (this->up()) {
        result.concat('u');
    }
    if (this->down()) {
        result.concat('d');
    }
    if (this->left()) {
        result.concat('l');
    }
    if (this->right()) {
        result.concat('r');
    }
    if (this->center()) {
        result.concat('c');
    }
    return result;
}

bool VCU::anySwitchPressed() {
    return (this->up() || this->down() || this->left() || this->right() || this->center());
}

bool VCU::up() {
    return !digitalRead(S_UP_PIN);
}

bool VCU::down() {
    return !digitalRead(S_DOWN_PIN);
}

bool VCU::left() {
    return !digitalRead(S_LEFT_PIN);
}

bool VCU::right() {
    return !digitalRead(S_RIGHT_PIN);
}

bool VCU::center() {
    return !digitalRead(S_CENTER_PIN);
}

long VCU::getLoopsCompleted() {
    long result = this->loopsCompleted;
    this->loopsCompleted = 0;
    return result;
}

float VCU::mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool VCU::strContains(const String &outer, const String &inner) {
    return outer.indexOf(inner) >= 0;
}
