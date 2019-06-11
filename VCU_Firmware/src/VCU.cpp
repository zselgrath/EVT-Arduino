#include "VCU.h"
#include "VCUTasks.cpp"
//#include "CanMessages.cpp"

//#define Serial 0x0

// Reads ADC values for both APPS sensors as well as the BSE and appends them to the oversampling array

static void onReceive(int packetSize) {
  // received a packet
  // Serial.print("\nReceived ");

  // if (CAN.packetExtended()) {
  //   Serial.print("extended ");
  // }

  // if (CAN.packetRtr()) {
  //   // Remote transmission request, packet contains no data
  //   Serial.print("RTR ");
  // }

  // Serial.print("packet with id 0x");
  // Serial.print(CAN.packetId(), HEX);

  int currentByte = 0;
  byte bytes [packetSize];

  if (CAN.packetRtr()) {
    // Serial.print(" and requested length ");
    // Serial.println(CAN.packetDlc());
  } else {
    // Serial.print(" and length ");
    // Serial.println(packetSize);
    // only print packet data for non-RTR packets
    while (CAN.available()) {
      bytes[currentByte] = ((byte)CAN.read());
      currentByte++;
      // Serial.print(bytes[currentByte]);
      // Serial.print(" ");
    }
    // Serial.println();
  }

  if(bytes[0] == 0){
    Serial.println("Received a packet where the first byte was 0. Unhandled.");
    return;
  }
  long currentTime = millis();
  for(int i = 0; i < MC_REGISTER_ARRAY_LENGTH; i++){
    if(bytes[0] == mcObjects[i].registerLocation){
      // Serial.println("I care about this packet");
      mcObjects[i].lastReadTime = currentTime;
      for(int j = 0; j < MC_VALUES_HOLDER_SIZE; j++){
        mcObjects[i].values[j] = bytes[j+1]; // Offset 1 since the first byte is the address being received
      }
      break; // We found the register we needed to update, stop looping
    }
  }

  // if(bytes[0] == (byte)0xA8){
  //   Serial.println("RPM Received: " + String(0 + bytes[1] + (bytes[2] >> 8)));
  // }
  // Serial.println("B0: " + String(bytes[0]));

  // Serial.println();
}

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
    if (!CAN.begin((long)250E3)) {
      while(true){
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

    // CAN0 (frontend, FlexCAN) setup
    CAN_filter_t defaultedMask; //Needed to do full declaration for can bus on secondary pins
    const byte canAltTxRx = 1;
    // FlexCAN canBus(CAN_BAUD_RATE);
     //canBus = FlexCAN(CAN_BAUD_RATE);
    defaultedMask.ext = 0;
    defaultedMask.rtr = 0;
    defaultedMask.id = 0;
    canBus.begin(CAN_BAUD_RATE, defaultedMask, canAltTxRx, canAltTxRx);

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
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    
    //Teensy3Clock.set(__DATE__);
//  Teensy3Clock.set(DateTime(F(__DATE__), F(__TIME__)));
//  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    //RTC.adjust(DateTime(__DATE__, __TIME__));

    // Declare tasks which should be executed
    tasks[0] = new UpdateThrottleAnalogValues(this->apps1samples, this->apps2samples, this->bseSamples);
    tasks[1] = new VehicleStateTask(this);
    tasks[2] = new RequestCanData(this);
    tasks[3] = new SaveDataToSD(this);
    tasks[4] = new ValidateShutdownCircuitCurrent(this->sdcpSamples, this->sdcnSamples);
    tasks[5] = new PrintStatus(this);
    tasks[6] = new DoSpecificDebugThing(this);
    tasks[7] = new ProcessSerialInput(this);
    tasks[8] = new UpdateImu(&(this->imuAccel), &(this->imuMag), &(this->imuGyro), &(this->imuTemperature));
    tasks[9] = new HandlePumpAndFan(this);
    tasks[10] = new HandleBrakeLight(this);
    tasks[11] = new HandleDashUpdates(this);
    tasks[12] = new PrintWew();
    tasks[13] = new PrintLad();

    // Default variable initialization
    this->maxAdcValue = pow(2, ADC_READ_RESOLUTION);
    for (int i = 0; i < OVERSAMPLING_BUFFER_SIZE; i++) {
        this->apps1samples[i] = 0;
        this->apps2samples[i] = 0;
        this->bseSamples[i] = 0;
    }
    this->loopsCompleted = 0;

    // Register important registers to fetch. 
    // This is not a clean way to do this; each message type is its own concept, and really need their own classes. A map would also help.
    for(int i = 0; i < MC_REGISTER_ARRAY_LENGTH; i++){
      mcObjects[i].registerLocation = 0;
      mcObjects[i].name = String("none");
      mcObjects[i].lastReadTime = 0;
      // importantMotorControllerRegisters[i] = 0;
      // motorControllerRegisterNames[i] = String("none"); // This might be dangerous
      // motorControllerRegisterLastReadTimes[i] = 0;
      for(int j = 0; i < MC_VALUES_HOLDER_SIZE; i++){
        mcObjects[i].values[j] = 0x00;
        // receivedMotorControllerValues[i][j] = 0x00;
      }
    }
    mcObjects[0].registerLocation = MOTOR_CONTROLLER_ADDRESS_VOLTAGE_BUS;
    mcObjects[0].name = "motorControllerBusVoltage";

    mcObjects[1].registerLocation = MOTOR_CONTROLLER_ADDRESS_VOLTAGE_OUTPUT;
    mcObjects[1].name = "motorControllerOutputVoltage";

    mcObjects[2].registerLocation = MOTOR_CONTROLLER_ADDRESS_CURRENT_PACK;
    mcObjects[2].name = "motorControllerPackCurrent";

    mcObjects[3].registerLocation = MOTOR_CONTROLLER_ADDRESS_CURRENT_PHASE;
    mcObjects[3].name = "motorControllerPhaseCurrent";

    mcObjects[4].registerLocation = MOTOR_CONTROLLER_ADDRESS_MOTOR_TEMPERATURE;
    mcObjects[4].name = "motorControllerMotorTemperature";

    mcObjects[5].registerLocation = MOTOR_CONTROLLER_ADDRESS_IGBT_TEMPERATURE;
    mcObjects[5].name = "motorControllerIgbtTemperature";

    mcObjects[6].registerLocation = MOTOR_CONTROLLER_ADDRESS_SETPOINT_CURRENT;
    mcObjects[6].name = "motorControllerCurrentSetpoint";

    mcObjects[7].registerLocation = MOTOR_CONTROLLER_ADDRESS_SETPOINT_TORQUE;
    mcObjects[7].name = "motorControllerTorqueSetpoint";
    
    mcObjects[8].registerLocation = MOTOR_CONTROLLER_ADDRESS_RPM;
    mcObjects[8].name = "motorControllerRpm";
}

void VCU::sendDashText(String text){
  // Serial.println("Displaying " + text);
  CAN_message_t msg;
  msg.id = canDashAddress;
  msg.len = 4; // Not text.length since we only have 3 digits, plus the target register address
  msg.timeout = 20;
  const byte dashTextRegister = 0x01;
  char charArray[3];
  text.toCharArray(charArray, 3);
  msg.buf[0] = dashTextRegister;
  msg.buf[1] = charArray[0];
  msg.buf[2] = charArray[1];
  msg.buf[3] = charArray[2];
  // bamStat.commanded.lastMessageStatus = canBus.write(msg);
  // Serial.println("CBM: " + canBus.write(msg));
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
        volatile unsigned long currentTime = micros(); // Putting this inside the for loop may make sense
        if (tasks[i]->shouldExecute(currentTime)) {
            tasks[i]->execute();
        }
    }
    this->loopsCompleted++;
}

int VCU::calculateTorqueRegisterValueForWrite(){
  float toReturn = VCU::mapf(this->getDeratedTorqueSetpoint(), 0.0, 1.0, 0, -32767); // the max might actually be 32768
  if(toReturn < -32767){
    toReturn = -32767;
  }else if(toReturn > -10){
    toReturn = 0;
  }
  return (int) toReturn;
}

void VCU::requestStartup() {
  requestTransition(CarState::TransitionType::ENABLE);
}

void VCU::requestShutdown() {
  requestTransition(CarState::TransitionType::DISABLE);
}

void VCU::requestTransition(CarState::TransitionType transitionType){
  CarState::CarStateType state = currentCarState->handleUserInput(*this, transitionType);
  if(state != CarState::CarStateType::SAME_STATE){
    noInterrupts();
    if(state == CarState::CarStateType::PRECHARGE_STATE){
      long timePassedSinceLastUserTransitionRequest = millis() - this->lastUserTransitionRequest;
      if(timePassedSinceLastUserTransitionRequest < MINIMUM_TRANSITION_DELAY){
        // Serial.println("TPSL: " + String(timePassedSinceLastUserTransitionRequest) + ", LUTR: " + String(this->lastUserTransitionRequest));
        return; // It has not been long enough since the last transition
      }
      this->lastUserTransitionRequest = millis();
      delete currentCarState;
      currentCarState = new PrechargeState();
      currentCarState->enter(*this);
    }else if(state == CarState::CarStateType::OFF_STATE){
      delete currentCarState;
      currentCarState = new OffState();
      currentCarState->enter(*this);
    }
    // NOTE: We do not allow user transitions to ON_STATE. That can only be done by the precharge state update method, if its verification passes. 
    interrupts();
  }
}

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
    unsigned char canTorquePacket[3] = {MOTOR_CONTROLLER_ADDRESS_SETPOINT_TORQUE, (byte)value, (byte)(value>>8)};
    this->sendMotorControllerMessage(canTorquePacket, 3);
}

void VCU::disableMotorController() {
  this->setTorqueValue(0);
  this->sendMotorControllerMessage(this->canArrDisableMotorController, 3);
}

void VCU::enableMotorController() //DO NOT USE, DANGEROUS TO OVERRIDE LIKE THIS
{
  this->setTorqueValue(0);
  this->sendMotorControllerMessage(this->canArrEnableMotorController, 3);
}

void VCU::requestMotorControllerRegisterOnce(byte registerAddress) {
  unsigned char packetToSend[3] = {regReadBamocarData, registerAddress, 0x00};
  this->sendMotorControllerMessage(packetToSend, 3);
}

bool VCU::carIsOn(){
  return this->currentCarState->getStateType() == CarState::CarStateType::ON_STATE;
}

bool VCU::carIsOff(){
  return this->currentCarState->getStateType() == CarState::CarStateType::OFF_STATE;
}

// This was thrown together for the start button and is different from the checks that the VCU does to determine if it is SAFE to start. 
// This is essentially a debounce and it (and the main.cpp / .ino) needs to be reworked to cancel precharge. 
bool VCU::carCanStart(){
  if(carIsOff() && (this->currentCarState->getTimeInState() > MINIMUM_TRANSITION_DELAY)){
    return true;
  }else{
    return false;
  }
}

int VCU::getApps1() {
    int sum = 0;
    for (int apps1Sample : apps1samples) {
        sum += apps1Sample;
    }
    return sum / OVERSAMPLING_BUFFER_SIZE;
}

int VCU::getApps2() {
    int sum = 0;
    for (int apps2Sample : apps2samples) {
        sum += apps2Sample;
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

// Returns the APPS 1 float value as scaled from 0V to AREF (3.3V)
float VCU::getApps1AdcFloat() {
    return VCU::mapf(this->getApps1(), 0, this->maxAdcValue, 0.0f, 1.0f);
}
// Returns the APPS 2 float value as scaled from 0V to AREF (3.3V)
float VCU::getApps2AdcFloat() {
    return VCU::mapf(this->getApps2(), 0, this->maxAdcValue, 0.0f, 1.0f);
}
// Returns the BSE float value as scaled from 0V to AREF (3.3V)
float VCU::getBseAdcFloat() {
    return VCU::mapf(this->getBse(), 0, this->maxAdcValue, 0.0f, 1.0f);
}

float VCU::getApps1Travel(){
  // These values are the ADC scale readings from the extremes of physical travel
  float toReturn = VCU::mapf(this->getApps1AdcFloat(), 0.1604, 0.411, 0.0f, 1.0f);
  return toReturn; // This method may return values outside of the range of 0.0 .. 1.0
}

float VCU::getApps2Travel(){
  // These values are the ADC scale readings from the extremes of physical travel
  float toReturn = VCU::mapf(this->getApps2AdcFloat(), 0.1128, 0.2792, 0.0f, 1.0f);
  return toReturn; // This method may return values outside of the range of 0.0 .. 1.0
}

float VCU::getBseTravel(){
  // These values are the ADC scale readings from the extremes of physical travel
  float toReturn = VCU::mapf(this->getBseAdcFloat(), 0.1, 0.2, 0.0f, 1.0f);
  return toReturn; // This method may return values outside of the range of 0.0 .. 1.0
}

bool VCU::acceleratorPedalIsPlausible(){
    float apps1Scaled = this->getApps1Travel(); 
    float apps2Scaled = this->getApps2Travel(); 
    bool apps1IsInRange = apps1Scaled > -0.1 && apps1Scaled < 1.1;
    bool apps2IsInRange = apps2Scaled > -0.1 && apps2Scaled < 1.1;
    bool appsAreInRange = apps1IsInRange && apps2IsInRange;
    if(!appsAreInRange){
      return false; // Return here if there's no way the values could be valid, since one of them is very far out of valid range
    }

    bool apps1IsTooHigh = (apps1Scaled > (apps2Scaled + 0.05)); // 0.05 is literally 5% of the calculated pedal travel percentage
    bool apps1IsTooLow = (apps1Scaled < (apps2Scaled - 0.05));
    bool isPlausible = (!apps1IsTooHigh && !apps1IsTooLow);
    return isPlausible; // Return here if the values vary too significantly (5%)
}

// Returns the APPS values from 0.0 to 1.0, performing the scaling as a fraction of their calibrated readings
// and returning 0 if they are significantly different aka implausible (T 6.2.3).
float VCU::getCheckedAndScaledAppsValue() {
    float apps1Scaled = this->getApps1Travel(); 
    float apps2Scaled = this->getApps2Travel(); 
    float physicalTravel = 0.0f;
    bool isImplausible = !(this->acceleratorPedalIsPlausible());
    if(isImplausible){
      // Serial.print("The APPS values differ by more than 5% - they are implausible.");
      // Serial.println(String(toReturn) + ", " + String(apps1Scaled) + ", " + String(apps2Scaled));
      // Serial.println(apps1Scaled > (apps2Scaled * 1.08));
      // Serial.println(apps1Scaled < (apps2Scaled / 1.08));
      return 0.0f;
    }else{
      physicalTravel = (apps1Scaled + apps2Scaled)/2.0f;
    }
    if(physicalTravel > 1.0f){
      physicalTravel = 1.0f;
    }
    if(physicalTravel < 0.001f){
      physicalTravel = 0.0f;
    }
    float toReturn = 0.0f;
    // Scale based on a pedal deadzone
    if(physicalTravel > PEDAL_DEADZONE){
      toReturn = VCU::mapf(physicalTravel, PEDAL_DEADZONE, 1.0, 0.0f, 1.0f);  // We do not currently scale based on the full-travel deadzone because that could increase the torque requested by the driver. This may change.
    }
    // Make sure that the scaled value is possible
    if(toReturn < 0.0f){
      toReturn = 0.0f;
    }else if(toReturn > 1.0){
      toReturn = 1.0;
    }
    return toReturn;
}

float VCU::getDeratedTorqueSetpoint(){
  // TODO: Make this derate, and something near here also probably perform safety checks
  return this->getCheckedAndScaledAppsValue(); // TODO: Make this real.
}

int VCU::getSdcCurrentPos() {
    int sum = 0;
    for (int sdcpSample : sdcpSamples) {
        sum += sdcpSample;
    }
    return sum / NONCRITICAL_OVERSAMPLING_BUFFER_SIZE;
}

int VCU::getSdcCurrentNeg() {
    int sum = 0;
    for (int sdcnSample : sdcnSamples) {
        sum += sdcnSample;
    }
    return sum / NONCRITICAL_OVERSAMPLING_BUFFER_SIZE;
}


String VCU::getSwitchStates() {
    String result = "";
    if (VCU::up()) {
        result.concat('u');
    }
    if (VCU::down()) {
        result.concat('d');
    }
    if (VCU::left()) {
        result.concat('l');
    }
    if (VCU::right()) {
        result.concat('r');
    }
    if (VCU::center()) {
        result.concat('c');
    }
    return result;
}

bool VCU::anySwitchPressed() {
    return (VCU::up() || VCU::down() || VCU::left() || VCU::right() || VCU::center());
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

bool VCU::start() {
    return !digitalRead(START_BUTTON_PIN);
}

long VCU::getLoopsCompleted() {
    long result = this->loopsCompleted;
    this->loopsCompleted = 0;
    return result;
}

float VCU::mapf(float x, float inMin, float inMax, float outMin, float outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

bool VCU::strContains(const String &outer, const String &inner) {
    return outer.indexOf(inner) >= 0;
}
