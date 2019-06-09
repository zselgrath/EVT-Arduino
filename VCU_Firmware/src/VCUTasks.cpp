#include "VCU.h"

#define MAXIMUM_ON_TIME 120000

// To add a new VCUTask, edit 3 things:
// 1. Add to this class, by adding a new class extending/implementing VCUTask
// 2. Modify TASKS_ARRAY_SIZE in VCU.h so that the array of VCUTask pointers can be sized correctly
// 3. Modify the VCU.cpp init() method, by adding the task to the array of tasks

class UpdateThrottleAnalogValues : public VCUTask {
public:
    void execute() override {
        // Voodoo-voltage (aka analog) sensor updates
        pApps1Samples[this->_oversamplingLocation] = analogRead(APPS_1_PIN);
        pApps2Samples[this->_oversamplingLocation] = analogRead(APPS_2_PIN);
        pBseSamples[this->_oversamplingLocation] = analogRead(BSE_PIN);
        this->_oversamplingLocation = (this->_oversamplingLocation + 1) % OVERSAMPLING_BUFFER_SIZE;
    }

    UpdateThrottleAnalogValues(int *apps1Array, int *apps2Array, int *bseArray) {
        pApps1Samples = apps1Array;
        pApps2Samples = apps2Array;
        pBseSamples = bseArray;
    }

private:
    volatile unsigned int getExecutionDelay() override { return 100; }

    int *pApps1Samples;
    int *pApps2Samples;
    int *pBseSamples;
    int _oversamplingLocation = 0;
};

// Save run data to the SD card periodically
class SaveDataToSD : public VCUTask {
public:
    void execute() override {
        // TODO
    }

private:
    volatile unsigned int getExecutionDelay() override { return 50000; }
};

// Update the IMU variables. This class has ownership of the LSM object since nothing else should care about using it.
class UpdateIMU : public VCUTask {
public:
    void execute() override {
        lsm.getEvent(&*accel, &*mag, &*gyro, &*temperature); // Pass the pointers to the function as references?
        //*mag = _mag;
    }

    explicit UpdateIMU(sensors_event_t *a, sensors_event_t *m, sensors_event_t *g, sensors_event_t *temperatureIn) {
        lsm = Adafruit_LSM9DS1();
        if (!lsm.begin()) {
            Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
        }
        lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
        lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
        lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
        this->accel = a;
        this->mag = m;
        this->gyro = g;
        this->temperature = temperatureIn;
    }

private:
    volatile unsigned int getExecutionDelay() override { return 100000; }

    Adafruit_LSM9DS1 lsm;
    sensors_event_t *accel, *mag, *gyro, *temperature;

};

// Reads the positive and negative shutdown circuit current sensors to validate that they are drawing what they should and that there is no significant imbalance
class ValidateShutdownCircuitCurrent : public VCUTask {
public:
    void execute() override {
        sdcpSamples[this->sdcOversamplingLocation] = analogRead(SDC_POSITIVE_CURRENT_PIN);
        sdcnSamples[this->sdcOversamplingLocation] = analogRead(SDC_NEGATIVE_CURRENT_PIN);
        this->sdcOversamplingLocation = (this->sdcOversamplingLocation + 1) % NONCRITICAL_OVERSAMPLING_BUFFER_SIZE;
        // The sensors as installed seem to rest at values of about 620 at 0 current, increasing by 50 per 100mA.
    }

    explicit ValidateShutdownCircuitCurrent(int *sdcpArray, int *sdcnArray) {
        sdcpSamples = sdcpArray;
        sdcnSamples = sdcnArray;
    }

private:
    volatile unsigned int getExecutionDelay() override { return 20000; }

    int *sdcpSamples; // Pointer to the array containing the shutdown circuit positive side current sensor ADC samples
    int *sdcnSamples;
    int sdcOversamplingLocation = 0;
};

// Periodically print the status of the vehicle over serial
class PrintStatus : public VCUTask {
public:
    void execute() override {
        int runRate = pVCU->getLoopsCompleted() * (1000000.0 / (getExecutionDelay() + 1.0f)); // avoid division by 0
        Serial.print(String(">").concat("        "));
        Serial.print(String(">").concat(pVCU->getSwitchStates()));
        Serial.print(String(" a1:").concat(pVCU->getApps1()));
        Serial.print(String(" a2:").concat(pVCU->getApps2()));
        Serial.print(String(" bse:").concat(pVCU->getBse()));
        Serial.print(String(" a1f:").concat(String(pVCU->getApps1ADCFloat(), 6)));
        Serial.print(String(" a2f:").concat(String(pVCU->getApps2ADCFloat(), 6)));
        Serial.print(String(" a1T:").concat(String(pVCU->getApps1Travel(), 6)));
        Serial.print(String(" a2T:").concat(String(pVCU->getApps2Travel(), 6)));
        Serial.print(String(" APPS:").concat(String(pVCU->getCheckedAndScaledAppsValue(), 6)));
        Serial.print(String(" sdcp:").concat(pVCU->getSDCCurrentPos()));
        Serial.print(String(" sdcn:").concat(pVCU->getSDCCurrentNeg()));
        Serial.print(String(" rr:").concat(runRate));
        String x = String(pVCU->imuAccel.acceleration.x).concat(",");
        String y = String(pVCU->imuAccel.acceleration.y).concat(",");
        String z = String(pVCU->imuAccel.acceleration.z);
        Serial.print(String(" axyz:").concat(x).concat(y).concat(z));
        Serial.println("");

        // Serial.print(String("").concat(String(pVCU->getApps1ADCFloat()*100.0, 6)));
        // Serial.print(String(",").concat(String(pVCU->getApps2ADCFloat()*100.0, 6)));
        // Serial.print(String(",").concat(String(pVCU->getApps1Travel()*100.0, 6)));
        // Serial.print(String(",").concat(String(pVCU->getApps2Travel()*100.0, 6)));
        // Serial.print(String(",").concat(String(pVCU->getCheckedAndScaledAppsValue()*100.0, 6)));
        // Serial.println(",");
    }

    explicit PrintStatus(VCU *aVCU) {
        pVCU = aVCU;
    }

private:
    volatile unsigned int getExecutionDelay() override { return 200000; }

    VCU *pVCU;
};

class OffState: public CarState {
  public:
    OffState(){
      Serial.println("Creating OffState");
    };
    NewStateType handleUserInput(VCU& vcu, CarState::TransitionType requestedTransition) override {
      if(requestedTransition == TransitionType::ENABLE){
        // Perform startup sequence, checking either here or in constructor for that state for safety checks
        // Serial.println("OffState transitioning to PrechargeState");
        return NewStateType::PRECHARGE_STATE;
      }
      //Serial.println("Continuing being off");
      return SAME_STATE;
    }
    void enter(VCU& vcu) override {
      // TODO: Set the car safe here
      vcu.setSafe();
      vcu.writePinStates();
      vcu.disableMotorController();
    }
    NewStateType update(VCU& vcu) override {
      // There's not much to update here when the car is off.
      return NewStateType::SAME_STATE;
    }
  private:
};

class PrechargeState: public CarState {
  public:
    PrechargeState(){
      Serial.println("Creating PrechargeState");
    };
    NewStateType handleUserInput(VCU& vcu, CarState::TransitionType requestedTransition) override {
      if(requestedTransition == TransitionType::DISABLE){
        // Halt startup
        return NewStateType::OFF_STATE;
      }
      return SAME_STATE;
    }
    void enter(VCU& vcu) override {
      Serial.println("Precharge entering.");
      this->prechargeStartTime = millis();
      bool prechargeCanContinue = prechargeIsSafeToStart(vcu);
      if(prechargeCanContinue){
        vcu.sdcActive = true; // Close the G2RL SDC relay
        vcu.prechargeActive = true; // Enable the pin to enable the precharge output
        vcu.dischargeActive = false; // Disable the precharge system (may not be real in the car)
        vcu.writePinStates();
        Serial.println("The car was determined to be okay to charge when starting precharge.");
      }else{
        Serial.println("The car was determined to be dangerous when starting precharge.");
      }
    }
    NewStateType update(VCU& vcu) override {
      // Validate that the car should be precharging. At some point, change the state to OnState.
      long currentPrechargeTime = millis() - this->prechargeStartTime;
      if(currentPrechargeTime < MAX_PRECHARGE_TIME){
        bool prechargeCanContinue = prechargeIsSafeToContinue(vcu);
        if(prechargeCanContinue){
          bool carIsPrecharged = false;
          if(currentPrechargeTime > 2000){
            carIsPrecharged = true; // TODO: THIS IS FAKE
          }
          // TODO: Determine if the car is ready to enable here
          if(carIsPrecharged){
            Serial.println("Precharge ending, CAR TURNING ON");
            return NewStateType::ON_STATE;
          }
          return NewStateType::SAME_STATE;
        }else{
          Serial.println("Car was determined to be dangerous during precharge loop");
          return NewStateType::OFF_STATE;
        }
      }
      // TODO: Log timeout message here
      Serial.println("Precharge timeout");
      return NewStateType::OFF_STATE;
    }
    
    // TODO: Determine if the car is safe and that precharge should proceed here 
    static bool prechargeIsSafeToStart(VCU& vcu){
      if(!vcu.acceleratorPedalIsPlausible()){
        Serial.println("Not starting precharge because the accelerator value is not plausible.");
        return false;
      }
      if(vcu.getCheckedAndScaledAppsValue() > 0.001){
        Serial.println("Not starting precharge because the accelerator is pressed.");
        return false;
      }
      return true;
    }

    static bool prechargeIsSafeToContinue(VCU& vcu){
      // TODO: Determine if the car is safe and that precharge should proceed here
      if(!vcu.acceleratorPedalIsPlausible()){
        Serial.println("Disabling precharge because the accelerator value is not plausible.");
        return false;
      }
      if(vcu.getCheckedAndScaledAppsValue() > 0.001){
        Serial.println("Disabling precharge because the accelerator is pressed.");
        return false;
      }
      return true;
    }
  private:
    long prechargeStartTime;
    const long MAX_PRECHARGE_TIME = 3000; // milliseconds
};

class OnState: public CarState {
  public:
    OnState(){
      Serial.println("Creating OnState");
    };
    NewStateType handleUserInput(VCU& vcu, CarState::TransitionType requestedTransition) override {
      if(requestedTransition == TransitionType::DISABLE){
        // TODO: Log a request to stop the car here
        return NewStateType::OFF_STATE;
      }
      return SAME_STATE;
    }
    void enter(VCU& vcu) override {
      // TODO: Set the car actually dangerous here
//      vcu.setDangerous();
      vcu.prechargeActive = false;
      vcu.dischargeActive = false;
      vcu.chargedActive = true;
      vcu.motorControllerActive = true;
      vcu.writePinStates(); // TODO: This might be fake
      vcu.enableMotorController();
      carStartTime = millis();
      
    }
    NewStateType update(VCU& vcu) override {
      // TODO: Perform runtime safety validation here
      long currentRuntime = millis() - carStartTime;
      if(currentRuntime > MAXIMUM_ON_TIME){
        return NewStateType::OFF_STATE; // TODO: THIS IS FAKE
      }else{
        return NewStateType::SAME_STATE;
      }
    }
  private:
    long carStartTime;
};

class VehicleStateTask : public VCUTask {
public:
    void execute() override {
      CarState::NewStateType state = pVCU->currentCarState->update(*pVCU);
      
      if(state != CarState::NewStateType::SAME_STATE){
        delete pVCU->currentCarState;
        Serial.println("Change");
        if(state == CarState::NewStateType::ON_STATE){
          // The startup/precharge state machine has requested a transition to ON
          Serial.println("Change to on");
          pVCU->currentCarState = new OnState();
        }else if(state == CarState::NewStateType::OFF_STATE){
          Serial.println("Change to off");
          // The startup/precharge state machine has requested a transition to OFF
          pVCU->currentCarState = new OffState();
        }
        pVCU->currentCarState->enter(*pVCU);
      }
    }

    explicit VehicleStateTask(VCU *vcuPointer) {
        this->pVCU = vcuPointer;
        pVCU->currentCarState = new OffState();
    }

private:
    volatile unsigned int getExecutionDelay() override { return 10000; }
    VCU *pVCU;
};

class HandlePumpAndFan : public VCUTask {
public:
    void execute() override {
        this->setPumpOutput(0.0, false);
        // float looper = (millis() / 1000) % 11;
        // float adder = pVCU->mapf(looper, 0.0, 10.0, 0.0, 1.0);
        // pVCU->setPumpOutput(adder);
    }
    explicit HandlePumpAndFan(VCU *aVCU) {
        pVCU = aVCU;
        this->setPumpOutput(0.0, true); // TODO: Determine how this should default
    }

  void setPumpOutput(float value, bool force){
    if(value != pumpSetpoint || force){
      pumpSetpoint = value;
      Wire.beginTransmission(PUMP_DIGITAL_POTENTIOMETER);
      byte valueToWrite = (byte)pVCU->mapf(value, 0.0, 1.0, 0, 127);
      Wire.write(valueToWrite);
      Wire.endTransmission();
      Serial.println("Setting pump to:" + String(valueToWrite));
    }
  }
private:
    volatile unsigned int getExecutionDelay() override { return 1000000; }
    float pumpSetpoint = 0.0f;
    VCU *pVCU;
};

class HandleBrakeLight : public VCUTask {
public:
    void execute() override {
        if(pVCU->getBseTravel() > BSE_TRAVEL_BRAKE_LIGHT_THRESHOLD){
          // TODO: Handle brake light turning on and off
        }else{

        }
    }
    explicit HandleBrakeLight(VCU *aVCU) {
        pVCU = aVCU;
        // TODO: PinMode state for brake light
    }
private:
    volatile unsigned int getExecutionDelay() override { return 50000; }
    const float BSE_TRAVEL_BRAKE_LIGHT_THRESHOLD = 0.05f;
    VCU *pVCU;
};

class HandleDashUpdates : public VCUTask {
public:
    void execute() override {
        pVCU->sendDashText(String((millis() / 1000) % 1000));
    }
    explicit HandleDashUpdates(VCU *aVCU) {
        pVCU = aVCU;
        
    }
private:
    volatile unsigned int getExecutionDelay() override { return 100000; }
    VCU *pVCU;
};


// Just used to prove that multiple tasks are running at independent rates
class PrintWew : public VCUTask {
public:
    void execute() override {
        //Serial.println("wew");
    }

private:
    volatile unsigned int getExecutionDelay() override { return 5000000; }
};

// Just used to prove that multiple tasks are running at independent rates
class PrintLad : public VCUTask {
public:
    void execute() override {
        //Serial.println("lad");
    }

private:
    volatile unsigned int getExecutionDelay() override { return 6000100; }
};

// This is a debugging and development class. What it does depends on what is being worked on.
class DoSpecificDebugThing : public VCUTask {
public:
    void execute() override {
      float genericTorque = pVCU->calculateTorqueRegisterValueForWrite();
//      Serial.println("I could set torque to " + String(genericTorque));
      // Serial.println(String(genericTorque));
      //Serial.println(pVCU->getCheckedAndScaledAppsValue());
      //Serial.println("VCU: " + String("e"));
      if (pVCU->right()) {
          Serial.println("Disabling motor controller torque");
          pVCU->disableMotorController();
      }
//      if(pVCU->center()){
//        Serial.println("Setting torque to " + String((int) genericTorque));
//        pVCU->setTorqueValue((int) genericTorque);
//      }else{
//        pVCU->setTorqueValue(0);
//      }
        // pVCU->setTorqueValue((int) genericTorque);
//      if(pVCU->center()){
//        Serial.println("Requesting RPM");
//        pVCU->requestRPM();
//        pVCU->requestMotorPosition();
//      }
    }

    explicit DoSpecificDebugThing(VCU *vcuPointer) {
        this->pVCU = vcuPointer;
    }

private:
    volatile unsigned int getExecutionDelay() override { return 20000; }

    VCU *pVCU;
};

// This class allows using the serial terminal to send commands to be processed.
class ProcessSerialInput : public VCUTask {
public:
    void execute() override {
        while (Serial.available() > 0) {
            didRead = true;
            serialBuffer[serialBufferIndex] = Serial.read();
            serialBufferIndex++;
            if (serialBufferIndex + 1 > SERIAL_BUFFER_SIZE) {
                // This is an overflow.
                serialBufferIndex = 0;
                serialBuffer[serialBufferIndex] = '\0';
            } else {
                serialBuffer[serialBufferIndex] = '\0';
            }
        }
        if (didRead) {
            didRead = false;
            serialBufferIndex = 0;
            String serialLineReceived = String(serialBuffer);
            Serial.println("Received: " + serialLineReceived);
            // Dump EEPROM
            if (pVCU->strContains(serialLineReceived.toLowerCase(), "dumpeeprom")) {
                int address = 0;
                byte value;
                while (address < EEPROM_BYTES) {
                    value = EEPROM.read(address);
                    Serial.print(value, HEX);
                    address++;
                    if (address % 32 == 0) {
                        Serial.println("");
                    }
                }
                Serial.println("");
            }
            // Set a single EEPROM byte
            if (pVCU->strContains(serialLineReceived.toLowerCase(), "seteeprom")) {
                int address = 0;
                byte value = 0;
                String processingString, eepromAddress, eepromValue;

                processingString = serialLineReceived.substring(serialLineReceived.indexOf(' ') + 1);
                eepromAddress = processingString.substring(0, processingString.indexOf(' '));
                if (address >= EEPROM_BYTES) {
                    Serial.println("EEPROM address too large.");
                    return;
                }

                processingString = processingString.substring(processingString.indexOf(' ') + 1);
                if (processingString.indexOf(' ') == -1) {
                    eepromValue = processingString;//.substring(0, processingString.indexOf(' '));
                } else {
                    Serial.println("Error parsing string, probably too many spaces");
                    return;
                }

                if (eepromValue.toInt() < 256) {
                    address = eepromAddress.toInt();
                    value = eepromValue.toInt(); // Ints in Teensy are 32 bits
                } else {
                    Serial.println("EEPROM value does not fit within 1 byte");
                    return;
                }

                EEPROM.write(address, value);

                Serial.println("Wrote to " + String(address) + " with " + String(value));
            }
        }
    }

    explicit ProcessSerialInput(VCU *vcuPointer) {
        this->pVCU = vcuPointer;
        for (int i = 0; i < SERIAL_BUFFER_SIZE; i++) {
            serialBuffer[i] = 0;
        }
    }

private:
    volatile unsigned int getExecutionDelay() override { return 50000; }

    VCU *pVCU;
    static const int SERIAL_BUFFER_SIZE = 32;
    char serialBuffer[SERIAL_BUFFER_SIZE] = {};
    int serialBufferIndex = 0;
    bool didRead = false;
};
