#include "VCU.h"

#define MAXIMUM_ON_TIME 600000 // 10 mins

// To add a new VCUTask, edit 3 things:
// 1. Add to this class, by adding a new class extending/implementing VCUTask
// 2. Modify TASKS_ARRAY_SIZE in VCU.h so that the array of VCUTask pointers can be sized correctly
// 3. Modify the VCU.cpp init() method, by adding the task to the array of tasks

class UpdateThrottleAnalogValues : public VCUTask {
public:
    void execute() override {
        // Voodoo-voltage (aka analog) sensor updates
        pApps1Samples[this->oversamplingLocation] = analogRead(APPS_1_PIN);
        pApps2Samples[this->oversamplingLocation] = analogRead(APPS_2_PIN);
        pBseSamples[this->oversamplingLocation] = analogRead(BSE_PIN);
        this->oversamplingLocation = (this->oversamplingLocation + 1) % OVERSAMPLING_BUFFER_SIZE;

        bool startPressedHere = !digitalRead(START_BUTTON_PIN);
        if(startSignalIsActive != startPressedHere){
          // The start button transitioned
          startSignalIsActive = startPressedHere;
          if(startPressedHere){
            // Start button transitioned active
            startButtonPressedBeginTime = millis();
            Serial.println("Start button transitioned to active");
          }else{
            // Start button transitioned to inactive
            startButtonPressedBeginTime = 0;
          }
        }
        if(startSignalIsActive && startButtonPressedBeginTime != 0){
          if(millis() > startButtonPressedBeginTime + 100l && pVCU->startIsPressed == false){
            // Start has been pressed for 100ms, we're really starting now
            Serial.println("Start button has been pressed for 100ms.");
            pVCU->startIsPressed = true;
          }
        }else{
          pVCU->startIsPressed = false;
        }
    }

    UpdateThrottleAnalogValues(int *apps1Array, int *apps2Array, int *bseArray, VCU *aVCU) {
        pApps1Samples = apps1Array;
        pApps2Samples = apps2Array;
        pBseSamples = bseArray;
        pVCU = aVCU;
    }

private:
    volatile unsigned int getExecutionDelay() override { return 100; }

    int *pApps1Samples;
    int *pApps2Samples;
    int *pBseSamples;
    int oversamplingLocation = 0;
    VCU *pVCU;
    volatile bool startSignalIsActive = false;
    volatile unsigned long startButtonPressedBeginTime = 0;
};

// Save run data to the SD card periodically
class SaveDataToSD : public VCUTask {
public:
    void execute() override {
      unsigned long start = micros();
      // Serial.println("Titles: " + getTitleString());
      String mcString = getMotorControllerString();
      // Serial.println("MC: " + mcString);

      String lineToWrite = mcString;
      char fileName[12];
      String asString = getLogFileName();
      getLogFileName().toCharArray(fileName, 12);

      logToFile(getGeneralReportString() + " mc: " + lineToWrite, fileName);
      // logToFile(getGeneralReportString(), fileName);

      unsigned long end = micros();
      Serial.println("Time to log: " + String(end - start));

      Serial.println("BMS 1: ");
      Serial.println("Pack Current, Pack Voltage, pack DCL, pack CCL, Simulated SOC, High Temp, Low Temp, SOC, Resistance, Open Voltage, Capacity");
      Serial.println(getBMSString1());

      Serial.println("BMS 2: ");
      Serial.println("Pack Current, Pack Voltage, pack DCL, pack CCL, Simulated SOC, High Temp, Low Temp, SOC, Resistance, Open Voltage, Capacity");
      Serial.println(getBMSString2());

    }

    String getLogFileName(){
      char fileName[12] = "";
      sprintf(fileName, "%s%i", fileName, sdLoggerStartTime);
      sprintf(fileName, "%s%s", fileName, ".txt");
      return String(fileName);
    };

    void logToFile(String stringToWrite, char* fileNameToWriteTo){
#ifdef USE_I2C_LOGGING
      Wire1.beginTransmission(LOGGER_I2C_ADDRESS);
      int length = stringToWrite.length();
      char charactersToTransmit[length];
      stringToWrite.toCharArray(charactersToTransmit, length);
      for(int i = 0; i < length; i++){
        Wire1.write(charactersToTransmit[i]);
      }
      Wire1.write('\n');
      Wire1.endTransmission();
#endif
#ifdef USE_SD_LOGGING
      if(!SD.exists(fileNameToWriteTo)){
        Serial.println("No SD file; could not log " + stringToWrite);
        if(!SD.begin(SD_CHIP_SELECT)){
          Serial.println(" -- NO SD CARD DETECTED");
          return;
        }
      }

      dataFile = SD.open(fileNameToWriteTo, FILE_WRITE);
      // If the data file is available, write to it:
      if (dataFile) {
        dataFile.println(stringToWrite);
        dataFile.close();
        // Serial.println("L, " + String(fileNameToWriteTo) + ": " + stringToWrite);
      } else {
        // If the file isn't open, report an error:
        Serial.print("Error opening SD for " + String(fileNameToWriteTo));
      }
#endif
    }

    String getGeneralReportString(){
      String toReturn = "";
      int runRate = pVCU->getLoopsCompleted() * (1000000.0f / (getExecutionDelay() + 1.0f)); // avoid division by 0
      // toReturn.concat(String("").concat("        "));
      toReturn.concat(String("> ms:").concat(String(millis())));
      toReturn.concat(String(" ss:").concat(pVCU->getSwitchStates()));
      toReturn.concat(String(" a1:").concat(pVCU->getApps1()));
      toReturn.concat(String(" a2:").concat(pVCU->getApps2()));
      toReturn.concat(String(" bse:").concat(pVCU->getBse()));
      toReturn.concat(String(" a1f:").concat(String(pVCU->getApps1AdcFloat(), 6)));
      toReturn.concat(String(" a2f:").concat(String(pVCU->getApps2AdcFloat(), 6)));
      toReturn.concat(String(" a1T:").concat(String(pVCU->getApps1Travel(), 6)));
      toReturn.concat(String(" a2T:").concat(String(pVCU->getApps2Travel(), 6)));
      toReturn.concat(String(" APPS:").concat(String(pVCU->getCheckedAndScaledAppsValue(), 6)));
      toReturn.concat(String(" sdcp:").concat(pVCU->getSdcCurrentPos()));
      toReturn.concat(String(" sdcn:").concat(pVCU->getSdcCurrentNeg()));
      toReturn.concat(String(" rr:").concat(runRate));
      String x = String(String(pVCU->imuAccel.acceleration.x).concat(","));
      String y = String(String(pVCU->imuAccel.acceleration.y).concat(","));
      String z = String(pVCU->imuAccel.acceleration.z);
      toReturn.concat(String(" axyz:").concat(x).concat(y).concat(z));
      return toReturn;
    }

    explicit SaveDataToSD(VCU *aVCU) {
        pVCU = aVCU;
        String toAppend = mcObjects[0].name;
        for(int i = 1; i < MC_REGISTER_ARRAY_LENGTH; i++){
          toAppend = toAppend + ", " + mcObjects[i].name;
        }

#ifdef USE_I2C_LOGGING
        Wire1.begin();
        Wire1.setClock(1000000);
#endif

#ifdef USE_SD_LOGGING
        // SD card logging stuff
        Serial.print("Initializing SD card...");
        // See if the card is present and can be initialized:
        if (!SD.begin(SD_CHIP_SELECT)) {
          Serial.println("Card failed, or not present");
        }else{
          Serial.println("card initialized.");
        }
        char fileName2[12] = "";
        sdLoggerStartTime = hour()*10000 + minute()*100 + SaveDataToSD::countAllFiles(); // http://i.imgur.com/Me04jVB.jpg
        sprintf(fileName2, "%s%i", fileName2, sdLoggerStartTime);
        sprintf(fileName2, "%s%s", fileName2, ".txt");
        Serial.print("FileName2: ");
        Serial.println(fileName2);
        dataFile = SD.open(fileName2, FILE_WRITE);
        dataFile.close();
        if(SD.exists(fileName2)){
          Serial.println("Datafile exists!");
        }else{
          Serial.println("Datafile doesn't exist.");
        }
#endif
        // Serial.println("Labels: " + getTitleString());
        char logFileName[12];
        getLogFileName().toCharArray(logFileName, 12);
        logToFile("begin", logFileName);
        logToFile(getTitleString(), logFileName);
    }

    String getTitleString(){
      if(titleString == "" || millis() < 500){
        String toAppend = mcObjects[0].name;
        for(int i = 1; i < MC_REGISTER_ARRAY_LENGTH; i++){
          if(mcObjects[i].registerLocation == 0){
            break;
          }
          toAppend = toAppend + ",  " + mcObjects[i].name;
        }
        titleString = toAppend;
        // Serial.println("Labels: " + toAppend);
      }
      return titleString;
    }

    String getMotorControllerString(){
      String toAppend = (String((mcObjects[0].values[1] << 8) + mcObjects[0].values[0]));
      for(int i = 1; i < MC_REGISTER_ARRAY_LENGTH; i++){
        if(mcObjects[i].registerLocation == 0){
          break;
        }
        short value = (mcObjects[i].values[1] << 8) + mcObjects[i].values[0]; // Not sure which is more or less significant (endianness)
        toAppend = toAppend + ", " + String(value);
      }
      return toAppend;
    }

    String getBMSString1() {
      return(
          String(bms1.message1.packCurrent) + ","
        + String(bms1.message1.packVoltage) + ","
        + String(bms1.message2.packDCL) + ","
        + String(bms1.message2.packCCL) + ","
        + String(bms1.message2.simulatedSOC) + ","
        + String(bms1.message2.highTemp) + ","
        + String(bms1.message2.lowTemp) + ","
        + String(bms1.message3.SOC) + ","
        + String(bms1.message3.packResistance) + ","
        + String(bms1.message3.packOpenVoltage) + ","
        + String(bms1.message3.packAmpHours)
      );
    }


        String getBMSString2() {
          return(
              String(bms2.message1.packCurrent) + ","
            + String(bms2.message1.packVoltage) + ","
            + String(bms2.message2.packDCL) + ","
            + String(bms2.message2.packCCL) + ","
            + String(bms2.message2.simulatedSOC) + ","
            + String(bms2.message2.highTemp) + ","
            + String(bms2.message2.lowTemp) + ","
            + String(bms2.message3.SOC) + ","
            + String(bms2.message3.packResistance) + ","
            + String(bms2.message3.packOpenVoltage) + ","
            + String(bms2.message3.packAmpHours)
          );
        }

    static int countAllFiles(){
      File thisDataFile = SD.open("/");
      int thisFileNumber = countFiles(thisDataFile);
      thisDataFile.close();
      return thisFileNumber;
    }

    static int countFiles(File dir) {
      int numFiles = 0;
      while(true) {
        File entry = dir.openNextFile();
        if (! entry) {
          // No more files
          break;
        }
        if (entry.isDirectory()) {
          // Use recursion to count how many files are in the folder
          numFiles += countFiles(entry);
        } else {
          // Count this as a file
          numFiles++;
        }
        entry.close();
      }
      return numFiles;
    }

private:
    volatile unsigned int getExecutionDelay() override { return 100000; }
    VCU *pVCU;
    String titleString = "";
};

// Update the IMU variables. This class has ownership of the LSM object since nothing else should care about using it.
class UpdateImu : public VCUTask {
public:
    void execute() override {
        lsm.getEvent(&*accel, &*mag, &*gyro, &*temperature); // Pass the pointers to the function as references?
        //*mag = _mag;
    }

    explicit UpdateImu(sensors_event_t *a, sensors_event_t *m, sensors_event_t *g, sensors_event_t *temperatureIn) {
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
    volatile unsigned int getExecutionDelay() override { return 100100; }

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
    volatile unsigned int getExecutionDelay() override { return 20300; }

    int *sdcpSamples; // Pointer to the array containing the shutdown circuit positive side current sensor ADC samples
    int *sdcnSamples;
    int sdcOversamplingLocation = 0;
};

// Periodically print the status of the vehicle over serial
class PrintStatus : public VCUTask {
public:
    void execute() override {
        // int runRate = pVCU->getLoopsCompleted() * (1000000.0f / (getExecutionDelay() + 1.0f)); // avoid division by 0
        // Serial.print(String(">").concat("        "));
        // Serial.print(String(">").concat(VCU::getSwitchStates()));
        // Serial.print(String(" a1:").concat(pVCU->getApps1()));
        // Serial.print(String(" a2:").concat(pVCU->getApps2()));
        // Serial.print(String(" bse:").concat(pVCU->getBse()));
        // Serial.print(String(" a1f:").concat(String(pVCU->getApps1AdcFloat(), 6)));
        // Serial.print(String(" a2f:").concat(String(pVCU->getApps2AdcFloat(), 6)));
        // Serial.print(String(" a1T:").concat(String(pVCU->getApps1Travel(), 6)));
        // Serial.print(String(" a2T:").concat(String(pVCU->getApps2Travel(), 6)));
        // Serial.print(String(" APPS:").concat(String(pVCU->getCheckedAndScaledAppsValue(), 6)));
        // Serial.print(String(" sdcp:").concat(pVCU->getSdcCurrentPos()));
        // Serial.print(String(" sdcn:").concat(pVCU->getSdcCurrentNeg()));
        // Serial.print(String(" rr:").concat(runRate));
        // String x = String(String(pVCU->imuAccel.acceleration.x).concat(","));
        // String y = String(String(pVCU->imuAccel.acceleration.y).concat(","));
        // String z = String(pVCU->imuAccel.acceleration.z);
        // Serial.print(String(" axyz:").concat(x).concat(y).concat(z));
        // Serial.println("");

        // Serial.print(String("").concat(String(pVCU->getApps1AdcFloat()*100.0, 6)));
        // Serial.print(String(",").concat(String(pVCU->getApps2AdcFloat()*100.0, 6)));
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
    CarStateType handleUserInput(VCU& vcu, CarState::TransitionType requestedTransition) override {
      if(requestedTransition == TransitionType::ENABLE){
        // Perform startup sequence, checking either here or in constructor for that state for safety checks
        // Serial.println("OffState transitioning to PrechargeState");
        return CarStateType::PRECHARGE_STATE;
      }
      //Serial.println("Continuing being off");
      return SAME_STATE;
    }
    void enter(VCU& vcu) override {
      // TODO: Set the car safe here
      vcu.setSafe();
      vcu.writePinStates();
      vcu.disableMotorController();
      stateStartTime = millis();
    }
    CarStateType update(VCU& vcu) override {
      // There's not much to update here when the car is off.
      return CarStateType::SAME_STATE;
    }
    CarStateType getStateType() override {
      return CarStateType::OFF_STATE;
    }
    long getTimeInState() override {
      return millis() - stateStartTime;
    }
  private:
    long stateStartTime;
};

class PrechargeState: public CarState {
  public:
    PrechargeState(){
      Serial.println("Creating PrechargeState");
    };
    CarStateType handleUserInput(VCU& vcu, CarState::TransitionType requestedTransition) override {
      if(requestedTransition == TransitionType::DISABLE){
        // Halt startup
        return CarStateType::OFF_STATE;
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
    CarStateType update(VCU& vcu) override {
      // Validate that the car should be precharging. At some point, change the state to OnState.
      long currentPrechargeTime = millis() - this->prechargeStartTime;
      if(currentPrechargeTime < MAX_PRECHARGE_TIME){
        bool prechargeCanContinue = prechargeIsSafeToContinue(vcu);
        if(prechargeCanContinue){
          bool carIsPrecharged = false;
          if(currentPrechargeTime > 4000){ // TODO: THIS IS FAKE
            carIsPrecharged = getPrechargeIsCompleted(vcu);
          }
          if(currentPrechargeTime > 5000){
            Serial.println("Precharge Timeout 2");
            return CarStateType::OFF_STATE;
          }

          // TODO: Determine if the car is ready to enable here
          if(carIsPrecharged){
            Serial.println("Precharge ending, CAR TURNING ON");
            return CarStateType::ON_STATE;
          }
          return CarStateType::SAME_STATE;
        }else{
          Serial.println("Car was determined to be dangerous during precharge loop");
          return CarStateType::OFF_STATE;
        }
      }
      // TODO: Log timeout message here
      Serial.println("Precharge timeout");
      return CarStateType::OFF_STATE;
    }

    CarStateType getStateType() override {
      return CarStateType::PRECHARGE_STATE;
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

    // TODO: THIS IS FAKEish
    static bool getPrechargeIsCompleted(VCU& vcu){
      if(LastMotorControllerBusVoltageReportTime == 0){
        Serial.println("We never got a bus voltage report.");
        return false; // We never got a MC bus voltage
      }
      if(millis() > LastMotorControllerBusVoltageReportTime + 100L ){
        Serial.println("The bus voltage is too old.");
        return false; // The bus voltage is too old
      }
      if(VCU::getHumanReadableVoltage(LastMotorControllerBusVoltageRaw) < 120.0){
        Serial.println("The bus voltage is too low.");
        return false;
      }
      Serial.println("PRECHARGE IS COMPLETE");
      return true;
    };

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

    long getTimeInState() override {
      return millis() - prechargeStartTime;
    }
  private:
    long prechargeStartTime;
    const long MAX_PRECHARGE_TIME = 5000; // milliseconds
};

class OnState: public CarState {
  public:
    OnState(){
      Serial.println("Creating OnState");
    };
    CarStateType handleUserInput(VCU& vcu, CarState::TransitionType requestedTransition) override {
      if(requestedTransition == TransitionType::DISABLE){
        // TODO: Log a request to stop the car here
        return CarStateType::OFF_STATE;
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
    CarStateType update(VCU& vcu) override {
      // TODO: Perform runtime safety validation here
      long currentRuntime = millis() - carStartTime;
      if(currentRuntime > MAXIMUM_ON_TIME){
        return CarStateType::OFF_STATE; // TODO: THIS IS FAKE
      }else{
        return CarStateType::SAME_STATE;
      }
    }

    CarStateType getStateType() override {
      return CarStateType::ON_STATE;
    }
    long getTimeInState() override {
      return millis() - carStartTime;
    }
  private:
    long carStartTime;
};

class VehicleStateTask : public VCUTask {
public:
    void execute() override {
      CarState::CarStateType state = pVCU->currentCarState->update(*pVCU);

      if(state != CarState::CarStateType::SAME_STATE){
        Serial.println("Change");

        if(state == CarState::CarStateType::ON_STATE){
          // The startup/precharge state machine has requested a transition to ON
          Serial.println("Change to on");
          noInterrupts();
          delete pVCU->currentCarState;
          pVCU->currentCarState = new OnState();
          interrupts();
          pVCU->currentCarState->enter(*pVCU);
        }else if(state == CarState::CarStateType::OFF_STATE){
          Serial.println("Change to off");
          // The startup/precharge state machine has requested a transition to OFF
          noInterrupts();
          delete pVCU->currentCarState;
          pVCU->currentCarState = new OffState();
          interrupts();
          pVCU->currentCarState->enter(*pVCU);
        }else{
          Serial.println("Unhandled state transition!");
        }
      }
    }

    explicit VehicleStateTask(VCU *vcuPointer) {
        this->pVCU = vcuPointer;
        pVCU->currentCarState = new OffState();
        pVCU->currentCarState->enter(*pVCU);
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
      byte valueToWrite = (byte)VCU::mapf(value, 0.0, 1.0, 0, 127);
      Wire.write(valueToWrite);
      Wire.endTransmission();
      Serial.println("Setting pump to:" + String(valueToWrite));
    }
  }
private:
    volatile unsigned int getExecutionDelay() override { return 1001000; }
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
    volatile unsigned int getExecutionDelay() override { return 50200; }
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
    volatile unsigned int getExecutionDelay() override { return 100100; }
    VCU *pVCU;
};


class RequestCanData : public VCUTask {
public:
    void execute() override {
      for(int i = 0; i < MC_REGISTER_ARRAY_LENGTH; i++){
        if(mcObjects[i].registerLocation == 0){
          break;
        }
        pVCU->requestMotorControllerRegisterOnce(mcObjects[i].registerLocation);
      }
    }
    explicit RequestCanData(VCU *aVCU) {
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
      if (VCU::right()) {
          Serial.println("Disabling motor controller torque");
          pVCU->disableMotorController();
      }
      //      if(pVCU->center()){
      //        Serial.println("Setting torque to " + String((int) genericTorque));
      //        pVCU->setTorqueValue((int) genericTorque);
      //      }else{
      //        pVCU->setTorqueValue(0);
      //      }
      if(pVCU->carIsOn()){
        pVCU->setTorqueValue((int) genericTorque);
      }else{
        pVCU->setTorqueValue(0);
      }
      //      if(pVCU->center()){
      //        Serial.println("Requesting RPM");
      //        pVCU->requestRPM();
      //        pVCU->requestMotorPosition();
      //      }

      // thisDataFile.close();
      // int fileCount = SaveDataToSD::countAllFiles();
      // Serial.println("Files: " + String(fileCount));
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
            serialLineReceived.toLowerCase(); // toLowerCase() modifies the string in-place, it doesn't return a new string
            Serial.println("Received: " + serialLineReceived);
            // Dump EEPROM
            if (pVCU->strContains(serialLineReceived, "dumpeeprom")) {
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
            if (pVCU->strContains(serialLineReceived, "seteeprom")) {
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
