/*
 * Kennesaw State University
 * KSU Motorsports & Electric Vehicle Team
 * KS3-E VCU Library
 * Clinton Flowers, 2019-1-13
 */

#ifndef VCU_H
#define VCU_H

//#define F_CPU 120000000
//#define __MK64FX512__
//#define BOARD "Teensy 3.5"
//#define __cplusplus 98L

#include <Arduino.h>
#include "CAN.h"
#include "EEPROM.h"
//#include <TimeLib.h>
#include <Wire.h>         // Used by IMU
#include <SD.h>
#include <HardwareSerial.h>

// Header file constants
#define OVERSAMPLING_BUFFER_SIZE 32
#define NONCRITICAL_OVERSAMPLING_BUFFER_SIZE 8
#define STATES_ARRAY_SIZE 8
#define TASKS_ARRAY_SIZE 10 // Should exactly match the number of task class elements

// Teensy VCU pinouts
#define SDC_RELAY_PIN 25
#define PRECHARGE_PIN 26
#define DISCHARGE_PIN 27
#define CHARGED_ACTIVE_PIN 28
#define MOTOR_CONTROLLER_ENABLE_PIN 12 // Originally A26, then bodge-wired to pin 12/MISO0
#define CAN_CS_PIN 6
#define CAN_INT_PIN 5
// Teensy sensor definitions
#define APPS_1_PIN A16 // 35
#define APPS_2_PIN A17 // 36
#define BSE_PIN A12   // 31 -- The BSE needs a pulldown resistor and ceramic capacitor added to it
#define SDC_NEGATIVE_CURRENT_PIN A11
#define SDC_POSITIVE_CURRENT_PIN A10
#define THERMISTOR_1_PIN A25     // The thermistors need pullup resistors (to 3.3v or aref) and likely capacitors (to gnd) // TODO: Unused?
#define THERMISTOR_2_PIN A22 // TODO: Unused?
// Teensy 5-way switch pin definitions
#define S_UP_PIN 24
#define S_DOWN_PIN 23
#define S_LEFT_PIN 30
#define S_RIGHT_PIN 29
#define S_CENTER_PIN 11

// Operational Constants
#define ADC_READ_RESOLUTION 12   // Default is 10. Above 12, read time increases drastically.
//#define DEFAULT_CAN_POLL_DELAY_TIME 10 // 10ms -> 100hz
#define CAN_BAUD_RATE 500000
#define MAXIMUM_PRECHARGE_TIME 2000000
#define DEFAULT_TASK_DELAY 1000000

#define EEPROM_BYTES 4096
//#define EEPROM_WDT_COUNT_ADDRESS 3      // The number of times the watchdog timeout has activated // TODO: Unused?
//#define EEPROM_WDT_TASK_ADDRESS 5       // What the last task that executed when a watchdog timeout occurred was // TODO: Unused?
//#define EEPROM_STARTS_COUNT_ADDRESS 32  // How many times a precharge sequence has intiated // TODO: Unused?


// I2C and IMU
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // required for adafruit sensors

//--Bamocar Regisers
#define regReadBamocarData        0x3D
#define regSysModeBits            0x51
#define regLogicReadBits          0xD8
#define regTorqueCommanded        0x90
#define regSignedRPM              0xA8
#define regSignedPackCurrent      0x20
#define regSignedPhaseCurrent     0x5F
#define regRPMmax                 0xC8

class VCUTask {
protected:
    friend class VCU;

    virtual void execute() = 0;

    boolean shouldExecute(volatile unsigned long currentTime) {
        //long randomTime = random(50);
        if (currentTime > lastExecutionTime + getExecutionDelay()) {
            lastExecutionTime = currentTime;
            return true;
        } else {
            return false;
        }
    };

    VCUTask() = default;;
    ~VCUTask() = default;;

private:
    virtual volatile unsigned int getExecutionDelay() = 0;
    volatile unsigned int lastExecutionTime = 0;
    // Not sure if these should be volatile. Need to somehow guarantee bug-free task execution code.
    // TODO: Check code with a linter
};

// TODO
struct VCUStates {
    // The point of VCU States is to provide a structure which the car state validator (validateStates()) can construct and report on the SD card
    // The integers represent codes which are to be determined. They may be a series of bit flags, or follow HTTP status code-like numeric ranges... teensy int32 = +2147483648
    // These states are not used to directly operate the car, only to validate that its operation is correct and report it or shut it down (Maybe rename to e.g. VCUValidationStates)
public:
    int states[STATES_ARRAY_SIZE]; // TODO: Unused?
    // Byte 1 (index 0) contains Master Tractive System status
    // Byte 2 (index 1) contains BMS 1 status
    // Byte 3 (index 2) contains BMS 2 status
    // Byte 4 (index 3) contains Motor Controller status
    // Byte 5 (index 4) contains Motor status
    // Byte 6 (index 5) contains Coolant status
};

class VCU; // Forward-declaration - define VCU class's existence (below) so that CarState and CanMessage can point to objects of type VCU

class CarState {
  public:
    CarState() = default;;
    virtual ~CarState(){};
    enum TransitionType {
        ENABLE,
        DISABLE,
    };
    enum NewStateType {
        OFF_STATE,
        PRECHARGE_STATE,
        ON_STATE,
        SAME_STATE,
    };
    virtual void enter(VCU& vcu);
    virtual void exit(){};
    virtual NewStateType handleUserInput(VCU& vcu, TransitionType requestedTransition);
    virtual NewStateType update(VCU& vcu);
};

class VCU {
public:
    bool sdcActive;                 // Whether the onboard relay enabling the shutdown circuit is (requested to be) active (note that it also requires BSPD)
    bool prechargeActive;           // Whether the signal to enable precharge is active
    bool dischargeActive;           // Whether the signal to enable discharge is active (is this active high or low?)
    bool chargedActive;             // Whether the signal to enable the fourth relay is active (for when precharge is complete)
    bool motorControllerActive;     // Whether the motor controller "enable" relay is active
    VCUStates recentVCUStates;      // The most recent VCU states evaluated by validateStates() // TODO: Unused?
    sensors_event_t imuTemperature{};
    sensors_event_t imuGyro{};
    sensors_event_t imuMag{};
    sensors_event_t imuAccel{};  // The latest LSM 9DOF IMU event

    // Debug variables


    // Safety-critical methods
    void init();

    void vcuLoop();   // Main execution loop of Teensy code, updates sensor fields, performs safety checks, calls periodic code (like CAN) as needed
    void setSafe();       // Immediately turn off all dangerous pins -- IRequires writeStates() be called afterward by user code
    void setDangerous();  // Turn on all dangerous settings -- Requires writeStates() be called afterward by user code
    void writePinStates();

    void requestStartup();
    void requestShutdown();

    // BAMOCAR Data Methods
    float getBamocarRPM(); // TODO: Unused?
    float getBamocarBusVoltage(); // TODO: Unused?

    // Primary I/O methods
    int getApps1();
    int getApps2();
    int getBse();
    float getApps1Float();
    float getApps2Float(); // TODO: Unused?
    float getBseFloat(); // TODO: Unused?
    int getSDCCurrentPos();
    int getSDCCurrentNeg();

    // 5-way Switch Methods
    String getSwitchStates();
    bool anySwitchPressed();
    static bool up();
    static bool down();
    static bool left();
    static bool right();
    static bool center();

    // Debug information
    long getLoopsCompleted();   // Not idempotent
    int getTasksSize(); // TODO: Unused?

    // Utility methods
    static float mapf(float, float, float, float, float);

    static bool strContains(const String &, const String &);
protected:
    // Friend classes (usually VCUTask classes) which can access the protected VCU variables here
    friend class DoSpecificDebugThing;
    friend class VehicleStateTask;
    friend class OffState; // Friended to allow writing values to stop motor controller
    friend class CanMessage; // Friended to allow writing and reading values to the motor controller
    // OLD
    friend class StartupSequenceStateMachine;

    // I/O variables
    int apps1samples[OVERSAMPLING_BUFFER_SIZE]{};
    int apps2samples[OVERSAMPLING_BUFFER_SIZE]{};
    int bseSamples[OVERSAMPLING_BUFFER_SIZE]{};
    int sdcpSamples[NONCRITICAL_OVERSAMPLING_BUFFER_SIZE]{};
    int sdcnSamples[NONCRITICAL_OVERSAMPLING_BUFFER_SIZE]{};
    int oversamplingLocation{}; // TODO: Unused?
    int maxAdcValue{};

    // Debug variables
    int loopsCompleted{};

    // Safety-critical private variables
    // TODO: Datestamps of the last messages which arrived

    // CAN Methods
    float getTorqueRegisterValue();

    void setTorqueValue(int someValue);
    void disableMotorController();
    void enableMotorController();
    void requestRPM();
    void requestMotorPosition();
    
//    void blockingRequest(unsigned char *bytes, int messageLength); // TODO: add a blocking CAN read

    // Other operational variables
    VCUTask **tasks = new VCUTask *[TASKS_ARRAY_SIZE];

    // Operational methods
    //VCUStates validateStates(); // OLD. Called in vcuLoop(); Performs safety-related checks on the car to determine/validate car state(s).

    CarState* currentCarState;
    long lastUserTransitionRequest = 0;
    const long MINIMUM_TRANSITION_DELAY = 3000; // TODO: Make like 10S
private:
    void requestTransition(CarState::TransitionType transitionType);
    // CAN Variables
    void sendMotorControllerMessage(unsigned char *bytes, int messageLength);
    unsigned int canMotorControllerAddress = 528;
    unsigned char canArrEnableMotorController[3] = {regSysModeBits, 0x00 & 0xFF, 0x00 & 0xFF}; // TODO: Unused?
    unsigned char canArrDisableMotorController[3] = {regSysModeBits, 0x04 & 0xFF, 0x00 & 0xFF};
    unsigned char canArrStopMotorController[3] = {regTorqueCommanded, 0, 0}; // Sets the torque to 0
    unsigned char canRequestRPM[3] = {regReadBamocarData, 0xA8, 0x00}; // Request RPM once
    unsigned char canRequestMotorPosition[3] = {regReadBamocarData, 0x6D, 0x00}; // Request RPM once
};

// A class to define the data associated with a CAN message, and keep track of data such as the last time that message was received
//class CanMessage {
//  public:
//    virtual unsigned char* getCanPacket();
//    virtual unsigned short getCanPacketLength();
//    bool isStale(){
//      if(timeout == -1){
//        return false;
//      }else{
//        long elapsedTime = millis() - lastReceivedTimestamp;
//        return elapsedTime > timeout;
//      }
//    }
//    virtual void send(VCU& vcu){};
//
//    CanMessage() = default;;
//    ~CanMessage() = default;;
//  protected:
//    volatile long lastReceivedTimestamp = 0;
////    unsigned int messageLength = 3;
////    unsigned char message[3];// = {0x90, 0, 0}; // Sets the torque to 0
//    const long timeout = -1; // Override this variable for values which must be made fresh
//};



//class CanMessages{
//  public:
//    CanMessages(){
//      Messages::DisableMotorController = new DisableMotorController(),
//    };
//    enum Messages{
//      DisableMotorController = **CanMessage,
//    };
//};

#endif
