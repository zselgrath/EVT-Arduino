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
#include "CAN.h" // Originally by Sandeep Mistry, modified to use SPI1. Used for CAN1, going to the BMS's and Motor Controller
#include "FlexCAN.h" // Used for CAN0, going to the front of the car
#include "EEPROM.h"
//#include <TimeLib.h>
#include <Wire.h>         // Used by IMU
#include <SD.h>
#include <HardwareSerial.h>

// Header file operational constants
#define OVERSAMPLING_BUFFER_SIZE 32
#define NONCRITICAL_OVERSAMPLING_BUFFER_SIZE 8
#define STATES_ARRAY_SIZE 8
#define TASKS_ARRAY_SIZE 14 // Should exactly match the number of task class elements
#define ADC_READ_RESOLUTION 12   // Default is 10. Above 12, read time increases drastically.

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
#define START_BUTTON_PIN 15

//BMS message ID's
#define BMS_CUSTOM_MESSAGE_1 0x03b
#define BMS_CUSTOM_MESSAGE_2 0x3cb
#define BMS_CUSTOM_MESSAGE_3 0x6b2
#define BMS_CUSTOM_MESSAGE_4 0x1806E5F4
#define BMS_CUSTOM_MESSAGE_5 0x1806E9F4
#define BMS_CUSTOM_MESSAGE_6 0x18FF50E5

//#define DEFAULT_CAN_POLL_DELAY_TIME 10 // 10ms -> 100hz
#define CAN_BAUD_RATE 500000
#define MAXIMUM_PRECHARGE_TIME 2000000
#define DEFAULT_TASK_DELAY 1000000

#define EEPROM_BYTES 4096
//#define EEPROM_WDT_COUNT_ADDRESS 3      // The number of times the watchdog timeout has activated // TODO: Unused?
//#define EEPROM_WDT_TASK_ADDRESS 5       // What the last task that executed when a watchdog timeout occurred was // TODO: Unused?
//#define EEPROM_STARTS_COUNT_ADDRESS 32  // How many times a precharge sequence has intiated // TODO: Unused?

#define PEDAL_DEADZONE 0.05f

// I2C and IMU
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // required for adafruit sensors
#define PUMP_DIGITAL_POTENTIOMETER 0x2F // MCP4018

//--Bamocar Regisers
#define regReadBamocarData        0x3D
#define regSysModeBits            0x51
#define regLogicReadBits          0xD8
//#define regTorqueCommanded        0x90
//#define regSignedRPM              0xA8
// #define regSignedPackCurrent      0x20
// #define regSignedPhaseCurrent     0x5F
#define regRPMmax                 0xC8

#define MC_REGISTER_ARRAY_LENGTH 16
#define MC_VALUES_HOLDER_SIZE 4

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

class VCU; // Forward-declaration - define VCU class's existence (below) so that CarState can point to objects of type VCU

class CarState {
  public:
    CarState() = default;;
    virtual ~CarState(){};
    enum TransitionType {
        ENABLE,
        DISABLE,
    };
    enum CarStateType {
        SAME_STATE,
        OFF_STATE,
        PRECHARGE_STATE,
        ON_STATE,
    };
    virtual void enter(VCU& vcu);
    virtual void exit(){};
    virtual CarStateType handleUserInput(VCU& vcu, TransitionType requestedTransition);
    virtual CarStateType update(VCU& vcu);
    virtual CarStateType getStateType();
    virtual long getTimeInState();
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
    bool acceleratorPedalIsPlausible();
    bool carIsOn();
    bool carIsOff();
    bool carCanStart();

    void requestStartup();
    void requestShutdown();

    // BAMOCAR Data Methods
    float getBamocarRPM(); // TODO: Unused?
    float getBamocarBusVoltage(); // TODO: Unused?

    // Primary I/O methods
    float getCheckedAndScaledAppsValue(); // The average of the two APPS travels, or 0.0 if implausible
    float getDeratedTorqueSetpoint();
    int getSDCCurrentPos();
    int getSDCCurrentNeg();
    float getBseTravel();

    void sendDashText(String text);

    // 5-way Switch Methods
    String getSwitchStates();
    bool anySwitchPressed();
    static bool up();
    static bool down();
    static bool left();
    static bool right();
    static bool center();
    static bool start();

    // Debug information
    long getLoopsCompleted();   // Not idempotent
    int getTasksSize(); // TODO: Unused?

    // Utility methods
    static float mapf(float in, float fromMin, float fromMax, float toMin, float toMax);

    static bool strContains(const String &, const String &);
protected:
    // Friend classes (usually VCUTask classes) which can access the protected VCU variables here
    friend class DoSpecificDebugThing;
    friend class VehicleStateTask;
    friend class OffState; // Friended to allow writing values to stop motor controller
    friend class OnState;
    friend class PrintStatus;
    friend class HandlePumpAndFan;
    friend class RequestCanData;
    // OLD
    friend class StartupSequenceStateMachine;

    // I/O variables
    int apps1samples[OVERSAMPLING_BUFFER_SIZE];
    int apps2samples[OVERSAMPLING_BUFFER_SIZE];
    int bseSamples[OVERSAMPLING_BUFFER_SIZE];
    int sdcpSamples[NONCRITICAL_OVERSAMPLING_BUFFER_SIZE]{};
    int sdcnSamples[NONCRITICAL_OVERSAMPLING_BUFFER_SIZE]{};
    int oversamplingLocation{}; // TODO: Unused?
    int maxAdcValue{};

    // Debug variables
    int loopsCompleted{};

    // Safety-critical private variables
    // TODO: Datestamps of the last messages which arrived

    // CAN Methods
    int calculateTorqueRegisterValueForWrite();

    void setTorqueValue(int someValue);
    void disableMotorController();
    void enableMotorController();
    void requestMotorControllerRegisterOnce(byte registerAddress);
    void requestRPM();

//    void blockingRequest(unsigned char *bytes, int messageLength); // TODO: add a blocking CAN read

    // Other operational variables
    VCUTask **tasks = new VCUTask *[TASKS_ARRAY_SIZE];

    // Operational methods
    //VCUStates validateStates(); // OLD. Called in vcuLoop(); Performs safety-related checks on the car to determine/validate car state(s).
    CarState* currentCarState;
    long lastUserTransitionRequest = 0;
    const long MINIMUM_TRANSITION_DELAY = 3000; // TODO: Make like 10 seconds

    // CAN Variables
    unsigned const int canDashAddress = 0x01;
    void sendMotorControllerMessage(unsigned char *bytes, int messageLength);
    unsigned const int canMotorControllerAddress = 528; // 0x210

    const byte MOTOR_CONTROLLER_ADDRESS_VOLTAGE_BUS = 0xEB;
    const byte MOTOR_CONTROLLER_ADDRESS_VOLTAGE_OUTPUT = 0x8A;
    const byte MOTOR_CONTROLLER_ADDRESS_CURRENT_PACK = 0x20;
    const byte MOTOR_CONTROLLER_ADDRESS_CURRENT_PHASE = 0x5F;
    const byte MOTOR_CONTROLLER_ADDRESS_MOTOR_TEMPERATURE = 0x49;
    const byte MOTOR_CONTROLLER_ADDRESS_IGBT_TEMPERATURE = 0x4A;
    const byte MOTOR_CONTROLLER_ADDRESS_SETPOINT_CURRENT = 0x22;
    const byte MOTOR_CONTROLLER_ADDRESS_SETPOINT_TORQUE = 0x90;
    const byte MOTOR_CONTROLLER_ADDRESS_RPM = 0xA8;
    const byte MOTOR_CONTROLLER_ADDRESS_MOTOR_POSITION = 0x6D;

    unsigned char canArrEnableMotorController[3] = {regSysModeBits, 0x00 & 0xFF, 0x00 & 0xFF};
    unsigned char canArrDisableMotorController[3] = {regSysModeBits, 0x04 & 0xFF, 0x00 & 0xFF};
    unsigned char canArrStopMotorController[3] = {MOTOR_CONTROLLER_ADDRESS_SETPOINT_TORQUE, 0, 0}; // Sets the torque to 0

    // unsigned char canRequestRPM[3] = {regReadBamocarData, 0xA8, 0x00}; // Request RPM once
    // unsigned char canRequestMotorPosition[3] = {regReadBamocarData, 0x6D, 0x00}; // Request position once
    // unsigned char canRequestMotorControllerCurrent[3] = {regReadBamocarData, 0xA8, 0x00}; // TODO
    // unsigned char canRequestMotorControllerVoltage[3] = {regReadBamocarData, 0xA8, 0x00}; // TODO
    // unsigned char canRequestMotorControllerTemperature[3] = {regReadBamocarData, 0xA8, 0x00}; // TODO
    // unsigned char canRequestMotorTemperature[3] = {regReadBamocarData, 0xA8, 0x00}; // TODO


private:
    void requestTransition(CarState::TransitionType transitionType);

    // ADC -> Samples array -> getApps1() -> getApps1ADCFloat() -> getApps1Travel() -> getCheckedAndScaledAppsValue() -> getDeratedTorqueSetpoint()
    int getApps1(); // Average of the sampling array
    int getApps2();
    int getBse();
    float getApps1ADCFloat(); // Sampling array as a percentage between 0V and AREF
    float getApps2ADCFloat();
    float getBseADCFloat();
    float getApps1Travel(); // The scaled percentage between 0V and AREF which is then mapped to some calibrated throttle value
    float getApps2Travel();
    FlexCAN canBus{};
};

struct
{

  union
  {
    byte bytes[8];
    struct
    {
      int16_t packCurrent;
      int16_t packVoltage;
      int8_t checkSum;
      int8_t padding[3];
    };
  } x03B;

  union
  {
    byte bytes [8];
    struct vars
    {
      int8_t packDCL;
      int8_t packCCL;
      int8_t blank;
      int8_t simulatedSOC;
      int8_t highTemp;
      int8_t lowTemp;
      int8_t checkSum;
      int8_t padding;
    };
  } x3CB;

  union
  {
    byte bytes [8];
    struct vars
    {
      int8_t customFlag;
      int8_t SOC;
      int16_t packResistance;
      int16_t packOpenVoltage;
      int8_t packAmpHours;
      int8_t checkSum;
    };
  } x6B2;

  union
  {
    byte bytes [8];
    struct vars
    {
      //TODO
    };
  } xxx5F4;

  union
  {
    byte bytes [8];
    struct vars
    {
      //TODO
    };
  } xxx9F4;

  union
  {
    byte bytes [8];
    struct vars
    {
      //TODO
    };
  } xxx0E5;

} bms;

// Motor Controller CAN registers, received value holders, etc.
// These are static so that they can be accessed from the receive interrupt.
// We really need better structures for these, e.g., HashMaps and classes.
static byte importantMotorControllerRegisters[MC_REGISTER_ARRAY_LENGTH];
static String motorControllerRegisterNames[MC_REGISTER_ARRAY_LENGTH];
static long motorControllerRegisterLastReadTimes[MC_REGISTER_ARRAY_LENGTH];
static byte receivedMotorControllerValues[MC_REGISTER_ARRAY_LENGTH][MC_VALUES_HOLDER_SIZE];



#endif
