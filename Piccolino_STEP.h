// Piccolino_STEP significantly improves on the standard Arduino Stepper library in several ways:
// Supports acceleration and deceleration
// Supports multiple simultaneous steppers, with independent concurrent stepping on each stepper
// API functions never delay() or block
// Very slow speeds are supported
// Extensive API
// I2C Communication (up to 8 configurable addresses 0x20 - 0x27 for a total of 16 stepper motors)
// Dynamic Step resolution
// Additional 4 I/O available to the user
//
// A big chunk of this library is borrowed from AccelStepper by Mike McCauley
// Located at http://www.open.com.au/mikem/arduino/AccelStepper
// Modified by Alex Sardo (wctek.com) for Piccolino's PicoStepper (http://piccolino.rocks)
//

#ifndef Piccolino_STEP_h
#define Piccolino_STEP_h

//#include <stdlib.h>
#include <Wire.h>
#include <Arduino.h>

// These defs cause trouble on some versions of Arduino
#undef round

#define CMD_RD0 0x00 
#define CMD_RD1 0x01 
#define CMD_WR0 0x02 
#define CMD_WR1 0x03 
#define CMD_PI0 0x04 
#define CMD_PI1 0x05 
#define CMD_CP0 0x06 
#define CMD_CP1 0x07 

#define INVERT 1
#define NORMAL 0

#define MOTOR1 1
#define MOTOR2 2

#define STEP_FULL    0
#define STEP_HALF    1
#define STEP_QUARTER 2
#define STEP_EIGHT   3
#define STEP_SIXTEEN 4

#define E1      6
#define E2      7
#define E3      14
#define E4      15

class Piccolino_STEP
{
  public:

    void enable(void);
    void disable(void);

    void _digitalWrite(uint8_t pin, char mode);
    uint8_t _digitalRead(uint8_t pin);
    void _pinMode(uint8_t pin, char mode);
    void digitalWrite(uint8_t pin, char mode);
    uint8_t digitalRead(uint8_t pin);
    void pinMode(uint8_t pin, char mode);

    Piccolino_STEP();

    // Set Step resolution
    // Valid values are: STEP_FULL, STEP_HALF, STEP_QUARTER, STEP_EIGHT, STEP_SIXTEEN
    void setStep(uint8_t step);

    void begin(uint8_t addr, uint8_t motor);

    // Alternate Constructor which will call your own functions for forward and backward steps. 
    // You can have multiple simultaneous steppers, all moving
    // at different speeds and accelerations, provided you call their run()
    // functions at frequent enough intervals. Current Position is set to 0, target
    // position is set to 0. MaxSpeed and Acceleration default to 1.0.
    // Any motor initialization should happen before hand, no pins are used or initialized.
    // \param[in] forward void-returning procedure that will make a forward step
    // \param[in] backward void-returning procedure that will make a backward step
    Piccolino_STEP(void (*forward)(), void (*backward)());
    
    // Set the target position. The run() function will try to move the motor
    // from the current position to the target position set by the most
    // recent call to this function. Caution: moveTo() also recalculates the speed for the next step. 
    // If you are trying to use constant speed movements, you should call setSpeed() after calling moveTo().
    // param[in] absolute The desired absolute position. Negative is
    // anticlockwise from the 0 position.
    void    moveTo(long absolute); 

    // Set the target position relative to the current position
    // param[in] relative The desired position relative to the current position. Negative is
    // anticlockwise from the current position.
    void    move(long relative);

    // Poll the motor and step it if a step is due, implementing
    // accelerations and decelerations to acheive the target position. You must call this as
    // frequently as possible, but at least once per minimum step interval,
    // preferably in your main loop.
    // \return true if the motor is at the target position.
    boolean run();

    // Poll the motor and step it if a step is due, implmenting a constant
    // speed as set by the most recent call to setSpeed(). You must call this as
    // frequently as possible, but at least once per step interval,
    // \return true if the motor was stepped.
    boolean runSpeed();

    // Sets the maximum permitted speed. the run() function will accelerate
    // up to the speed set by this function.
    // param[in] speed The desired maximum speed in steps per second. Must
    // be > 0. Caution: Speeds that exceed the maximum speed supported by the processor may
    // Result in non-linear accelerations and decelerations.
    void    setSpeed(float speed);

    // Sets the acceleration and deceleration parameter.
    // param[in] acceleration The desired acceleration in steps per second
    // per second. Must be > 0.0. This is an expensive call since it requires a square 
    // root to be calculated. Dont call more ofthen than needed
    void    setAcceleration(float acceleration);

    // The most recently set speed
    // return the most recent speed in steps per second
    float   speed();

    // The distance from the current position to the target position.
    // return the distance from the current position to the target position
    // in steps. Positive is clockwise from the current position.
    long    distanceToGo();

    // The most recently set target position.
    // \return the target position
    // in steps. Positive is clockwise from the 0 position.
    long    targetPosition();

    // The currently motor position.
    // \return the current motor position
    // in steps. Positive is clockwise from the 0 position.
    long    currentPosition();  

    // Resets the current position of the motor, so that wherever the motor
    // happens to be right now is considered to be the new 0 position. Useful
    // for setting a zero position on a stepper after an initial hardware
    // positioning move.
    // Has the side effect of setting the current motor speed to 0.
    // param[in] position The position in steps of wherever the motor
    // happens to be right now.
    void    setCurrentPosition(long position);  
    
    // Moves the motor at the currently selected constant speed (forward or reverse) 
    // to the target position and blocks until it is at
    // position. Dont use this in event loops, since it blocks.
    void    runToPosition();

    // Runs at the currently selected speed until the target position is reached
    // Does not implement accelerations.
    // return true if it stepped
    boolean runSpeedToPosition();

    // Moves the motor to the new target position and blocks until it is at
    // position. Dont use this in event loops, since it blocks.
    // param[in] position The new target position.
    void    runToNewPosition(long position);

    // Sets a new target position that causes the stepper
    // to stop as quickly as possible, using to the current speed and acceleration parameters.
    void stop();

    // Disable motor pin outputs by setting them all LOW
    // Depending on the design of your electronics this may turn off
    // the power to the motor coils, saving power.
    // This is useful to support Arduino low power modes: disable the outputs
    // during sleep and then reenable with enableOutputs() before stepping
    // again.
    void    disableOutputs();

    // Enable motor pin outputs by setting the motor pins to OUTPUT
    // mode. Called automatically by the constructor.
    void    enableOutputs();

    // Sets the minimum pulse width allowed by the stepper driver. The minimum practical pulse width is 
    // approximately 20 microseconds. Times less than 20 microseconds
    // will usually result in 20 microseconds or so.
    // param[in] minWidth The minimum pulse width in microseconds. 
    void    setMinPulseWidth(unsigned int minWidth);

    // Sets the enable pin number for stepper drivers.
	// 0xFF indicates unused (default).
    // Otherwise, if a pin is set, the pin will be turned on when 
    // enableOutputs() is called and switched off when disableOutputs() 
    // is called.
    // param[in] enablePin Arduino digital pin number for motor enable
    // sa setPinsInverted
    void    setEnablePin(uint8_t enablePin = 0xff);

protected:

    // brief Direction indicator
    // Symbolic names for the direction the motor is turning
    typedef enum
    {
	DIRECTION_CCW = 0,  //< Clockwise
        DIRECTION_CW  = 1   //< Counter-Clockwise
    } Direction;

    // Forces the library to compute a new instantaneous speed and set that as
    // the current speed. It is called by
    // the library:
    // after each step
    // after change to maxSpeed through setMaxSpeed()
    // after change to acceleration through setAcceleration()
    // after change to target position (relative or absolute) through
    // move() or moveTo()
    void           computeNewSpeed();

    // Low level function to set the motor output pins
    // bit 0 of the mask corresponds to _pin[0]
    // bit 1 of the mask corresponds to _pin[1]
    // You can override this to impment, for example serial chip output insted of using the
    // output pins directly
    virtual void   setOutputPins(uint8_t mask);

    // Called to execute a step. Only called when a new step is
    // required. Subclasses may override to implement new stepping
    // interfaces. The default calls step1(), step2(), step4() or step8() depending on the
    // number of pins defined for the stepper.
    // param[in] step The current step phase number (0 to 7)
    virtual void   step(uint8_t step);

private:

    uint8_t _io_addr, _io_int;
    void _iowrite(uint8_t cmd, uint8_t data);
    uint8_t _ioread(uint8_t cmd);

    uint8_t        _pin[6];

    // Whether the _pins is inverted or not
    uint8_t        _pinInverted[2];

    // The current absolution position in steps.
    long           _currentPos;    // Steps

    // The target position in steps. The Piccolino_STEP library will move the
    // motor from the _currentPos to the _targetPos, taking into account the
    // max speed, acceleration and deceleration
    long           _targetPos;     // Steps

    // The current motos speed in steps per second
    // Positive is clockwise
    float          _speed;         // Steps per second

    // The maximum permitted speed in steps per second. Must be > 0.
    float          _maxSpeed;

    // The acceleration to use to accelerate or decelerate the motor in steps
    // per second per second. Must be > 0
    float          _acceleration;
    float          _sqrt_twoa; // Precomputed sqrt(2*_acceleration)

    // The current interval between steps in microseconds.
    // 0 means the motor is currently stopped with _speed == 0
    unsigned long  _stepInterval;

    // The last step time in microseconds
    unsigned long  _lastStepTime;

    // The minimum allowed pulse width in microseconds
    unsigned int   _minPulseWidth;

    // Is the direction pin inverted?
    //bool           _dirInverted; // Moved to _pinInverted[1]

    // Is the step pin inverted?
    //bool           _stepInverted; // Moved to _pinInverted[0]

    // Is the enable pin inverted?
    bool           _enableInverted;

    // Enable pin for stepper driver, or 0xFF if unused.
    uint8_t        _enablePin;

    // The pointer to a forward-step procedure
    void (*_forward)();

    // The pointer to a backward-step procedure
    void (*_backward)();

    // The step counter for speed calculations
    long _n;

    // Initial step size in microseconds
    float _c0;

    // Last step size in microseconds
    float _cn;

    // Min step size in microseconds based on maxSpeed
    float _cmin; // at max speed

    // Current direction motor is spinning in
    boolean _direction; // 1 == CW

};

#endif 
