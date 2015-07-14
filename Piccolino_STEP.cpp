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
#include "Piccolino_STEP.h"

void Piccolino_STEP::moveTo(long absolute)
{
    if (_targetPos != absolute)
    {
	_targetPos = absolute;
	computeNewSpeed();
	// compute new n?
    }
}

void Piccolino_STEP::move(long relative)
{
    moveTo(_currentPos + relative);
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean Piccolino_STEP::runSpeed()
{
    // Dont do anything unless we actually have a step interval
    if (!_stepInterval)
	return false;

    unsigned long time = micros();
    // Gymnastics to detect wrapping of either the nextStepTime and/or the current time
    unsigned long nextStepTime = _lastStepTime + _stepInterval;
    if (   ((nextStepTime >= _lastStepTime) && ((time >= nextStepTime) || (time < _lastStepTime)))
	|| ((nextStepTime < _lastStepTime) && ((time >= nextStepTime) && (time < _lastStepTime))))

    {
	if (_direction == DIRECTION_CW)
	{
	    // Clockwise
	    _currentPos += 1;
	}
	else
	{
	    // Anticlockwise  
	    _currentPos -= 1;
	}
	step(_currentPos & 0x7); // Bottom 3 bits (same as mod 8, but works with + and - numbers) 

	_lastStepTime = time;
	return true;
    }
    else
    {
	return false;
    }
}

long Piccolino_STEP::distanceToGo()
{
    return _targetPos - _currentPos;
}

long Piccolino_STEP::targetPosition()
{
    return _targetPos;
}

long Piccolino_STEP::currentPosition()
{
    return _currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void Piccolino_STEP::setCurrentPosition(long position)
{
    _targetPos = _currentPos = position;
    _n = 0;
    _stepInterval = 0;
}

void Piccolino_STEP::computeNewSpeed()
{
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

    long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1)
    {
	// We are at the target and its time to stop
	_stepInterval = 0;
	_speed = 0.0;
	_n = 0;
	return;
    }

    if (distanceTo > 0)
    {
	// We are anticlockwise from the target
	// Need to go clockwise from here, maybe decelerate now
	if (_n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
		_n = -stepsToStop; // Start deceleration
	}
	else if (_n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
		_n = -_n; // Start accceleration
	}
    }
    else if (distanceTo < 0)
    {
	// We are clockwise from the target
	// Need to go anticlockwise from here, maybe decelerate
	if (_n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
		_n = -stepsToStop; // Start deceleration
	}
	else if (_n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
		_n = -_n; // Start accceleration
	}
    }

    // Need to accelerate or decelerate
    if (_n == 0)
    {
	// First step from stopped
	_cn = _c0;
	_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
	// Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
	_cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
	_cn = max(_cn, _cmin); 
    }
    _n++;
    _stepInterval = _cn;
    _speed = 1000000.0 / _cn;
    if (_direction == DIRECTION_CCW)
	_speed = -_speed;

#if 0
    Serial.println(_speed);
    Serial.println(_acceleration);
    Serial.println(_cn);
    Serial.println(_c0);
    Serial.println(_n);
    Serial.println(_stepInterval);
    Serial.println(distanceTo);
    Serial.println(stepsToStop);
    Serial.println("-----");
#endif
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if we are still running to position
boolean Piccolino_STEP::run()
{
    if (runSpeed())
	computeNewSpeed();
    return true;
}

Piccolino_STEP::Piccolino_STEP() {
    // nothing here -- all done in begin
}


void Piccolino_STEP::begin(uint8_t addr, uint8_t motor)
{

    Wire.begin(); 


  _iowrite(CMD_CP0,0x00); //configure PORT0 as output

  _iowrite(CMD_CP1,0x00); //configure PORT1 as output
  _iowrite(CMD_PI0,0x00); //configure PORT0 normal polarity
  _iowrite(CMD_PI1,0x00); //configure PORT1 normal polarity
  _iowrite(CMD_WR0,0x00); //configure PORT0 all off
  _iowrite(CMD_WR1,0x00); //configure PORT0 all off  

    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = 0xff;
    _lastStepTime = 0;
    if(motor==1) { // first half of PCA9555
        _pin[0] = 0;
        _pin[1] = 1;
        _pin[2] = 2;
        _pin[3] = 3;
        _pin[4] = 4;
        _pin[5] = 5;
    } else { // second half
        _pin[0] = 8;
        _pin[1] = 9;
        _pin[2] = 10;
        _pin[3] = 11;
        _pin[4] = 12;
        _pin[5] = 13;        
    }

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    int i;
    
  _io_addr=addr;
  _io_int=0;

    enableOutputs();
}

Piccolino_STEP::Piccolino_STEP(void (*forward)(), void (*backward)())
{
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 1.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = 0xff;
    _lastStepTime = 0;
    _pin[0] = 0;
    _pin[1] = 0;
    _forward = forward;
    _backward = backward;

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

}

void Piccolino_STEP::setSpeed(float speed)
{
    if (_maxSpeed != speed)
    {
	_maxSpeed = speed;
	_cmin = 1000000.0 / speed;
	// Recompute _n from current speed and adjust speed if accelerating or cruising
	if (_n > 0)
	{
	    _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
	    computeNewSpeed();
	}
    }
}

void Piccolino_STEP::setAcceleration(float acceleration)
{
    if (acceleration == 0.0)
	return;
    if (_acceleration != acceleration)
    {
	// Recompute _n per Equation 17
	_n = _n * (_acceleration / acceleration);
	// New c0 per Equation 7
	_c0 = sqrt(2.0 / acceleration) * 1000000.0;
	_acceleration = acceleration;
	computeNewSpeed();
    }
}


float Piccolino_STEP::speed()
{
    return _speed;
}

void Piccolino_STEP::setOutputPins(uint8_t mask)
{
    uint8_t numpins = 2;
    uint8_t i;
    for (i = 0; i < numpins; i++)
	_digitalWrite(_pin[i], (mask & (1 << i)) ? HIGH : LOW );
}

// Dynamically modify MS1, MS2, MS3 to set step resolution
void Piccolino_STEP::setStep(uint8_t step) 
{
    _digitalWrite(_pin[3],LOW);
    _digitalWrite(_pin[4],LOW);
    _digitalWrite(_pin[5],LOW);

    switch(step) {
        case STEP_FULL: // nothing to do
        break;
        case STEP_HALF: 
            _digitalWrite(_pin[3],HIGH);
        break;

        case STEP_SIXTEEN: 
            _digitalWrite(_pin[5],HIGH);       
        case STEP_EIGHT: 
            _digitalWrite(_pin[3],HIGH);
        case STEP_QUARTER: 
            _digitalWrite(_pin[4],HIGH);
        break;
    }

}

void Piccolino_STEP::step(uint8_t step)
{
    // _pin[0] is step, _pin[1] is direction
    setOutputPins(_direction ? 0b11 : 0b01); // step HIGH
    // Caution 200ns setup time 
    // Delay the minimum allowed pulse width
    delayMicroseconds(_minPulseWidth);
    setOutputPins(_direction ? 0b10 : 0b00); // step LOW

}

void    Piccolino_STEP::enableOutputs()
{
    _pinMode(_pin[0], OUTPUT);
    _pinMode(_pin[1], OUTPUT);
    _pinMode(_pin[2], OUTPUT); 
    _pinMode(_pin[3], OUTPUT);
    _pinMode(_pin[4], OUTPUT);
    _pinMode(_pin[5], OUTPUT);
}

void Piccolino_STEP::setMinPulseWidth(unsigned int minWidth)
{
    _minPulseWidth = minWidth;
}

// Blocks until the target position is reached and stopped
void Piccolino_STEP::runToPosition()
{
    while (_speed != 0 || distanceToGo() != 0)
	run();
}

boolean Piccolino_STEP::runSpeedToPosition()
{
    if (_targetPos == _currentPos)
	return false;
    if (_targetPos >_currentPos)
	_direction = DIRECTION_CW;
    else
	_direction = DIRECTION_CCW;
    return runSpeed();
}

// Blocks until the new target position is reached
void Piccolino_STEP::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}

void Piccolino_STEP::stop()
{
    if (_speed != 0.0)
    {    
	long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
	if (_speed > 0)
	    move(stepsToStop);
	else
	    move(-stepsToStop);
    }
}

void Piccolino_STEP::disable(void)
{
    _digitalWrite(_pin[2],HIGH);
}

void Piccolino_STEP::enable(void)
{
    
    _digitalWrite(_pin[2],LOW);
}

// i2c write shortcut
void Piccolino_STEP::_iowrite (uint8_t cmd, uint8_t data)
{

  uint8_t twbrbackup = TWBR;
  TWBR = 18; // upgrade to 400KHz!

  Wire.beginTransmission(_io_addr);
  Wire.write(cmd);
  Wire.write(data); 
  Wire.endTransmission();

  TWBR = twbrbackup;
}

// i2c read shortcut
uint8_t Piccolino_STEP::_ioread(uint8_t cmd)
{

    uint8_t twbrbackup = TWBR;
    TWBR = 18; // upgrade to 400KHz!

    Wire.beginTransmission(_io_addr);
    Wire.write(cmd); 
    Wire.endTransmission();
    Wire.requestFrom((int)_io_addr, 1);
    
    TWBR = twbrbackup;

    return Wire.read();
}

// local alias
void Piccolino_STEP::pinMode(uint8_t pin, char mode)
{
    _pinMode(pin, mode);
}

// sets pin (0 - 15) for input or output 
void Piccolino_STEP::_pinMode(uint8_t pin, char mode)
{
   uint8_t dir,p;
   p=pin&0x7;
  
   if(pin<8) 
    dir = _ioread(CMD_CP0);
    else 
    dir = _ioread(CMD_CP1);
  

  if (mode == INPUT)
    dir |= 1 << p; 
  else 
    dir &= ~(1 << p);
  
  if(pin<8)
    _iowrite(CMD_CP0,dir);
  else
    _iowrite(CMD_CP1,dir);

}

// local alias
void Piccolino_STEP::digitalWrite(uint8_t pin, char data)
{
    _digitalWrite(pin, data);
}
 
// change state of pin (0 - 15)
void Piccolino_STEP::_digitalWrite(uint8_t pin, char data)
{
   uint8_t io,p;

   p=pin&0x7;
  
   //read current port conf
   if(pin<8)
    io = _ioread(CMD_WR0); 
    else
    io = _ioread(CMD_WR1);
  

  if (data == HIGH)
    io |= 1 << p; 
  else 
    io &= ~(1 << p);
  
  if(pin<8)
    _iowrite(CMD_WR0,io);
  else
    _iowrite(CMD_WR1,io);
}
 
// local alias
uint8_t Piccolino_STEP::digitalRead(uint8_t pin)
{
    return _digitalRead(pin);
}

// read state of pin (0 - 15)
uint8_t Piccolino_STEP::_digitalRead(uint8_t pin)
{
   uint8_t io,p;

   p=pin&0x7;
  
   if(pin<8)
    io = _ioread(CMD_RD0);
   else 
    io = _ioread(CMD_RD1);
  

  return (io >> p) & 0x1;
}
