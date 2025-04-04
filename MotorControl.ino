// Pin Definitions
const int limitSwitch1 = 2;  // Interrupt pin for limit switch 1
const int limitSwitch2 = 3;  // Interrupt pin for limit switch 2
const int buttonPin = 4;     // Push button pin
const int vibsenPin = 5;     // Vibration sensor pin
const int directionPin = 6;  // Motor direction control pin
const int pwmPin = 9;        // Motor PWM control pin

// Variables
volatile bool limitSwitchTriggered = false;  // Flag for limit switch interrupt
bool limitSwitchTriggeredPrev = false;
bool motorRunning = false;       // Motor running state
bool motorDirection = true;      // true = forward, false = reverse
bool buttonPressed = false;      // Debounced button state
bool buttonPressedPrev = false;  // Debounced button state previous
bool vibsenTriggered = false;    // VibrationSensor state
bool endPos = false;
bool f_Stall = false;
bool f_TriggerStuck = false;
unsigned long lastDebounceTime = 0;  // Button debounce timer
unsigned long lastDebounceTimeFall = 0;  // Limit Switch Failing interupt debounce timer
unsigned long lastDebounceTimeRise = 0;  // Limit Switch Rising interupt debounce timer
unsigned long lastMotorTime = 0;         // Motor delay timer
const unsigned long debounceDelay = 50;  // Debounce delay in ms
const unsigned long motorDelay = 750;    // motor delay in ms
const unsigned long userTime = 1500;     // Time to detect user pushing instead of mechanical skipping
unsigned long lastLimitSwitch1TriggeredTime = 0;
unsigned long lastLimitSwitch2TriggeredTime = 0;
unsigned long lastLimitSwitch1LeftTime = 0;
unsigned long lastLimitSwitch2LeftTime = 0;
unsigned long motorRunningTime = 0;
int motorSpeed = 255;
void setup() {
  // Configure pins
  pinMode(limitSwitch1, INPUT);
  pinMode(limitSwitch2, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(vibsenPin, INPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  // Stop motor initially
  stopMotor();

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(limitSwitch1), limitSwitch1Handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(limitSwitch2), limitSwitch2Handler, CHANGE);


  // Check for homing/ending position at the starting cycle only
  if (digitalRead(limitSwitch2) == LOW) {
    endPos = true;  // Detected that the tray is homed, edge case can happen
    runMotorReverse();
    motorRunning = true;
    motorDirection = !motorDirection;
    lastMotorTime = millis();
    while (lastMotorTime <= motorDelay) {
      lastMotorTime = millis();
    }
    // Direction of travel is not correct if any limit switch is still being LOW even after the motor is running
    if (digitalRead(limitSwitch2) == LOW) {
      stopMotor();  // Stop motor
      motorRunning = true;
      motorDirection = !motorDirection;  // Reverse direction for next run
      if (motorDirection) {
        runMotorForward();
      } else {
        runMotorReverse();
      }
    }
  }
  // The tray is in the middle of nowhere, start homing sequence
  else if ((digitalRead(limitSwitch1) == HIGH) && (digitalRead(limitSwitch2) == HIGH)) {
    runMotorReverse();
    motorRunning = true;
    motorDirection = !motorDirection;
  }
}

void loop() {

  // Handle button press
  if (((digitalRead(buttonPin) == LOW) && !f_TriggerStuck) && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    buttonPressed = true;
    buttonPressedPrev = true;
  }
  if (digitalRead(buttonPin) == HIGH) {
    f_TriggerStuck = false;
    if(buttonPressed == true){
    buttonPressed = false;
    }
  }

  if (((digitalRead(vibsenPin) == LOW) && !f_TriggerStuck) && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    vibsenTriggered = true;
    buttonPressedPrev = true;
  }
  if (digitalRead(vibsenPin) == LOW && motorRunning) {
    vibsenTriggered = false;
    f_TriggerStuck = false;
  }

  if ((buttonPressed || vibsenTriggered) && !f_TriggerStuck && !motorRunning) {
    buttonPressed = false;    // Reset button state
    vibsenTriggered = false;  // Reset vib sensor state
    // Start motor based on direction
    motorRunning = true;
    if (motorDirection) {
      runMotorForward();
    } else {
      runMotorReverse();
    }
    lastMotorTime = millis();
  }
  if (motorRunning) {
    if (millis() - lastMotorTime > motorDelay) {
      lastMotorTime = millis();
      // Direction of travel is not correct if any limit switch is still being LOW even after the motor is running
      if ((digitalRead(limitSwitch1) == LOW) || (digitalRead(limitSwitch2) == LOW)) {
        stopMotor();  // Stop motor
        motorRunning = true;
        motorDirection = !motorDirection;  // Reverse direction for next run
        if (motorDirection) {
          runMotorForward();
        } else {
          runMotorReverse();
        }
      }
    }
  }

  // Stop motor if limit switch is triggered
  if (limitSwitchTriggered) {
    // this logic included the debouncing, since the flag motorRunning would be FALSE after the first pulse
    if (motorRunning) {
      stopMotor();
      limitSwitchTriggered = false;      // Reset interrupt flag
      motorDirection = !motorDirection;  // Reverse direction for next run
      limitSwitchTriggeredPrev = true;
    }
  }

  if (motorRunning) {
    if (millis() - motorRunningTime > 5000) {
      stopMotor();
      motorRunningTime = millis();
      motorDirection = !motorDirection;  // Reverse direction for next run
      f_Stall = true;
    }
  }
  else {
    motorRunningTime = millis();
    f_Stall = false;
  }
}

// Interrupt handler for limit switch 1
void limitSwitch1Handler() {
  // Handle LOW edge: Limit Switch triggered
  if (digitalRead(limitSwitch1) == LOW && ((millis() - lastDebounceTimeFall) > debounceDelay)) {
    lastDebounceTimeFall = millis();
    limitSwitchTriggered = true;  // Set interrupt flag
    lastLimitSwitch1TriggeredTime = millis();
    if(digitalRead(buttonPin) == LOW || digitalRead(vibsenPin) == LOW) {
      f_TriggerStuck = true;
    }
  }
  // Handle HIGH edge: Tray left the Limit Switch
  else if (digitalRead(limitSwitch1) == HIGH && ((millis() - lastDebounceTimeRise) > debounceDelay)) {
    lastDebounceTimeRise = millis();
    lastLimitSwitch1LeftTime = millis();
    if ( ((lastLimitSwitch1LeftTime - lastLimitSwitch1TriggeredTime) > userTime) && !motorRunning ) {
      motorRunning = true;
      runMotorForward();
    }
  }
}

// Interrupt handler for limit switch 2
void limitSwitch2Handler() {
  // Handle LOW edge: Limit Switch triggered
  if (digitalRead(limitSwitch2) == LOW && (millis() - lastDebounceTimeFall) > debounceDelay) {
    lastDebounceTimeFall = millis();
    limitSwitchTriggered = true;  // Set interrupt flag
    lastLimitSwitch2TriggeredTime = millis();
    if(digitalRead(buttonPin) == LOW || digitalRead(vibsenPin) == LOW) {
      f_TriggerStuck = true;
    }
  } 
  // Handle HIGH edge: Tray left the Limit Switch
  else if (digitalRead(limitSwitch2) == HIGH && (millis() - lastDebounceTimeRise) > debounceDelay) {
    lastDebounceTimeRise = millis();
    lastLimitSwitch2LeftTime = millis();
    if ((lastLimitSwitch2LeftTime - lastLimitSwitch2TriggeredTime) > userTime && (!motorRunning)) {
      motorRunning = true;
      runMotorReverse();
    }
  }
}

// Function to stop the motor
void stopMotor() {
  digitalWrite(directionPin, LOW);  // Set direction
  analogWrite(pwmPin, 0);           // Set PWM to 0
  motorRunning = false;             // Motor is not running
}

// Function to run motor forward
void runMotorForward() {
  digitalWrite(directionPin, HIGH);       // Set direction
  analogWrite(pwmPin, 255 - motorSpeed);  // Set PWM to 50% (adjust as needed)
}

// Function to run motor reverse
void runMotorReverse() {
  digitalWrite(directionPin, LOW);  // Set direction
  analogWrite(pwmPin, motorSpeed);  // Set PWM to 50% (adjust as needed)
}
