// Pins 0 and 1 are used by the USB serial interface when connected to a PC
// Only pins 2 and 3 can accept simple external interrupts (though more complex interrupts are possible on all pins)
// When using the Ethernet Shield, Pins 11-13 are used for communication, Pin 10 is Ethernet SS, and Pin 4 is SD Card SS.
// So, can't use 0, 1, 4, 10-13.
// Of the remaining pins 2,3,5,6,7,8,9 - 2 & 3 are for interrupts, and 3,5,6,9 can handle PWM.

// Motor Controller Pins (output)
#define MOTOR_UP_PWM 5
#define MOTOR_DOWN_PWM 6
#define MOTOR_ENABLE 7

// Switch input pins (INPUT_PULLUP)
#define USER_SWITCH_UP 8
#define USER_SWITCH_DOWN 9

// Shaft Encoder Pins (input)
// Two staggared latching hall effect sensors
#define SHAFT_A 2
#define SHAFT_B 3
volatile unsigned long shaft_a_last_signal_interrupt = 0;
volatile unsigned long shaft_b_last_signal_interrupt = 0;
volatile bool shaft_a_has_fired = false;
volatile bool shaft_b_has_fired = false;

#define LOWERED_TARGET_POSITION 0
#define RAISED_TARGET_POSITION 8800 // in shaft encoder count
#define ACCELERATION 0.0625 // motor percent (0-100) per millisecond (1 = 100ms, 0.5 = 200ms, 0.25 = 400ms, 0.125 = 800ms, 0.0625 = 1600ms...)
#define MAX_MOTOR_DRIVE 255
unsigned long last_loop_micros = 0;
volatile long current_position_interrupt = 0; // in shaft encoder count
long last_position = 0; // used to calculate velocity
long target_position = LOWERED_TARGET_POSITION;
double motor_percent = 0; // between -1 and 1
bool home_position_is_known = false;
unsigned long started_homing_micros = 0;
bool navigating = false;
unsigned long started_navigating_micros = 0;
unsigned long micros_since_last_moved = 0;
unsigned long now;
bool stalled = false;

void setup() {
  Serial.begin(115200);

  // Set the PWM base frequency for pins D9 and D10 to 7.8 kHz. 
  // The default frequency is 490 Hz and audible with such a big motor.
  //TCCR1B = (TCCR1B & 0b11110000) | 0b00001010;

  pinMode(MOTOR_UP_PWM, OUTPUT);
  pinMode(MOTOR_DOWN_PWM, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);

  pinMode(USER_SWITCH_UP, INPUT_PULLUP); // Must use internal pullup resistor since the switch leads float when the circuit is open.
  pinMode(USER_SWITCH_DOWN, INPUT_PULLUP);

  pinMode(SHAFT_A, INPUT); // Our voltage divider from 12v -> 5v on the shaft encoder provides pulldown.
  pinMode(SHAFT_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(SHAFT_A), shaftAInterrupt, CHANGE); // Since the hall effect sensors are latching, and they can be out of phase with eachother, we only look for a CHANGE edge
  attachInterrupt(digitalPinToInterrupt(SHAFT_B), shaftBInterrupt, CHANGE);

  // TODO - read currentPosition from file
  // If current position is not known, seek to bottom.

  hardStop();

  Serial.println("Waiting for 24v rail to stabilize...");
  delay(2000);

  Serial.println("Running...");
}

void loop() {
  // This gets a little tricky...
  // At full speed, shaft encoder interrupts fire about 4 times in every 3ms.
  // If we don't delay here, this loop runs as fast as it can, which is about once every 2ms.
  // That means we spend some non-zero fraction of time in noInterrupts(), and we miss current_position updates during that time. This causes our current_position to drift badly.
  // By only running our loop every ~100ms, we reduce the amount of time inside noInterrupts() and reduce our shaft encoder drift rate.
  delay(100);

  now = micros();

  if (last_loop_micros == 0) {
    last_loop_micros = now;
    return;
  }

  unsigned long elapsed_time_micros = (unsigned long)(now - last_loop_micros);
  if (elapsed_time_micros == 0) {
    return; // We can't do math when dividing by zero
  }
  double elapsed_time_millis = elapsed_time_micros / 1000.0 ;
  last_loop_micros = now;

//  Serial.print("elapsed time ms ");
//  Serial.println(elapsed_time_micros / 1000.0);
  double accel_amount = (ACCELERATION / 100.0) * elapsed_time_millis;

//Serial.println(accel_amount);

  noInterrupts();
  unsigned long shaft_a_last_signal = shaft_a_last_signal_interrupt;
  long current_position = current_position_interrupt;
  interrupts();

  if (current_position != last_position) {
    micros_since_last_moved = now;
  }

  double current_velocity = ((double)(current_position - last_position) / elapsed_time_millis); // encoder count per ms
  last_position = current_position;

  if (stalled) {
    Serial.println("Stalled.");
    hardStop();
    return;
  }

  // Apply acceleration curve and calculate motor_percent
  if (digitalRead(USER_SWITCH_UP) == LOW) {
//    Serial.print("Going Up...\n");
      target_position = RAISED_TARGET_POSITION;
  }
  else if (digitalRead(USER_SWITCH_DOWN) == LOW) {
//    Serial.print("Going Down...\n");
    target_position = LOWERED_TARGET_POSITION;
  }
  else {
//    Serial.print("Stopping for Siri...\n");
    target_position = current_position; // should be whichever is closer, RAISED or LOWERED
    //slowMotor(accel_amount);
  }

  Serial.println("cur : tar position ");
  Serial.println(current_position);
  Serial.println(target_position);

  if (home_position_is_known) {
    double ms_to_zero_motor = (abs(motor_percent) * 100.0) / ACCELERATION;
    if (current_velocity < 0){
      ms_to_zero_motor = ms_to_zero_motor * 1.5;  
    }
    double pos_delta = abs(target_position - current_position);
    double fudge = 10;
    if (current_velocity == 0) {
      if (target_position > (current_position + fudge)) {
        speedMotor(accel_amount);
      }
      else if(target_position < (current_position - fudge)) {
        speedMotor(-accel_amount);
      }
      else {
        slowMotor(accel_amount);
      }
    }
    else {
      double ms_to_target_range = (pos_delta) / abs(current_velocity);
      double ms_to_target = pos_delta / abs(current_velocity);
      
      if (ms_to_zero_motor >= ms_to_target_range) {
        if (ms_to_zero_motor >= ms_to_target) {
          slowMotor(accel_amount * 1.35);
        }
        else {
          slowMotor(accel_amount * 1.35);
        }
      }
      else {
        if (target_position > (current_position + fudge)) {
          speedMotor(accel_amount);
        }
        else if(target_position < (current_position - fudge)) {
          speedMotor(-accel_amount);
        }
        else {
          slowMotor(accel_amount);
        }
      }
    }
  }
  else {
    Serial.println("homing...");

    if (started_homing_micros == 0) {
      started_homing_micros = now;
    }

    if ((unsigned long)(now - started_homing_micros) > 200000) { // give the motor 200ms to start moving before we check for it having stopped
      Serial.println("Watching for stop ");
//      Serial.println("now / shaft");
//Serial.println(current_position);
//      Serial.println(now);
//      Serial.println(shaft_a_last_signal);
//      Serial.println(now - shaft_a_last_signal);

      if (current_velocity == 0) {
//      if ((unsigned long)(now - shaft_a_last_signal) > 100000) {
        Serial.println("Found bottom stop");
        hardStop();
        delay(100);
        motor_percent = 0.25; // back of the very bottom
        driveMotor();
        delay(5000);
        hardStop();
        delay(100);
        
        noInterrupts();
        current_position_interrupt = 0;
        interrupts();
        home_position_is_known = true;
        started_homing_micros = 0;
        last_loop_micros = 0;
      }
    }
    else {
      motor_percent = -0.25;
    }
  }

//it is stalling when going from full raised to lowered
//dropping several hundred shaft encoder count and overshooting physically while still thinking position is ~+100

//even going down at homing speed, it was at 8792 and dropped -8717 before hitting the bottom stop

//  if (home_position_is_known && navigating && (unsigned long)(now - started_navigating_micros) > 300000 && (unsigned long)(now - micros_since_last_moved) > 300000 && current_velocity == 0) {
//    stalled = true;
//    hardStop();
//  }
  
  // Drive the motor
  driveMotor();
  
  // TODO - periodically write current position to disk
//NEXT:
// consider breaking this code into some libraries 
}

void enableMotion(bool enable) {
  // On the BTS7960, BOTH the left_enable and right_enable pins must be high for anything to move.
  // So they are hard-wired together on the circuit board.
  if (enable) {
    digitalWrite(MOTOR_ENABLE, HIGH);
  }
  else {
    digitalWrite(MOTOR_ENABLE, LOW);
  }
}

void driveMotor() {
//  Serial.print("DM: ");
//  Serial.println(motor_percent);
  if (motor_percent == 0) {
    if (navigating) {
      navigating = false;
      if (target_position == LOWERED_TARGET_POSITION) {
        Serial.println("Finished navigating to bottom position. Finding home.");
        home_position_is_known = false;
      }
    }
    
    hardStop();
  }
  else {
    if (motor_percent > 0) {
      analogWrite(MOTOR_UP_PWM, motor_percent * MAX_MOTOR_DRIVE);
      analogWrite(MOTOR_DOWN_PWM, 0);
    }
    else {
      analogWrite(MOTOR_DOWN_PWM, -motor_percent * MAX_MOTOR_DRIVE);
      analogWrite(MOTOR_UP_PWM, 0);
    }
    enableMotion(true);
  }
}

void slowMotor(double accel_amount) {
//  Serial.print("SM: ");
//  Serial.println(accel_amount);
  
  if (motor_percent < 0) {
    motor_percent = motor_percent + accel_amount;
    if (motor_percent > 0) {
      motor_percent = 0;
    }
  }
  else {
    motor_percent = motor_percent - accel_amount;
    if (motor_percent < 0) {
      motor_percent = 0;
    }
  }
}

void speedMotor(double accel_amount) {
  if (navigating == false) {
    Serial.println("Starting navigation.");
    navigating = true;
    started_navigating_micros = now;
  }
   
  motor_percent = motor_percent + accel_amount;
  
  if (motor_percent > 1) {
    motor_percent = 1;
  }
  
  if (motor_percent < -1) {
    motor_percent = -1;
  }
}

void hardStop() { 
  analogWrite(MOTOR_UP_PWM, 0);
  analogWrite(MOTOR_DOWN_PWM, 0);
  enableMotion(false);

  noInterrupts();
  long current_position = current_position_interrupt;
  interrupts();
  last_position = current_position;
  target_position = current_position;
  motor_percent = 0;
}

void raiseTV() {

}

void lowerTV() {
  // If we know where we are, lower quickly to close to zero.
  // Then locateBottomStop()
}

void locateBottomStop() {
  // Lower very slowly, monitoring continually if we still have motion.
  // If motion stops, then immediately hardStop()
  // Set current_position to 0
}

// Interrupt Service Routines (ISR)
// - Only one ISR can run at a time, other interrupts will be executed after the current one finishes.
// - millis() will not increment during the ISR, but it's initial value is valid.
// - micros() will increment for about 1-2 ms before becoming irratic.
// - Don't use the serial port.
// - global values set inside ISRs should be declared volatile
// - multi-byte globals set inside ISRs should be bracketed when reading with noInterrupts() and interrupts() to make reads atomic.

void shaftAInterrupt() {
  // The shaft encoders are latching hall effect sensors, positioned about 25 degrees apart. So the sequence during will rotation will be
  // A -> short interval -> B -> long interval -> repeat
  // or
  // A -> long interval -> B -> short interval -> repeat
  //
  // To determine direction of rotation, record the time each interrupt fires.
  // When interrupt for SHAFT_A fires, measure the time between the last time it fired and current time.
  //   If SHAFT_B has fired in the second half of that interval, then we are moving from B to A.
  //   Otherwise, we are moving from A to B.
  // The inverse of the above is true for SHAFT_B interrupts, but we don't need to determine direction more than once per rotation.

  unsigned long now = micros();
  
  if (shaft_a_has_fired && shaft_b_has_fired) {
    unsigned long interval_since_last_a = (unsigned long)(now - shaft_a_last_signal_interrupt);
    unsigned long interval_since_last_b = (unsigned long)(now - shaft_b_last_signal_interrupt);
    
    if (interval_since_last_b < (interval_since_last_a / 2)) { // Down
      current_position_interrupt -= 1;
    }
    else { // Up
      current_position_interrupt += 1;
    }
  }

  shaft_a_last_signal_interrupt = now;
  shaft_a_has_fired = true;
}

void shaftBInterrupt() {
  shaft_b_last_signal_interrupt = micros();
  shaft_b_has_fired = true;
}
