#include <Bounce2.h>  // Include the Bounce2 library


/*
#define STEP_PIN 3        // Pin for stepper motor step
#define DIR_PIN 4         // Pin for stepper motor direction
#define INCREASE_BTN_PIN 6 // Pin for the increase speed button
#define DECREASE_BTN_PIN 7 // Pin for the decrease speed button
#define DIR_BTN_PIN 8     // Pin for direction toggle button
#define START_STOP_BTN_PIN 9 // Pin for the start/stop button (not needed anymore)

#define MAX_SPEED 500     // Maximum speed (steps per second)
#define MIN_SPEED 10      // Minimum speed (steps per second)
#define ACCELERATION_TIME 5000 // Time for full acceleration (milliseconds)

// Stepper speed control variables
unsigned long stepInterval = 1000;   // Initial step interval (microseconds)
int currentSpeed = 100;              // Current speed (steps per second)
int targetSpeed = 100;               // Target speed (steps per second)
bool direction = true;               // Motor direction (true for clockwise, false for counterclockwise)

// Time tracking for acceleration (software)
unsigned long previousMillis = 0;    // For acceleration timing
unsigned long accelerationStartTime = 0; // When acceleration starts

// Motor running state (initially set to true to start motor immediately)
bool motorRunning = true;

// Button debounce objects
Bounce increaseBtnDebouncer = Bounce();  // Debouncer for increase button
Bounce decreaseBtnDebouncer = Bounce();  // Debouncer for decrease button
Bounce dirBtnDebouncer = Bounce();  // Debouncer for direction button

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(INCREASE_BTN_PIN, INPUT_PULLUP);
  pinMode(DECREASE_BTN_PIN, INPUT_PULLUP);
  pinMode(DIR_BTN_PIN, INPUT_PULLUP);

  // Initialize debouncers for buttons
  increaseBtnDebouncer.attach(INCREASE_BTN_PIN);
  increaseBtnDebouncer.interval(50);  // Set debounce interval to 50ms
  decreaseBtnDebouncer.attach(DECREASE_BTN_PIN);
  decreaseBtnDebouncer.interval(50);  // Set debounce interval to 50ms
  dirBtnDebouncer.attach(DIR_BTN_PIN);
  dirBtnDebouncer.interval(50);  // Set debounce interval to 50ms

  // Set initial direction (clockwise or counterclockwise)
  digitalWrite(DIR_PIN, direction);

  // Initialize Timer2 for step pulse generation
  initTimer2();

  // Start serial for debugging
  Serial.begin(9600);

  // Start motor immediately (no need for a start/stop button)
  accelerationStartTime = millis(); // Start acceleration immediately
}

void loop() {
  unsigned long currentMillis = millis();

  // Update button states and handle debouncing
  increaseBtnDebouncer.update();
  decreaseBtnDebouncer.update();
  dirBtnDebouncer.update();

  // Read buttons with debounced states
  if (increaseBtnDebouncer.fell()) {
    // Increase speed
    if (targetSpeed < MAX_SPEED) {
      targetSpeed += 10;  // Increase by 10 steps per second
      Serial.print("Increased Speed: ");
      Serial.println(targetSpeed);
    }
    // Do not reset acceleration start time, keep it continuous
  }

  if (decreaseBtnDebouncer.fell()) {
    // Decrease speed
    if (targetSpeed > MIN_SPEED) {
      targetSpeed -= 10;  // Decrease by 10 steps per second
      Serial.print("Decreased Speed: ");
      Serial.println(targetSpeed);
    }
    // Do not reset acceleration start time, keep it continuous
  }

  if (dirBtnDebouncer.fell()) {
    // Toggle direction
    direction = !direction;
    digitalWrite(DIR_PIN, direction); // Change direction
    if (direction) {
      Serial.println("Direction: Clockwise");
    } else {
      Serial.println("Direction: Counter-clockwise");
    }
  }

  // Handle acceleration timing, adjust speed gradually
  if (motorRunning) {
    unsigned long accelTimeElapsed = currentMillis - accelerationStartTime;

    // Accelerate gradually based on time (no ACCEL_RATE)
    if (accelTimeElapsed < ACCELERATION_TIME) {
      // Calculate speed change (from current speed to target speed)
      float speedChange = (float)(targetSpeed - currentSpeed) * (float)accelTimeElapsed / ACCELERATION_TIME;

      // Apply the calculated speed change
      currentSpeed = currentSpeed + (int)speedChange;
    } else {
      // If the acceleration time has passed, we should reach the target speed
      currentSpeed = targetSpeed;
    }

    // Adjust the step interval based on current speed
    stepInterval = 1000000 / currentSpeed; // Convert speed (steps per second) to interval (microseconds)
  }

  // Print the current speed and interval every second (limit serial prints)
  if (currentMillis - previousMillis >= 1000) {  // Print every second
    previousMillis = currentMillis;
    if (motorRunning) {
      Serial.print("Speed: ");
      Serial.print(currentSpeed);
      Serial.print(" steps/sec, Interval: ");
      Serial.println(stepInterval);
    }
  }
}

// Timer2 Interrupt Service Routine for Step Pulse Generation
ISR(TIMER2_COMPA_vect) {
  static unsigned long lastStepTime = 0;

  // Only generate step pulses if the motor is running
  if (motorRunning && (micros() - lastStepTime >= stepInterval)) {
    // Set the step pin high and low to generate the step pulse
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);          // Short pulse duration
    digitalWrite(STEP_PIN, LOW);
    lastStepTime = micros();
  }
}

// Initialize Timer2 for step pulse generation
void initTimer2() {
  // Set Timer2 in CTC mode (Clear Timer on Compare Match)
  TCCR2A = 0;  // Normal mode (no output waveform)
  TCCR2B = (1 << WGM22) | (1 << CS21);  // CTC mode, prescaler 8
  OCR2A = 100; // Set compare match value (adjust this to control the frequency)
  TIMSK2 |= (1 << OCIE2A);  // Enable Timer2 compare match interrupt
}

*/


#define STEP_PIN 3        // Pin for stepper motor step
#define DIR_PIN 4         // Pin for stepper motor direction
#define INCREASE_BTN_PIN 6 // Pin for the increase speed button
#define DIR_BTN_PIN 7     // Pin for direction toggle button
#define DECREASE_BTN_PIN 8 // Pin for the decrease speed button
#define START_STOP_BTN_PIN 9 // Pin for the start/stop button

#define ACCEL_RATE 1     // Rate of acceleration/deceleration (steps per tick???)
#define MAX_SPEED 5000     // Maximum speed (steps per second)
#define MIN_SPEED 0      // Minimum speed (steps per second)
#define ACCELERATION_TIME 1000 // Time for full acceleration (milliseconds)

// Stepper speed control variables
unsigned long stepInterval = 1000;   // Initial step interval (microseconds)
int currentSpeed =  0;              // Current speed (steps per second)
int targetSpeed =  0;               // Target speed (steps per second)
 int logicalTargetSpeed = targetSpeed;
bool direction = true;               // Motor direction (true for clockwise, false for counterclockwise)
bool motorRunning = false;           // Flag to indicate if the motor is running
bool motorShouldStop = true;


int speedChangePrClick = 100;

int speedWhenStopped = 0;

int lastTargetSpeed = 0;  

// Time tracking for acceleration (software)
unsigned long previousMillis = 0;    // For acceleration timing
unsigned long accelerationStartTime = 0; // When acceleration starts
unsigned long lastAccelerationUpdate = 0; // For controlling acceleration timing
 float speedFactor =0;

 

// Button debounce objects
Bounce increaseBtnDebouncer = Bounce();  // Debouncer for increase button
Bounce decreaseBtnDebouncer = Bounce();  // Debouncer for decrease button
Bounce dirBtnDebouncer = Bounce();  // Debouncer for direction button
Bounce startStopBtnDebouncer = Bounce(); // Debouncer for start/stop button

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(INCREASE_BTN_PIN, INPUT_PULLUP);
  pinMode(DECREASE_BTN_PIN, INPUT_PULLUP);
  pinMode(DIR_BTN_PIN, INPUT_PULLUP);
  pinMode(START_STOP_BTN_PIN, INPUT_PULLUP); // Pin for start/stop button

  // Initialize debouncers for buttons
  increaseBtnDebouncer.attach(INCREASE_BTN_PIN);
  increaseBtnDebouncer.interval(50);  // Set debounce interval to 50ms
  decreaseBtnDebouncer.attach(DECREASE_BTN_PIN);
  decreaseBtnDebouncer.interval(50);  // Set debounce interval to 50ms
  dirBtnDebouncer.attach(DIR_BTN_PIN);
  dirBtnDebouncer.interval(50);  // Set debounce interval to 50ms
  startStopBtnDebouncer.attach(START_STOP_BTN_PIN);
  startStopBtnDebouncer.interval(50);  // Set debounce interval to 50ms

  // Set initial direction (clockwise or counterclockwise)
  digitalWrite(DIR_PIN, direction);

  // Initialize Timer2 for step pulse generation
  initTimer2();

  // Start serial for debugging
  Serial.begin(115200);
}

void loop() {
  
  unsigned long currentMillis = millis();

  // Update button states and handle debouncing
  increaseBtnDebouncer.update();
  decreaseBtnDebouncer.update();
  dirBtnDebouncer.update();
  startStopBtnDebouncer.update();

  // Read buttons with debounced states
  if (increaseBtnDebouncer.fell()) {
    // Increase speed
    if (targetSpeed < MAX_SPEED) {
       targetSpeed += speedChangePrClick;  // Increase by 10 steps per second
      Serial.print("Increased Speed: ");
      Serial.println(targetSpeed);
    }
    accelerationStartTime = currentMillis; // Reset acceleration when speed changes
  }

  if (decreaseBtnDebouncer.fell()) {
    // Decrease speed
    if (targetSpeed > MIN_SPEED) {
       targetSpeed -= speedChangePrClick;  // Decrease by 10 steps per second
      Serial.print("Decreased Speed: ");
      Serial.println(targetSpeed);
    }
    accelerationStartTime = currentMillis; // Reset acceleration when speed changes
  }

  if (dirBtnDebouncer.fell()) {
    // Toggle direction
    direction = !direction;
    digitalWrite(DIR_PIN, direction); // Change direction
    if (direction) {
      Serial.println("Direction: Clockwise");
    } else {
      Serial.println("Direction: Counter-clockwise");
    }
  }


  if (startStopBtnDebouncer.fell()) {
          Serial.println("toggle motor start");

    // Toggle motor running state
    if (motorRunning == true) {
                Serial.println("try to stop");

      accelerationStartTime = currentMillis; // Reset acceleration time when starting
      //motorRunning = false;
      motorShouldStop = true;
      
    } else if (motorRunning == false) {
                      Serial.println("try to start");

      accelerationStartTime = currentMillis; // Reset acceleration time when starting
      motorRunning = true;
      motorShouldStop = false;
      //targetSpeed = speedWhenStopped;
    }
/*    motorRunning = !motorRunning;

    if (motorRunning) {
      Serial.println("Motor Started");
      accelerationStartTime = currentMillis; // Reset acceleration time when starting
      targetSpeed = speedWhenStopped;
    } else {
      Serial.println("Motor Stopped");
            accelerationStartTime = currentMillis; // Reset acceleration time when starting
            speedWhenStopped = currentSpeed;
            targetSpeed = 0;
 
    }
    */
  }
    if (motorRunning == true && motorShouldStop == true && currentSpeed == 0) {
    motorRunning = false;
  }

  // Handle acceleration timing, adjust speed gradually
  if (motorRunning) {
    unsigned long accelTimeElapsed = currentMillis - accelerationStartTime;

/*
    // Accelerate gradually over the set acceleration time
    if (accelTimeElapsed < ACCELERATION_TIME) {
      // Calculate how much speed we should have reached based on time elapsed
      currentSpeed = map(accelTimeElapsed, 0, ACCELERATION_TIME, MIN_SPEED, targetSpeed);
      currentSpeed = constrain(currentSpeed, MIN_SPEED, targetSpeed);
    } else {
      // If the acceleration time has passed, we should reach the target speed
      currentSpeed = targetSpeed;
    }
*/

/*
if (accelTimeElapsed < ACCELERATION_TIME) {
      // Calculate speed change (from current speed to target speed)
      float speedChange = (float)(targetSpeed - currentSpeed) * (float)accelTimeElapsed * ACCEL_RATE / ACCELERATION_TIME;

      // Apply the calculated speed change
      currentSpeed = currentSpeed + (int)speedChange;
    } else {
      // If the acceleration time has passed, we should reach the target speed
      currentSpeed = targetSpeed;
    }
    */


  
// Exponential acceleration: Start slow, speed up toward the end

  
   if (motorShouldStop == true && motorRunning) {
     logicalTargetSpeed = 0;
   } else {
     logicalTargetSpeed = targetSpeed;
   }
 
    if (accelTimeElapsed < ACCELERATION_TIME) {
      // Calculate acceleration factor based on time elapsed (quadratic curve)
      float progress = (float)accelTimeElapsed / ACCELERATION_TIME;  // Value from 0 to 1
      // speedFactor = progress; * progress;  // Linear
      // speedFactor = progress; * progress;  // Quadratic curve (slow at start, fast at end)
       speedFactor = (1 - cos(progress * PI)) / 2;  // Sinusoidal curve


           // float accelerationFactor = (1 - cos(progress * PI)) / 2;  // Sinusoidal curve


      // Calculate the current speed based on the target speed and acceleration factor
      if (logicalTargetSpeed == 0) {
              currentSpeed = targetSpeed - (targetSpeed * speedFactor); 

      } else {
              currentSpeed = logicalTargetSpeed * speedFactor; 

      }
      
    } else {
      // If acceleration time has passed, we should reach the target speed
      currentSpeed = logicalTargetSpeed;
      //lastTargetSpeed = targetSpeed;
    }
     
    /*
    // Sinusoidal acceleration: Slow at start and end
    if (accelTimeElapsed < ACCELERATION_TIME) {
      // Normalize time (0 to 1)
      float progress = (float)accelTimeElapsed / ACCELERATION_TIME;

      // Apply sinusoidal function: Slow at the start, fast in the middle, slow at the end
      float accelerationFactor = (1 - cos(progress * PI)) / 2;  // Sinusoidal curve

      // Calculate the current speed based on the target speed and acceleration factor
      currentSpeed = targetSpeed * accelerationFactor; // VIL ALLTID STARTE PÅ 0, IKKE PÅ TIDLIGERE FART !!!!
      afac = accelerationFactor;
    } else {
      // If acceleration time has passed, we should reach the target speed
      currentSpeed = targetSpeed;
    }
    */
        if (motorShouldStop == true && currentSpeed ==0) {
          motorRunning=false;
        }

    
    // Adjust the step interval based on current speed
    stepInterval = 1000000 / currentSpeed; // Convert speed (steps per second) to interval (microseconds)
  }

  // Print the current speed and interval every second (limit serial prints)
  if (currentMillis - previousMillis >= 50) {
    previousMillis = currentMillis;
    if (motorRunning) {
      Serial.print("(on)Speed: ");
      Serial.print(currentSpeed);
       Serial.print(" target: ");
       
      Serial.print(targetSpeed);
          Serial.print(" logicalTarget: ");
      Serial.print(logicalTargetSpeed);
         Serial.print(" whenStopped: ");
      Serial.print(speedWhenStopped);
        Serial.print(" speedFactor: ");
       Serial.print(speedFactor,3);
      Serial.print(" steps/sec, Interval: ");
      Serial.println(stepInterval);

    } else  {
     // Serial.println("Motor is turned off");
         Serial.print("(off)Speed: ");
      Serial.print(currentSpeed);
        Serial.print(" target: ");
      Serial.print(targetSpeed);
              Serial.print(" logicalTarget: ");
      Serial.print(logicalTargetSpeed);
               Serial.print(" whenStopped: ");
               
      Serial.print(speedWhenStopped);
             Serial.print(" speedFactor: ");
       Serial.print(speedFactor,3);
      Serial.print(" steps/sec, Interval: ");
      Serial.println(stepInterval);

    }
  }
}




void loop2() {
  float afac = 0;
  unsigned long currentMillis = millis();

  // Update button states and handle debouncing
  increaseBtnDebouncer.update();
  decreaseBtnDebouncer.update();
  dirBtnDebouncer.update();
  startStopBtnDebouncer.update();

  // Read buttons with debounced states
  if (increaseBtnDebouncer.fell()) {
    // Increase speed
    if (targetSpeed < MAX_SPEED) {
      lastTargetSpeed = targetSpeed;
      targetSpeed += speedChangePrClick;  // Increase by 10 steps per second
      Serial.print("Increased Speed: ");
      Serial.println(targetSpeed);
    }
    accelerationStartTime = currentMillis; // Reset acceleration when speed changes
  }

  if (decreaseBtnDebouncer.fell()) {
    // Decrease speed
    if (targetSpeed > MIN_SPEED) {
            lastTargetSpeed = targetSpeed;
      targetSpeed -= speedChangePrClick;  // Decrease by 10 steps per second
      Serial.print("Decreased Speed: ");
      Serial.println(targetSpeed);
    }
    accelerationStartTime = currentMillis; // Reset acceleration when speed changes
  }

  if (dirBtnDebouncer.fell()) {
    // Toggle direction
    direction = !direction;
    digitalWrite(DIR_PIN, direction); // Change direction
    if (direction) {
      Serial.println("Direction: Clockwise");
    } else {
      Serial.println("Direction: Counter-clockwise");
    }
  }

  if (startStopBtnDebouncer.fell()) {
    // Toggle motor running state
    if (motorRunning) {
            accelerationStartTime = currentMillis; // Reset acceleration time when starting
      motorShouldStop = true;
      //targetSpeed = 0;
    } else if (motorRunning == false) {
            accelerationStartTime = currentMillis; // Reset acceleration time when starting
      motorRunning = true;
      motorShouldStop = false;
      //targetSpeed = speedWhenStopped;
    }
/*    motorRunning = !motorRunning;

    if (motorRunning) {
      Serial.println("Motor Started");
      accelerationStartTime = currentMillis; // Reset acceleration time when starting
      targetSpeed = speedWhenStopped;
    } else {
      Serial.println("Motor Stopped");
            accelerationStartTime = currentMillis; // Reset acceleration time when starting
            speedWhenStopped = currentSpeed;
            targetSpeed = 0;
 
    }
    */
  }

  // Handle acceleration timing, adjust speed gradually
  if (motorRunning) {
    unsigned long accelTimeElapsed = currentMillis - accelerationStartTime;

/*
    // Accelerate gradually over the set acceleration time
    if (accelTimeElapsed < ACCELERATION_TIME) {
      // Calculate how much speed we should have reached based on time elapsed
      currentSpeed = map(accelTimeElapsed, 0, ACCELERATION_TIME, MIN_SPEED, targetSpeed);
      currentSpeed = constrain(currentSpeed, MIN_SPEED, targetSpeed);
    } else {
      // If the acceleration time has passed, we should reach the target speed
      currentSpeed = targetSpeed;
    }
*/

/*
if (accelTimeElapsed < ACCELERATION_TIME) {
      // Calculate speed change (from current speed to target speed)
      float speedChange = (float)(targetSpeed - currentSpeed) * (float)accelTimeElapsed * ACCEL_RATE / ACCELERATION_TIME;

      // Apply the calculated speed change
      currentSpeed = currentSpeed + (int)speedChange;
    } else {
      // If the acceleration time has passed, we should reach the target speed
      currentSpeed = targetSpeed;
    }
    */


  
// Exponential acceleration: Start slow, speed up toward the end
int useTargetSpeed = targetSpeed;
if (motorShouldStop == true) {
  useTargetSpeed = 0;
}
    if (accelTimeElapsed < ACCELERATION_TIME) {
      // Calculate acceleration factor based on time elapsed (quadratic curve)
      float progress = (float)accelTimeElapsed / ACCELERATION_TIME;  // Value from 0 to 1
      float speedFactor = progress * progress;  // Quadratic curve (slow at start, fast at end)

      // Calculate the current speed based on the target speed and acceleration factor
      int diff = useTargetSpeed - lastTargetSpeed; // take into account last speed
      currentSpeed = lastTargetSpeed + (diff * speedFactor); 
    } else {
      // If acceleration time has passed, we should reach the target speed
      currentSpeed = useTargetSpeed;
      lastTargetSpeed = useTargetSpeed;
    }
     
    /*
    // Sinusoidal acceleration: Slow at start and end
    if (accelTimeElapsed < ACCELERATION_TIME) {
      // Normalize time (0 to 1)
      float progress = (float)accelTimeElapsed / ACCELERATION_TIME;

      // Apply sinusoidal function: Slow at the start, fast in the middle, slow at the end
      float accelerationFactor = (1 - cos(progress * PI)) / 2;  // Sinusoidal curve

      // Calculate the current speed based on the target speed and acceleration factor
      currentSpeed = targetSpeed * accelerationFactor; // VIL ALLTID STARTE PÅ 0, IKKE PÅ TIDLIGERE FART !!!!
      afac = accelerationFactor;
    } else {
      // If acceleration time has passed, we should reach the target speed
      currentSpeed = targetSpeed;
    }
    */
        if (motorShouldStop == true && currentSpeed ==0) {
          motorRunning=false;
        }

    
    // Adjust the step interval based on current speed
    stepInterval = 1000000 / currentSpeed; // Convert speed (steps per second) to interval (microseconds)
  }

  // Print the current speed and interval every second (limit serial prints)
  if (currentMillis - previousMillis >= 100) {
    previousMillis = currentMillis;
    if (motorRunning) {
      Serial.print("(ON)Speed: ");
     

    } else if (motorShouldStop)  {
     // Serial.println("Motor is turned off");
         Serial.print("(off...)Speed: ");
    } else {
               Serial.print("(OFF)Speed: ");

    }
          Serial.print(currentSpeed);

      Serial.print(" target: ");
      Serial.print(targetSpeed);
         Serial.print(" whenStopped: ");
      Serial.print(speedWhenStopped);
      Serial.print(" , Interval: ");
      Serial.println(stepInterval);

  }
}

// Timer2 Interrupt Service Routine for Step Pulse Generation
ISR(TIMER2_COMPA_vect) {
  static unsigned long lastStepTime = 0;

  // Only generate step pulses if the motor is running
  if (/*motorRunning*/ stepInterval<10000000 && (micros() - lastStepTime >= stepInterval)) {
    // Set the step pin high and low to generate the step pulse
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);          // Short pulse duration
    digitalWrite(STEP_PIN, LOW);
    lastStepTime = micros();
  }
}

// Initialize Timer2 for step pulse generation
void initTimer2() {
  // Set Timer2 in CTC mode (Clear Timer on Compare Match)
  TCCR2A = 0;  // Normal mode (no output waveform)
  TCCR2B = (1 << WGM22) | (1 << CS21);  // CTC mode, prescaler 8
  OCR2A = 100; // Set compare match value (adjust this to control the frequency)
  TIMSK2 |= (1 << OCIE2A);  // Enable Timer2 compare match interrupt
}


