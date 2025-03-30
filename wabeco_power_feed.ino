#include <Bounce2.h>  // Include the Bounce2 library
#include <LiquidCrystal.h>  // Include the LiquidCrystal library

#define STEP_PIN 3        // Pin for stepper motor step
#define DIR_PIN 2         // Pin for stepper motor direction
#define INCREASE_BTN_PIN 6 // Pin for the increase speed button
#define DIR_BTN_PIN 7     // Pin for direction toggle button
#define DECREASE_BTN_PIN 8 // Pin for the decrease speed button
#define START_STOP_BTN_PIN 9 // Pin for the start/stop button

#define ACCEL_RATE 1     // Rate of acceleration/deceleration (steps per tick???)
#define MAX_SPEED 3000     // Maximum speed (steps per second)
#define MIN_SPEED 0      // Minimum speed (steps per second)
#define ACCELERATION_TIME 1000 // Time for full acceleration (milliseconds)




// Create an LCD object. The LCD is connected to pins 8, 9, 4, 5, 6, 7 on the Arduino.
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// Define the analog pin where the keypad is connected
int analogPin = A0;  
int buttonValue = 0;   // Variable to store the value read from the keypad



int analogSpeedPin = A5;  
int analogSpeedDialValue = 0;   





// Stepper speed control variables
unsigned long stepInterval = 0;   // Initial step interval (microseconds)
int currentSpeed =  0;              // Current speed (steps per second)
int targetSpeed =  0;               // Target speed (steps per second)
 int logicalTargetSpeed = targetSpeed;
bool direction = true;               // Motor direction (true for clockwise, false for counterclockwise)
bool motorRunning = false;           // Flag to indicate if the motor is running
bool motorShouldStop = false;


int speedChangePrClick = 100;

int speedWhenStopped = 0;

int lastTargetSpeed = 0;  
bool initiated = false;
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




// for filter
const int maxValues = 100;  // The size of the array to hold the last 100 values
int values[maxValues];      // Array to store the last 100 values
int currentIndex = 0;       // Index for the current value
long sum = 0;               // Sum of all the values in the array
unsigned long lastFilteredMillis = 0;





void setup() {
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  //pinMode(A1, OUTPUT);
  //digitalWrite(A2, LOW);
  
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, direction);
  
  // Control buttons
//  pinMode(INCREASE_BTN_PIN, INPUT_PULLUP);
//  pinMode(DECREASE_BTN_PIN, INPUT_PULLUP);
//  pinMode(DIR_BTN_PIN, INPUT_PULLUP);
//  pinMode(START_STOP_BTN_PIN, INPUT_PULLUP); // Pin for start/stop button


  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A6, INPUT_PULLUP); //??
  pinMode(A6, INPUT_PULLUP); // ??




  
  

  // Initialize debouncers for buttons
 // increaseBtnDebouncer.attach(INCREASE_BTN_PIN);
 // increaseBtnDebouncer.interval(50);  // Set debounce interval to 50ms
    // Initialize debouncers for buttons
 
  //increaseBtnDebouncer.attach(A3);
  //increaseBtnDebouncer.interval(50);  // Set debounce interval to 50ms


  
  //decreaseBtnDebouncer.attach(DECREASE_BTN_PIN);
  //decreaseBtnDebouncer.interval(50);  // Set debounce interval to 50ms
   decreaseBtnDebouncer.attach(A2);
  decreaseBtnDebouncer.interval(50);  // Set debounce interval to 50ms

  
  //dirBtnDebouncer.attach(DIR_BTN_PIN);
  //dirBtnDebouncer.interval(50);  // Set debounce interval to 50ms
   dirBtnDebouncer.attach(A4);
  dirBtnDebouncer.interval(50);  // Set debounce interval to 50ms
  
  //startStopBtnDebouncer.attach(START_STOP_BTN_PIN);
  //startStopBtnDebouncer.interval(50);  // Set debounce interval to 50ms
  

  startStopBtnDebouncer.attach(A1);
  startStopBtnDebouncer.interval(50);  // Set debounce interval to 50ms






  lcd.begin(16, 2);      // Initialize the LCD with 16 columns and 2 rows
  lcd.print("Stepper!");  // Print a message to the LCD

  delay(1000);  // Wait for 2 seconds before clearing the display
  lcd.clear();  // Clear the LCD display

  //lcd.print("Press a button");





  

  // Initialize Timer2 for step pulse generation
  initTimer2();

  // Start serial for debugging
  Serial.begin(115200);
}



// four digit number
void printLcd4Digit(int col, int row, int value) {

  if (value<10) {
        lcd.setCursor(col, row);  // Set the cursor to the second row, first column

      lcd.println("   ");
            lcd.setCursor(col+3, row);  // Set the cursor to the second row, first column
    lcd.print(value);

    } else
    if (value<100) {
        lcd.setCursor(col, row);  // Set the cursor to the second row, first column

      lcd.println("  ");
            lcd.setCursor(col+2, row);  // Set the cursor to the second row, first column
    lcd.print(value);

    } else if (value<1000) {
        lcd.setCursor(col, row);  // Set the cursor to the second row, first column

      lcd.println(" ");
            lcd.setCursor(col+1, row);  // Set the cursor to the second row, first column
    lcd.print(value);

    }else {
            lcd.setCursor(col, row);  // Set the cursor to the second row, first column
    lcd.print(value);

    }


}







int previousLcdMillis = 0;
int aliveCounter = 0;
void handleLcdButtons(float currentMillis) {
 
 
  
  int m = millis();
  if (m - previousLcdMillis < 500) {
    return;
  }
  previousLcdMillis=m;
  

//  lcd.setCursor(0, 9);  // Set the cursor to the second row, first column
//  lcd.print(targetSpeed);
//  if (motorRunning) {
//    lcd.print("RUNNING");
//  } else {
//    lcd.print("STOPPED");
//  }

/*
  buttonValue = analogRead(analogPin);  // Read the value from the keypad (A0 pin)

    analogSpeedDialValue = analogRead(analogSpeedPin);   
    analogSpeedDialValue = analogSpeedDialValue * 5;

 /// do some filtering
 // Update the sum by subtracting the oldest value and adding the new one
  sum -= values[currentIndex];  // Remove the oldest value from the sum
  values[currentIndex] = analogSpeedDialValue;  // Store the new value
  sum += values[currentIndex];  // Add the new value to the sum
  
  // Move to the next index in the array, wrapping around when we reach the end
  currentIndex = (currentIndex + 1) % maxValues;
  
  // Calculate the mean
  long meanValue = sum / maxValues;
  */
  
  
   long meanValue = sum / maxValues; // from filtering analog input 
   targetSpeed = meanValue;


   // targetSpeed = analogSpeedDialValue * 5.0;

    
// override id rapid....
    int  rapid = digitalRead( A3);
    if(rapid==0) {
      targetSpeed = 3000;
    }
    

  // Display the button value on the LCD
  //lcd.setCursor(12, 0);  // Set the cursor to the second row, first column
  //  lcd.print(buttonValue);

  printLcd4Digit(12,0,analogSpeedDialValue);
  /*
  if (analogSpeedDialValue<10) {
        lcd.setCursor(12, 0);  // Set the cursor to the second row, first column

      lcd.println("   ");
            lcd.setCursor(15, 0);  // Set the cursor to the second row, first column
    lcd.print(analogSpeedDialValue);

    } else
    if (analogSpeedDialValue<100) {
        lcd.setCursor(12, 0);  // Set the cursor to the second row, first column

      lcd.println("  ");
            lcd.setCursor(14, 0);  // Set the cursor to the second row, first column
    lcd.print(analogSpeedDialValue);

    } else if (analogSpeedDialValue<1000) {
        lcd.setCursor(12, 0);  // Set the cursor to the second row, first column

      lcd.println(" ");
            lcd.setCursor(13, 0);  // Set the cursor to the second row, first column
    lcd.print(analogSpeedDialValue);

    }else {
            lcd.setCursor(12, 0);  // Set the cursor to the second row, first column
    lcd.print(analogSpeedDialValue);

    }
*/


  lcd.setCursor(7, 0);
  lcd.print(aliveCounter++ % 0xff);

  // Check the value of the button to see which one is pressed
  if (buttonValue < 50) {
         toggleStartStop(currentMillis);

    lcd.setCursor(0, 0);
    lcd.print("Right ");
    lcd.setCursor(10,1);  
  lcd.print(targetSpeed);
  
  }
  else if (buttonValue < 150) {
    lcd.setCursor(0, 0);
    lcd.print("Up  ");
    increaseSpeed();
    lcd.setCursor(10,1);   
  lcd.print(targetSpeed);
  }
//  else if (buttonValue < 300) {
//    lcd.setCursor(2, 1);
//    lcd.print("Left  ");
//  }
  else if (buttonValue < 450) {
    lcd.setCursor(0, 0);
    lcd.print("Down  ");
        decreaseSpeed();
        lcd.setCursor(10,1);   
  lcd.print(targetSpeed);

  }
  else if (buttonValue < 600) {
    lcd.setCursor(0, 0);
    lcd.print("left  ");
    toggleDirection();
    lcd.setCursor(10,1);  
  lcd.print(targetSpeed);  
  }
  else if (buttonValue < 750) {
    lcd.setCursor(0, 0);
    lcd.print("Select ");
  }

  else {
    lcd.setCursor(2, 1);
    lcd.print(" - ");
  }


   lcd.setCursor(9,1);   
if (direction) {
    lcd.print("FWD");

} else {
      lcd.print("REV");

}



  printLcd4Digit(12,1,targetSpeed);

    /*

   lcd.setCursor(12,1);  
  lcd.print(targetSpeed);
  if (targetSpeed<1000) {
       lcd.setCursor(15,1);  
         lcd.print(" ");
  } else if (targetSpeed<100) {
       lcd.setCursor(14,1);  
         lcd.print("  ");
  } else if (targetSpeed<10) {
       lcd.setCursor(13,1);  
         lcd.print("   ");
  }
  
  
   lcd.setCursor(12,1);   
  lcd.print(targetSpeed);
*/

  
     lcd.setCursor(0,1);  

if (motorRunning && motorShouldStop) {
      lcd.print("STOPPING");

}
else if (motorRunning) {
    lcd.print("RUNNING ");
  } else {
    lcd.print("STOPPED ");
  }
  

}




void increaseSpeed() {
    // Increase speed
    if (targetSpeed < MAX_SPEED) {
       targetSpeed += speedChangePrClick;  // Increase by 10 steps per second
      //Serial.print("Increased Speed: ");
      //Serial.println(targetSpeed);
    }
}

void decreaseSpeed() {
   // Decrease speed
    if (targetSpeed > MIN_SPEED) {
      
       targetSpeed -= speedChangePrClick;  // Decrease by 10 steps per second
      //Serial.print("Decreased Speed: ");
      //Serial.println(targetSpeed);
    }
}


void setDirection(bool up) {
       if (motorRunning and (targetSpeed > 500 or currentSpeed > 500) ) {
              Serial.println("ERROR: Motor must be stopped or run at low speed before changing direction");
              return;
      }
      if (up) {
               digitalWrite(DIR_PIN, 0); // Change direction

      } else {
                       digitalWrite(DIR_PIN, 1); // Change direction

      }
      direction = !up;
    if (direction) {
      Serial.println("Direction: Clockwise");
    } else {
      Serial.println("Direction: Counter-clockwise");
    }
}
void toggleDirection() {
      // Toggle direction
      if (motorRunning and (targetSpeed > 500 or currentSpeed > 500) ) {
              Serial.println("ERROR: Motor must be stopped or run at low speed before changing direction");
              return;
      }
    direction = !direction;
    digitalWrite(DIR_PIN, direction); // Change direction
    if (direction) {
      Serial.println("Direction: Clockwise");
    } else {
      Serial.println("Direction: Counter-clockwise");
    }
  
}

void start(float currentMillis, bool up) {
    Serial.print("toggle motor start, dir:");
    Serial.println(up);
    if (initiated == false) {
       // Clear potential error message
       lcd.setCursor(0, 0);
       lcd.print("                ");
              lcd.setCursor(0, 1);
       lcd.print("                ");
       initiated = true; // use this if there is stuff that shall not be done before after first start...
    }

     setDirection(up);
     
    // Toggle motor running state
    if (motorRunning == true) {
      Serial.println("already started");
    } else if (motorRunning == false) {
      Serial.println("try to start");
      accelerationStartTime = currentMillis; // Reset acceleration time when starting
      motorRunning = true;
      motorShouldStop = false;
      //targetSpeed = speedWhenStopped;
    }
}
void stop(float currentMillis) {
   if (motorRunning == true) {
      Serial.println("try to stop");
      accelerationStartTime = currentMillis; // Reset acceleration time when starting
      //motorRunning = false;
      motorShouldStop = true;
      
    } else if (motorRunning == false) {
      Serial.println("already stopped");
      
    }
}


void toggleStartStop(float currentMillis) {
    Serial.println("toggle motor start");
    initiated = true; // use this if there is stuff that shall not be done before after first start...

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
}

void handleSeparateButtons(float currentMillis) {
  // Update button states and handle debouncing
  increaseBtnDebouncer.update();
  decreaseBtnDebouncer.update();
  dirBtnDebouncer.update();
  startStopBtnDebouncer.update();

  // Read buttons with debounced states
  if (increaseBtnDebouncer.fell()) {
      increaseSpeed();
  }

  if (decreaseBtnDebouncer.fell()) {
     start(currentMillis, false);  

    //decreaseSpeed();
  }

  if (dirBtnDebouncer.fell()) {
     //toggleDirection();
  }

  if (startStopBtnDebouncer.fell()) {
    //toggleStartStop(currentMillis);
    start(currentMillis, true); //helge
  }
}





void loop() {



  
  unsigned long currentMillis = millis();


  


 
  
  handleSeparateButtons(currentMillis);
  handleLcdButtons(currentMillis);



if (millis() > lastFilteredMillis + 10) {
  lastFilteredMillis = millis();
 buttonValue = analogRead(analogPin);  // Read the value from the keypad (A0 pin)

    analogSpeedDialValue = analogRead(analogSpeedPin);   
    analogSpeedDialValue = analogSpeedDialValue * 5;

 /// do some filtering
 // Update the sum by subtracting the oldest value and adding the new one
  sum -= values[currentIndex];  // Remove the oldest value from the sum
  values[currentIndex] = analogSpeedDialValue;  // Store the new value
  sum += values[currentIndex];  // Add the new value to the sum
  
  // Move to the next index in the array, wrapping around when we reach the end
  currentIndex = (currentIndex + 1) % maxValues;
  
  // Calculate the mean
  //long meanValue = sum / maxValues;
}





  

  int  s = digitalRead( A1 );
    int  s2 = digitalRead( A2);
     // motor control must be idle when starting up !!!!!!!!
if (!initiated && (s == 0 || s2==0)) {
     Serial.println("ERROR! Set motor control to idle !!!!!!!");
            lcd.setCursor(0, 0);
            lcd.print("ERROR:");
            lcd.setCursor(0, 1);
            lcd.print("Disable motor !");
            return;
}

  if (s > 0 && s2 > 0) {

    if (motorRunning == true && motorShouldStop == false) {
      Serial.println("stop!!!!");
      stop(currentMillis);
    }
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

   int dynamicAccelTime =  ACCELERATION_TIME;
   if (targetSpeed > 3000 && targetSpeed > currentSpeed) {
    dynamicAccelTime * 4;
   }   
   else if (targetSpeed > 1500 && targetSpeed > currentSpeed) {
    dynamicAccelTime * 2;
   }
   
    if (accelTimeElapsed < dynamicAccelTime) {
      // Calculate acceleration factor based on time elapsed (quadratic curve)
      float progress = (float)accelTimeElapsed / dynamicAccelTime;  // Value from 0 to 1
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


// Timer2 Interrupt Service Routine for Step Pulse Generation
ISR(TIMER2_COMPA_vect) {
  static unsigned long lastStepTime = 0;

  // Only generate step pulses if the motor is running
  if ( motorRunning && initiated && stepInterval<10000000 && (micros() - lastStepTime >= stepInterval)) {
    // Set the step pin high and low to generate the step pulse
    digitalWrite(STEP_PIN, HIGH);
       // digitalWrite(A1, HIGH);

    delayMicroseconds(10);          // Short pulse duration
    digitalWrite(STEP_PIN, LOW);
       // digitalWrite(A1, LOW);

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


