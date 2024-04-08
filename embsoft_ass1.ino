#include <TridentTD_EasyFreeRTOS32.h>

// Define the pin for the digital output
#define OUTPUT_PIN 1  
#define INPUT_PIN1 2  
#define INPUT_PIN2 3  
#define LED_PIN 5
#define BUTTON_PIN 6

#define NUM_READINGS 10

#define MEASURE_PERIOD1 20 // in milliseconds (20ms)
#define MEASURE_PERIOD2 8 // in milliseconds (20ms)

#define HIGH_DURATION 180 
#define LOW_DURATION 40   
#define HIGH_DURATION_2 530 
#define LOW_DURATION_2 3250 
#define PERIOD 4000     

SemaphoreHandle_t frequencySemaphore;
struct FrequencyData {
  int task2Frequency;
  int task3Frequency;
};

// Initializing frequency data structure
FrequencyData frequencies = {0, 0};

int readings[NUM_READINGS];  // Array to store the readings

// Task function for generating signal
void task1(void *parameter) {
  pinMode(OUTPUT_PIN, OUTPUT);

  for (;;) {
    unsigned long startTime = millis();
    digitalWrite(OUTPUT_PIN, HIGH);
    delayMicroseconds(HIGH_DURATION);
    digitalWrite(OUTPUT_PIN, LOW);
    delayMicroseconds(LOW_DURATION);
    digitalWrite(OUTPUT_PIN, HIGH);
    delayMicroseconds(HIGH_DURATION_2);
    digitalWrite(OUTPUT_PIN, LOW);
    delayMicroseconds(LOW_DURATION_2);
    unsigned long endTime = millis();
    vTaskDelay(4-(endTime-startTime)); 
  }
}

// Task function for measuring the frequency of the input signal which goes by a potentiometer
void task2(void *parameter) {

  pinMode(INPUT_PIN1, INPUT);
  for (;;) {
    // Variables to store the pulse count and frequency
    unsigned long startTime = millis();

    // Count the number of rising edges within the measurement period
    Serial.println(analogRead(INPUT_PIN1));
    while(analogRead(INPUT_PIN1)>0){}
    unsigned long previousRisingEdge = micros();
    while(analogRead(INPUT_PIN1)<0){}
    while(analogRead(INPUT_PIN1)>0){}
    unsigned long currentRisingEdge = micros();

    // Calculate frequency in Hz
    double frequency = 1000000/(currentRisingEdge - previousRisingEdge) ;

    // Check if frequency falls within the acceptable range (333Hz to 1000Hz)
    //if (frequency >= 333 && frequency <= 1000) {
      // Frequency within acceptable range
      Serial.print("Frequency: ");
      Serial.print(frequency);
      Serial.println(" Hz");
      Serial.println(previousRisingEdge);
      Serial.println(currentRisingEdge);
      if (xSemaphoreTake(frequencySemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        frequencies.task2Frequency = frequency;
        xSemaphoreGive(frequencySemaphore);
    }
    //} else {
      // Frequency outside acceptable range
      //Serial.println("Frequency out of acceptable range");
    //}
    unsigned long endTime = millis();
    // Delay until the next measurement period*/
    vTaskDelay(MEASURE_PERIOD1-(endTime-startTime));
  }
}

// Task function for measuring the frequency of a second input signal
void task3(void *parameter) {

  pinMode(INPUT_PIN2, INPUT);
  for (;;) {
    unsigned long startTime = millis();

    Serial.println(analogRead(INPUT_PIN2));
    while(analogRead(INPUT_PIN2)>0){}
    unsigned long previousRisingEdge = micros();
    while(analogRead(INPUT_PIN2)<0){}
    while(analogRead(INPUT_PIN2)>0){}
    unsigned long currentRisingEdge = micros();

    // Calculate frequency in Hz
    double frequency = 1000000/(currentRisingEdge - previousRisingEdge) ;

    // Check if frequency falls within the acceptable range (333Hz to 1000Hz)
    //if (frequency >= 500 && frequency <= 1000) {
      // Frequency within acceptable range
      Serial.print("Frequency: ");
      Serial.print(frequency);
      Serial.println(" Hz");
      Serial.println(previousRisingEdge);
      Serial.println(currentRisingEdge);
    //} else {
      // Frequency outside acceptable range
      //Serial.println("Frequency out of acceptable range");
    //}
    unsigned long endTime = millis();
    // Delay until the next measurement period*/
    vTaskDelay(MEASURE_PERIOD2-(endTime-startTime));
  }
}

void task4(void *parameter) {
  pinMode(LED_PIN, OUTPUT);  // Set LED pin as output
  pinMode(INPUT_PIN1, INPUT); // Set analog pin as input
  
  int index = 0;               // Index to keep track of the current reading
  int total = 0;               // Variable to store the total sum of readings

  for (;;) {
    unsigned long startTime = millis();
    // Read the analog input
    int sensorValue = analogRead(INPUT_PIN1);

    // Subtract the oldest reading from the total
    total -= readings[index];
  
    // Add the new reading to the total
    total += sensorValue;

    // Store the new reading in the readings array
    readings[index] = sensorValue;

    // Increment the index (and wrap around if necessary)
    index = (index + 1) % NUM_READINGS;

    // Calculate the running average
    int average = total / NUM_READINGS;

    // Visualize error using LED if average value exceeds half of the maximum range (512 for 10-bit ADC)
    if (average > 512) {
      digitalWrite(LED_PIN, HIGH);  // Turn on LED
    } else {
      digitalWrite(LED_PIN, LOW);   // Turn off LED
    }    
    // Delay for 20ms (50Hz)
    unsigned long endTime = millis();
    vTaskDelay(20-(endTime-startTime));
  }
}



void task5(void *parameter) {
  for (;;) {
    unsigned long startTime = millis();
    // Wait for semaphore to access frequency data
    if (xSemaphoreTake(frequencySemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      // Scale and bound the frequency values between 0 and 99
      int scaledTask2Frequency = map(frequencies.task2Frequency, 333, 1000, 0, 99);
      int scaledTask3Frequency = map(frequencies.task3Frequency, 500, 1000, 0, 99);

      // Log frequency values in comma-delimited format
      Serial.print(scaledTask2Frequency);
      Serial.print(",");
      Serial.println(scaledTask3Frequency);

      // Release semaphore after accessing frequency data
      xSemaphoreGive(frequencySemaphore);
    }

    // Delay for 200ms (5 Hz)
    unsigned long endTime = millis();
    vTaskDelay(200-(endTime-startTime));
  }
}

int ledState = HIGH;        // the current state of the output pin
int buttonState;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


void task7(void *parameter) {

  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  for(;;){
    
      // read the state of the switch into a local variable:
    digitalWrite(LED_PIN, LOW);
    int reading = digitalRead(BUTTON_PIN);

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != buttonState) {
        buttonState = reading;

        // only toggle the LED if the new button state is HIGH
        if (buttonState == HIGH) {
          ledState = !ledState;
        }
      }
    }

    if (buttonState == LOW) {
        // Toggle LED state
        ledState = !ledState;
    }
    // save the reading
    lastButtonState = reading;
  }  
}

// Function to cause CPU work for the specified time in milliseconds
void CPU_work(int time) {
  unsigned long startTime = millis();
  unsigned long endTime = startTime + time;
  
  // Perform CPU work with a for loop
  for (;;) {
    // Check if the specified time has elapsed
    if (millis() >= endTime) {
      break; // Exit the loop if time has elapsed
    }
  }
}

// Task to call CPU_work(2) every 20ms
void task8(void *parameter) {
  for (;;) {
    unsigned long startTime = millis();
    // Call CPU_work function to utilize the CPU for about 2ms
    CPU_work(2);
    
    // Delay for 20ms
    unsigned long endTime = millis();
    vTaskDelay(20-(endTime-startTime));
  }
}

void setup() {
  Serial.begin(9600); // Initialize serial communication

  xTaskCreate(task1, "Task1 Output Signal", 1024, NULL, 1, NULL);
  xTaskCreate(task2, "Task2 Receive Signal 1 with potentiometer", 1024, NULL, 1, NULL);
  xTaskCreate(task3, "Receive Signal 2", 1024, NULL, 1, NULL);
  xTaskCreate(task4, "Task4 Voltage Average with potentiometer", 1024, NULL, 2, NULL);
  xTaskCreate(task5, "Task5 Frequencies Storage", 1024, NULL, 2, NULL);
  xTaskCreate(task7, "Task5 Pushbutton", 1024, NULL, 3, NULL);
  xTaskCreate(task8, "Task5 CPU", 1024, NULL, 3, NULL);
}

void loop() {
}
