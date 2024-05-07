#include <Arduino.h>
#include <TFT_eSPI.h>  // Hardware-specific library for TFT
#include <SPI.h>       // SPI library for TFT
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

TFT_eSPI tft = TFT_eSPI();  // Create an instance of the TFT_eSPI library

int pwmDrive = 0;
int dirDrive = 0;
int pwmChannel = 0;
int pwmFrequency = 100;
int pwmResolution = 12;
int pwmPin = 26;
int dirPin = 25;

// Pin definitions for encoder inputs
const int encoder1PinA = 32;
const int encoder1PinB = 33;
const int encoder2PinA = 36;
const int encoder2PinB = 37;

// Volatile variables for encoder position
volatile long encoder1Position = 0;
volatile long encoder2Position = 0;

// FreeRTOS Task Handle
TaskHandle_t DisplayTaskHandle = NULL;

// Task handle for the encoder task
TaskHandle_t encoderTaskHandle = NULL;
TaskHandle_t encoder2TaskHandle = NULL;

typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

// ISR for encoder
void IRAM_ATTR encoder1ISR() {
  static int lastState = LOW;
  int currentState = digitalRead(encoder1PinA);
  if (currentState != lastState) {
    // Change detected, update position
    if (digitalRead(encoder1PinB) != currentState) {
      encoder1Position++;
    } else {
      encoder1Position--;
    }
    // Notify the encoder task from ISR
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(encoderTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
  lastState = currentState;
}
// ISR for encoder 2
void IRAM_ATTR encoder2ISR() {
  static int lastState = LOW;
  int currentState = digitalRead(encoder2PinA);
  if (currentState != lastState) {
    if (digitalRead(encoder2PinB) != currentState) {
      encoder2Position++;
    } else {
      encoder2Position--;
    }
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(encoder2TaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
  lastState = currentState;
}

// Task to process encoder data
void encoder1Task(void *pvParameters) {
  while (1) {
    // Wait indefinitely for notification from ISR
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Safely copy volatile variable
    long position = encoder1Position;
  }
}

void encoder2Task(void *pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    long position = encoder2Position;  // Safely copy volatile variable
  }
}

// Display Update Task
void displayTask(void *pvParameters) {
  while (1) {
    tft.fillScreen(TFT_BLACK);
    tft.drawString("PWM: " + String(pwmDrive), 10, 10);
    tft.drawString("DIR: " + String(dirDrive), 10, 40);
    tft.drawString("Enc1: " + String(encoder1Position), 10, 70);
    tft.drawString("Enc2: " + String(encoder2Position), 10, 100);
    vTaskDelay(pdMS_TO_TICKS(100));  // Update the display every 100 ms
  }
}

// Serial Receive Task
void serialReceiveTask(void *pvParameters) {
  char incomingChar;
  String receivedData = "";

  while (1) {
    if (Serial.available() > 0) {
      // Read the incoming data
      incomingChar = Serial.read();
      // Check for end of line or new message character
      if (incomingChar == '\n' || incomingChar == '\r') {
        if (receivedData.length() > 0) {
          // Process the complete message
          int idx = receivedData.indexOf(' ');
          if (idx != -1) {
            pwmDrive = receivedData.substring(0, idx).toInt();
            dirDrive = receivedData.substring(idx + 1).toInt();
            // Update the PWM and direction
            ledcWrite(pwmChannel, pwmDrive);                   // Set PWM duty cycle
            digitalWrite(dirPin, dirDrive == 0 ? LOW : HIGH);  // Set motor direction
          }
          receivedData = "";  // Clear the string for the next message
        }
      } else {
        receivedData += incomingChar;  // Add character to the current message
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Slight delay to allow other tasks processing time
  }
}



void sendEncoderData(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 10 ms delay
  xLastWakeTime = xTaskGetTickCount();               // Initialize the xLastWakeTime variable with the current time.

  while (1) {
    // Wait for the next cycle.
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    //Encoder 1
    FLOATUNION_t encoder1Float;
    encoder1Float.number = (float)encoder1Position;

    //Encoder 2
    FLOATUNION_t encoder2Float;
    encoder2Float.number = (float)encoder2Position;

    Serial.write('A');

    // Print float data
    for (int i = 0; i < 4; i++) {
      Serial.write(encoder1Float.bytes[i]);
    }

     // Print float data
    for (int i = 0; i < 4; i++) {
      Serial.write(encoder2Float.bytes[i]);
    }

    // Print terminator
    Serial.print('\n');

    // Serial.print(encoder1Position);
    // Serial.print(";");
    // Serial.println(encoder2Position);
  }
}


void setup() {
  Serial.begin(115200);
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setRotation(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Initialize pins and interrupt
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, CHANGE);

  // Initialize pins and interrupt for encoder 2
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2ISR, CHANGE);

  // Initialize PWM
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
  pinMode(dirPin, OUTPUT);

  // Create the encoder task with the highest priority
  xTaskCreate(encoder1Task, "Handle Encoder", 2048, NULL, configMAX_PRIORITIES - 1, &encoderTaskHandle);
  // Create the encoder 2 task
  xTaskCreate(encoder2Task, "Handle Encoder 2", 2048, NULL, configMAX_PRIORITIES - 1, &encoder2TaskHandle);
  // Create the display task
  xTaskCreate(displayTask, "Display Task", 2048, NULL, 2, &DisplayTaskHandle);
  // Create the task to send encoder data
  xTaskCreate(sendEncoderData, "Send Data", 2048, NULL, 1, NULL);
  // Create the task to receive serial data
  xTaskCreate(serialReceiveTask, "Serial Receive", 2048, NULL, 1, NULL);  // Same priority as send data task
}

void loop() {
  // Empty loop - the scheduler is running the tasks
}
