#include <Arduino.h>
#include <TFT_eSPI.h>  // Hardware-specific library for TFT
#include <SPI.h>       // SPI library for TFT
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

TFT_eSPI tft = TFT_eSPI();  // Create an instance of the TFT_eSPI library

int pwmDrive = 0;
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
TaskHandle_t encoderTaskHandle = NULL;
TaskHandle_t encoder2TaskHandle = NULL;

// ISR for encoder 1
void IRAM_ATTR encoder1ISR() {
  static int lastState = LOW;
  int currentState = digitalRead(encoder1PinA);
  if (currentState != lastState) {
    if (digitalRead(encoder1PinB) != currentState) {
      encoder1Position++;
    } else {
      encoder1Position--;
    }
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
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    long position = encoder1Position;  // Safely copy volatile variable
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
    tft.drawString("Enc1: " + String(encoder1Position), 10, 40);
    tft.drawString("Enc2: " + String(encoder2Position), 10, 70);
    vTaskDelay(pdMS_TO_TICKS(100));  // Update the display every 100 ms
  }
}

// Serial Receive Task
void serialReceiveTask(void *pvParameters) {
  char incomingChar;
  String receivedData = "";

  while (1) {
    if (Serial.available() > 0) {
      incomingChar = Serial.read();
      if (incomingChar == '\n' || incomingChar == '\r') {
        if (receivedData.length() > 0) {
          pwmDrive = receivedData.toInt();
          pwmDrive = constrain(pwmDrive, -4000, 4000);  // Constrain the PWM value to ±4000
          int pwmValue = abs(pwmDrive);
          int dirValue = (pwmDrive >= 0) ? HIGH : LOW;
          ledcWrite(pwmChannel, pwmValue);
          digitalWrite(dirPin, dirValue);
          receivedData = "";
        }
      } else {
        receivedData += incomingChar;
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void sendEncoderData(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 100 ms delay
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    Serial.print(encoder1Position);
    Serial.print(" ");
    Serial.println(encoder2Position);
  }
}

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setRotation(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, CHANGE);

  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2ISR, CHANGE);

  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);
  pinMode(dirPin, OUTPUT);

  xTaskCreate(encoder1Task, "Handle Encoder", 2048, NULL, configMAX_PRIORITIES - 1, &encoderTaskHandle);
  xTaskCreate(encoder2Task, "Handle Encoder 2", 2048, NULL, configMAX_PRIORITIES - 1, &encoder2TaskHandle);
  xTaskCreate(displayTask, "Display Task", 2048, NULL, 2, &DisplayTaskHandle);
  xTaskCreate(sendEncoderData, "Send Data", 2048, NULL, 1, NULL);
  xTaskCreate(serialReceiveTask, "Serial Receive", 2048, NULL, 1, NULL);

  // Set PWM to 0 at startup
  ledcWrite(pwmChannel, 0);
  digitalWrite(dirPin, LOW);
}

void loop() {
  // Empty loop - the scheduler is running the tasks
}
