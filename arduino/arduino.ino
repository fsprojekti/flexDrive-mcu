#include <TFT_eSPI.h>  // Hardware-specific library
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

// Motor control variables
int pwmDrive = 0;
int dirDrive = 0;
volatile long encoderPosition = 0;  // Use volatile for variables accessed within ISR
volatile long secondaryEncoderPosition = 0; // For the secondary encoder

int encoderPinA = 32;
int encoderPinB = 33;
int secondaryEncoderPinA = 36;
int secondaryEncoderPinB = 37;

void setup() {
  Serial.begin(115200); // Initialize serial communication
  tft.init();           // Initialize the TFT display
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setRotation(1);   // Landscape mode
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // Set text color and background

  // Display Labels
  tft.drawString("PWM:", 10, 10);
  tft.drawString("DIR:", 10, 40);
  tft.drawString("Enc1:", 10, 70);
  tft.drawString("Enc2:", 10, 100); // Label for secondary encoder

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(encoderPinA, updateEncoder, CHANGE);
  attachInterrupt(encoderPinB, updateEncoder, CHANGE);

  pinMode(secondaryEncoderPinA, INPUT_PULLUP);
  pinMode(secondaryEncoderPinB, INPUT_PULLUP);
  attachInterrupt(secondaryEncoderPinA, updateSecondaryEncoder, CHANGE);
  attachInterrupt(secondaryEncoderPinB, updateSecondaryEncoder, CHANGE);
}

void loop() {
  updateDisplay("PWM:", pwmDrive, 70, 10);
  updateDisplay("DIR:", dirDrive, 70, 40);
  updateDisplay("Enc1:", encoderPosition, 70, 70);
  updateDisplay("Enc2:", secondaryEncoderPosition, 70, 100); // Update secondary encoder display

  delay(100); // Simple delay to manage update speed
}

// Function to update the display area for values
void updateDisplay(String label, long value, int x, int y) {
  String displayValue = (value >= 0 ? "+" : "") + String(value);
  tft.fillRect(x, y, 100, 16, TFT_BLACK); // Clear only the number area
  tft.drawString(displayValue, x, y); // Display new value
}

// Interrupt service routine to handle primary encoder
void updateEncoder() {
  static int lastEncoded = 0;
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPosition--;
  lastEncoded = encoded;
}

// Interrupt service routine to handle secondary encoder
void updateSecondaryEncoder() {
  static int lastEncoded = 0;
  int MSB = digitalRead(secondaryEncoderPinA);
  int LSB = digitalRead(secondaryEncoderPinB);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) secondaryEncoderPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) secondaryEncoderPosition--;
  lastEncoded = encoded;
}
