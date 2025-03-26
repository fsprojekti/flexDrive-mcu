#include <Arduino.h>
#include <TFT_eSPI.h>    // Hardware-specific library for TFT
#include <SPI.h>
#include <SimpleCLI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Initialize TFT
TFT_eSPI tft = TFT_eSPI();  

// PWM and Direction Pins
const int pwmFrequency = 100;
const int pwmResolution = 12;
const int pwmPin = 26;
const int dirPin = 25;

// Encoder Pins
const int encoder1PinA = 32;
const int encoder1PinB = 33;
const int encoder2PinA = 36;
const int encoder2PinB = 37;

// Encoder Positions (volatile as modified in ISR)
volatile long encoder1Position = 0;
volatile long encoder2Position = 0;

// Encoder Velocities
float encoder1Velocity = 0.0;
float encoder2Velocity = 0.0;

// Sampling time in ms
int ts = 10;

// Filter Constant for Velocity Calculation
const float alpha = 0.25;  // Higher value = less filtering, Lower value = more filtering

// Firmware version
const char* firmwareVersion = "1.0.0";

// SimpleCLI Objects
SimpleCLI cli;
Command cmdInit;
Command cmdRun;
Command cmdStop;
Command cmdMode;
Command cmdPwm;
Command cmdDir;
Command cmdList;
Command cmdRef;
Command cmdKp;
Command cmdKi;
Command cmdKd;
Command cmdPwmMax;
Command cmdPlot;
Command cmdTs;       // New command: set sampling time
Command cmdVersion;  // New command: fetch firmware version

// Control Variables
int mode = 0;        // 0 - Open loop, 1 - Closed loop
int state = 0;       // 0 - Stop, 1 - Run
int pwmESC = 0;      // PWM value for ESC
int dir = 1;         // Direction: 1 - Forward, 0 - Reverse
float ref = 0.3;     // Reference velocity [turn/s]
float Kp = 0.1;      // Proportional gain
float Ki = 0.0;      // Integral gain
float Kd = 0.0;      // Derivative gain
int pwmMax = 4000;   // Maximum PWM value
bool plot = false;   // Plotting flag

// PID Control Variables
float error = 0.0;
float integral = 0.0;
float previousError = 0.0;

// ISR for encoder 1
void IRAM_ATTR encoder1ISR() {
  static int lastEncoded = 0;
  int MSB = digitalRead(encoder1PinA);  // Most significant bit
  int LSB = digitalRead(encoder1PinB);  // Least significant bit

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoder1Position++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoder1Position--;
  }

  lastEncoded = encoded;
}

// ISR for encoder 2
void IRAM_ATTR encoder2ISR() {
  static int lastEncoded = 0;
  int MSB = digitalRead(encoder2PinA);  // Most significant bit
  int LSB = digitalRead(encoder2PinB);  // Least significant bit

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoder2Position++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoder2Position--;
  }

  lastEncoded = encoded;
}

// CLI Callback Functions
void errorCallback(cmd_error* e) {
  Serial.println("Error: Command not found or invalid parameters.");
}

void cmdInitCallback(cmd* c) {
  Serial.println("System Initialized.");
  // Additional initialization code if needed
}

void cmdRunCallback(cmd* c) {
  state = 1;
  Serial.println("System Started.");
}

void cmdStopCallback(cmd* c) {
  state = 0;
  Serial.println("System Stopped.");
}

void cmdListCallback(cmd* c) {
  Serial.println("----- System Status -----");
  Serial.print("Mode: ");
  Serial.println(mode == 0 ? "Open Loop" : "Closed Loop");
  Serial.print("State: ");
  Serial.println(state == 0 ? "Stop" : "Run");
  Serial.print("PWM: ");
  Serial.println(pwmESC);
  Serial.print("Dir: ");
  Serial.println(dir);
  Serial.print("Reference: ");
  Serial.println(ref);
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Ki: ");
  Serial.println(Ki);
  Serial.print("Kd: ");
  Serial.println(Kd);
  Serial.print("PWM Max: ");
  Serial.println(pwmMax);
  Serial.print("Plot: ");
  Serial.println(plot ? "Enabled" : "Disabled");
  Serial.print("Sampling Time (ts): ");
  Serial.println(ts);
  Serial.print("Firmware Version: ");
  Serial.println(firmwareVersion);
  Serial.print("Encoder1 Position: ");
  Serial.println(encoder1Position);
  Serial.print("Encoder1 Velocity: ");
  Serial.println(encoder1Velocity);
  Serial.print("Encoder2 Position: ");
  Serial.println(encoder2Position);
  Serial.print("Encoder2 Velocity: ");
  Serial.println(encoder2Velocity);
  Serial.println("-------------------------");
}

void cmdModeCallback(cmd* c) {
  Command cmd(c);
  int argNum = cmd.countArgs();
  if (argNum > 0) {
    int value = cmd.getArgument(0).getValue().toInt();
    mode = (value > 0) ? 1 : 0;
    Serial.print("Mode set to: ");
    Serial.println(mode == 0 ? "Open Loop" : "Closed Loop");
  } else {
    Serial.println("Error: 'mode' command requires a parameter (0 or 1).");
  }
}

void cmdPwmCallback(cmd* c) {
  Command cmd(c);
  int argNum = cmd.countArgs();
  if (argNum > 0) {
    int value = cmd.getArgument(0).getValue().toInt();
    value = constrain(value, 0, pwmMax);
    pwmESC = value;
    ledcWrite(pwmPin, pwmESC);
    Serial.print("PWM set to: ");
    Serial.println(pwmESC);
  } else {
    Serial.println("Error: 'pwm' command requires a parameter.");
  }
}

void cmdDirCallback(cmd* c) {
  Command cmd(c);
  int argNum = cmd.countArgs();
  if (argNum > 0) {
    int value = cmd.getArgument(0).getValue().toInt();
    dir = (value > 0) ? 1 : 0;
    digitalWrite(dirPin, dir > 0 ? HIGH : LOW);
    Serial.print("Direction set to: ");
    Serial.println(dir == 1 ? "Forward" : "Reverse");
  } else {
    Serial.println("Error: 'dir' command requires a parameter (0 or 1).");
  }
}

void cmdPwmMaxCallback(cmd* c) {
  Command cmd(c);
  int argNum = cmd.countArgs();
  if (argNum > 0) {
    int value = cmd.getArgument(0).getValue().toInt();
    if (value < 100) {
      Serial.println("Error: 'pwm_max' must be at least 100.");
      return;
    }
    pwmMax = value;
    Serial.print("PWM Max set to: ");
    Serial.println(pwmMax);
  } else {
    Serial.println("Error: 'pwm_max' command requires a parameter.");
  }
}

void cmdRefCallback(cmd* c) {
  Command cmd(c);
  int argNum = cmd.countArgs();
  if (argNum > 0) {
    float value = cmd.getArgument(0).getValue().toFloat();
    value = constrain(value, -10.0, 10.0);
    ref = value;
    Serial.print("Reference Velocity set to: ");
    Serial.println(ref);
  } else {
    Serial.println("Error: 'ref' command requires a parameter.");
  }
}

void cmdKpCallback(cmd* c) {
  Command cmd(c);
  int argNum = cmd.countArgs();
  if (argNum > 0) {
    float value = cmd.getArgument(0).getValue().toFloat();
    value = constrain(value, 0.0, 100.0);
    Kp = value;
    Serial.print("Kp set to: ");
    Serial.println(Kp);
  } else {
    Serial.println("Error: 'kp' command requires a parameter.");
  }
}

void cmdKiCallback(cmd* c) {
  Command cmd(c);
  int argNum = cmd.countArgs();
  if (argNum > 0) {
    float value = cmd.getArgument(0).getValue().toFloat();
    value = constrain(value, 0.0, 100.0);
    Ki = value;
    Serial.print("Ki set to: ");
    Serial.println(Ki);
  } else {
    Serial.println("Error: 'ki' command requires a parameter.");
  }
}

void cmdKdCallback(cmd* c) {
  Command cmd(c);
  int argNum = cmd.countArgs();
  if (argNum > 0) {
    float value = cmd.getArgument(0).getValue().toFloat();
    value = constrain(value, 0.0, 100.0);
    Kd = value;
    Serial.print("Kd set to: ");
    Serial.println(Kd);
  } else {
    Serial.println("Error: 'kd' command requires a parameter.");
  }
}

void cmdPlotCallback(cmd* c) {
  Command cmd(c);
  int argNum = cmd.countArgs();
  if (argNum > 0) {
    int value = cmd.getArgument(0).getValue().toInt();
    plot = (value > 0) ? true : false;
    Serial.print("Plotting ");
    Serial.println(plot ? "Enabled." : "Disabled.");
  } else {
    Serial.println("Error: 'plot' command requires a parameter (0 or 1).");
  }
}

// New command callback to set sampling time (ts)
void cmdTsCallback(cmd* c) {
  Command cmd(c);
  int argNum = cmd.countArgs();
  if (argNum > 0) {
    int value = cmd.getArgument(0).getValue().toInt();
    if (value < 1) {
      Serial.println("Error: ts must be at least 1 ms.");
      return;
    }
    ts = value;
    Serial.print("Sampling time (ts) set to: ");
    Serial.println(ts);
  } else {
    Serial.println("Error: 'ts' command requires a parameter.");
  }
}

// New command callback to fetch firmware version
void cmdVersionCallback(cmd* c) {
  Serial.print("Firmware Version: ");
  Serial.println(firmwareVersion);
}

// FreeRTOS Tasks

// Task to handle CLI input
void cliTask(void *pvParameters) {
  while (1) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      cli.parse(input);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void displayTask(void *pvParameters) {
  char buffer[30];
  int lineHeight = 15;
  int leftColumnX = 0;
  int rightColumnX = 50;
  int parametersColumnX = 130;
  int valuesColumnX = 180;

  tft.setTextFont(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  while (1) {
    int yPosition = 0;
    tft.fillScreen(TFT_BLACK); // Clear screen before update

    // Row 1: Encoder 1 & Velocity 1
    tft.drawString("Enc1:", leftColumnX, yPosition);
    sprintf(buffer, "%ld", encoder1Position);
    tft.drawString(buffer, rightColumnX, yPosition);
    tft.drawString("Vel1:", parametersColumnX, yPosition);
    sprintf(buffer, "%.2f", encoder1Velocity);
    tft.drawString(buffer, valuesColumnX, yPosition);
    yPosition += lineHeight;

    // Row 2: Encoder 2 & Velocity 2
    tft.drawString("Enc2:", leftColumnX, yPosition);
    sprintf(buffer, "%ld", encoder2Position);
    tft.drawString(buffer, rightColumnX, yPosition);
    tft.drawString("Vel2:", parametersColumnX, yPosition);
    sprintf(buffer, "%.2f", encoder2Velocity);
    tft.drawString(buffer, valuesColumnX, yPosition);
    yPosition += lineHeight;

    // Blank row
    yPosition += lineHeight;

    // Row 4: Mode & Reference
    tft.drawString("Mode:", leftColumnX, yPosition);
    tft.drawString(mode == 0 ? "Open" : "Close", rightColumnX, yPosition);
    tft.drawString("Ref:", parametersColumnX, yPosition);
    sprintf(buffer, "%.2f", ref);
    tft.drawString(buffer, valuesColumnX, yPosition);
    yPosition += lineHeight;

    // Row 5: State & Kp
    tft.drawString("State:", leftColumnX, yPosition);
    tft.drawString(state == 0 ? "Stop" : "Run", rightColumnX, yPosition);
    tft.drawString("Kp:", parametersColumnX, yPosition);
    sprintf(buffer, "%.2f", Kp);
    tft.drawString(buffer, valuesColumnX, yPosition);
    yPosition += lineHeight;

    // Row 6: PWM & Ki
    tft.drawString("PWM:", leftColumnX, yPosition);
    sprintf(buffer, "%d", pwmESC);
    tft.drawString(buffer, rightColumnX, yPosition);
    tft.drawString("Ki:", parametersColumnX, yPosition);
    sprintf(buffer, "%.2f", Ki);
    tft.drawString(buffer, valuesColumnX, yPosition);
    yPosition += lineHeight;

    // Row 7: Direction & Kd
    tft.drawString("Dir:", leftColumnX, yPosition);
    tft.drawString(dir == 0 ? "Fwd" : "Rvr", rightColumnX, yPosition);
    tft.drawString("Kd:", parametersColumnX, yPosition);
    sprintf(buffer, "%.2f", Kd);
    tft.drawString(buffer, valuesColumnX, yPosition);
    yPosition += lineHeight;

    // Row 8: Error contributions
    tft.drawString("eP: ", leftColumnX, yPosition);
    sprintf(buffer, "%.2f", Kp * error);
    tft.drawString(buffer, rightColumnX, yPosition);
    tft.drawString("eI:", parametersColumnX, yPosition);
    sprintf(buffer, "%.2f", Ki * integral);
    tft.drawString(buffer, valuesColumnX, yPosition);
    yPosition += lineHeight;

    // // Row 9: Sampling Time and Firmware Version
    // tft.drawString("ts:", leftColumnX, yPosition);
    // sprintf(buffer, "%d", ts);
    // tft.drawString(buffer, rightColumnX, yPosition);
    // tft.drawString("FW:", parametersColumnX, yPosition);
    // tft.drawString(firmwareVersion, valuesColumnX, yPosition);
    // yPosition += lineHeight;

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task to send encoder data over Serial
void sendEncoderData(void *pvParameters) {
  while (1) {
    if (plot) {
      Serial.print(encoder1Position);
      Serial.print(" ");
      Serial.print(encoder2Position);
      Serial.print(" ");
      Serial.print(encoder1Velocity);
      Serial.print(" ");
      Serial.println(encoder2Velocity);
    }
    vTaskDelay(pdMS_TO_TICKS(ts));
  }
}

// Task to calculate velocities and handle control logic every ts ms
void controlAndVelocityTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(ts);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  long lastEncoder1Position = 0;
  long lastEncoder2Position = 0;

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Calculate Velocity1
    long currentEnc1 = encoder1Position;
    float deltaEnc1 = currentEnc1 - lastEncoder1Position;
    float rawVelocity1 = deltaEnc1 / 4096.0 * 10.0;
    encoder1Velocity = alpha * rawVelocity1 + (1 - alpha) * encoder1Velocity;
    lastEncoder1Position = currentEnc1;

    // Calculate Velocity2
    long currentEnc2 = encoder2Position;
    float deltaEnc2 = currentEnc2 - lastEncoder2Position;
    float rawVelocity2 = deltaEnc2 / 48.0 / 20.4 * 10.0;
    encoder2Velocity = alpha * rawVelocity2 + (1 - alpha) * encoder2Velocity;
    lastEncoder2Position = currentEnc2;

    // Control Logic
    if (mode == 1 && state == 1) { // Closed-loop and Running
      error = ref - encoder1Velocity;
      integral += error * (ts / 1000.0);
      float derivative = (error - previousError) / (ts / 1000.0);
      float output = Kp * error + Ki * integral + Kd * derivative;
      output = constrain(output, -100.0, 100.0);
      pwmESC = int(output * 40.0);
      ledcWrite(pwmPin, pwmESC);
      digitalWrite(dirPin, output < 0.0 ? HIGH : LOW);
      if (plot) {
        Serial.print(pwmESC);
        Serial.print(" ");
        Serial.print(ref);
        Serial.print(" ");
        Serial.println(encoder1Velocity);
      }
      previousError = error;
    } else if (mode == 0 && state == 1) { // Open-loop and Running
      ledcWrite(pwmPin, pwmESC);
      digitalWrite(dirPin, dir);
      if (plot) {
        Serial.print(pwmESC);
        Serial.print(" ");
        Serial.print(encoder1Velocity);
        Serial.println();
      }
    } else { // Stopped
      ledcWrite(pwmPin, 0);
      digitalWrite(dirPin, LOW);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize TFT
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // Initialize Encoder Pins with Pull-up resistors
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), encoder1ISR, CHANGE);

  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), encoder2ISR, CHANGE);

  // Initialize PWM
  ledcAttach(pwmPin, pwmFrequency, pwmResolution);
  pinMode(dirPin, OUTPUT);
  ledcWrite(pwmPin, 0);
  digitalWrite(dirPin, LOW);

  // Initialize SimpleCLI
  cli.setOnError(errorCallback);
  cmdInit = cli.addCmd("init", cmdInitCallback);
  cmdRun = cli.addCmd("run", cmdRunCallback);
  cmdStop = cli.addCmd("stop", cmdStopCallback);
  cmdList = cli.addCmd("list", cmdListCallback);
  cmdMode = cli.addSingleArgCmd("mode", cmdModeCallback);
  cmdPwm = cli.addSingleArgCmd("pwm", cmdPwmCallback);
  cmdDir = cli.addSingleArgCmd("dir", cmdDirCallback);
  cmdRef = cli.addSingleArgCmd("ref", cmdRefCallback);
  cmdKp = cli.addSingleArgCmd("kp", cmdKpCallback);
  cmdKi = cli.addSingleArgCmd("ki", cmdKiCallback);
  cmdKd = cli.addSingleArgCmd("kd", cmdKdCallback);
  cmdPwmMax = cli.addSingleArgCmd("pwm_max", cmdPwmMaxCallback);
  cmdPlot = cli.addSingleArgCmd("plot", cmdPlotCallback);
  
  // Register new commands
  cmdTs = cli.addSingleArgCmd("ts", cmdTsCallback);
  cmdVersion = cli.addCmd("version", cmdVersionCallback);

  // Create FreeRTOS Tasks
  xTaskCreate(displayTask, "Display Task", 4096, NULL, 2, NULL);
  xTaskCreate(cliTask, "CLI Task", 2048, NULL, 1, NULL);
  xTaskCreate(sendEncoderData, "Send Data", 2048, NULL, 1, NULL);
  xTaskCreate(controlAndVelocityTask, "Calculate Velocity and control task", 2048, NULL, 1, NULL);

  Serial.println("READY");
}

void loop() {
  // Empty. All functionality is handled in FreeRTOS tasks.
}
