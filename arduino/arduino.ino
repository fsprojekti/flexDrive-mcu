#include <Arduino.h>
#include "soc/sens_reg.h"

// Base address for SENS peripheral
#define DR_REG_SENS_BASE 0x3FF48800

// ADC1 registers
#define SENS_SAR_MEAS1_CTRL1_REG    (DR_REG_SENS_BASE + 0x34) // ADC1 Control Register 1
#define SENS_SAR_MEAS1_CTRL2_REG    (DR_REG_SENS_BASE + 0x38) // ADC1 Control Register 2
#define SENS_SAR_MEAS1_MUX_REG      (DR_REG_SENS_BASE + 0x3C) // ADC1 MUX Register
#define SENS_SAR_START_FORCE_REG    (DR_REG_SENS_BASE + 0x24) // ADC Start Force Register
#define SENS_SAR1_STATUS_REG        (DR_REG_SENS_BASE + 0x48) // ADC1 Status Register
#define SENS_SAR_ATTEN1_REG         (DR_REG_SENS_BASE + 0x50) // ADC1 Attenuation Register

const uint8_t channel = 0; // ADC1_CHANNEL_0 (GPIO36)

void setup() {
    Serial.begin(115200);
    delay(100); // Ensure Serial Monitor connection
    Serial.println("Initializing ADC...");

    // Reset ADC registers
    REG_WRITE(SENS_SAR_MEAS1_CTRL1_REG, 0);
    REG_WRITE(SENS_SAR_MEAS1_CTRL2_REG, 0);
    REG_WRITE(SENS_SAR_MEAS1_MUX_REG, 0);
    REG_WRITE(SENS_SAR_ATTEN1_REG, 0);

    // Configure ADC width (12-bit)
    REG_WRITE(SENS_SAR_MEAS1_CTRL1_REG, (REG_READ(SENS_SAR_MEAS1_CTRL1_REG) & ~0x3) | 0x2);

    // Configure attenuation (11dB for full range 0-3.6V)
    uint32_t attenuation_bits = (3 << (channel * 3)); // Channel 0 in this case
    REG_WRITE(SENS_SAR_ATTEN1_REG, (REG_READ(SENS_SAR_ATTEN1_REG) & ~(0x7 << (channel * 3))) | attenuation_bits);

    // Debug: Print register values after configuration
    Serial.printf("SENS_SAR_MEAS1_CTRL1_REG: 0x%08X\n", REG_READ(SENS_SAR_MEAS1_CTRL1_REG));
    Serial.printf("SENS_SAR_ATTEN1_REG: 0x%08X\n", REG_READ(SENS_SAR_ATTEN1_REG));

    Serial.println("ADC initialized.");
}

uint16_t read_adc_channel(uint8_t channel) {
    // Select ADC channel
    REG_WRITE(SENS_SAR_MEAS1_MUX_REG, (REG_READ(SENS_SAR_MEAS1_MUX_REG) & ~0xF) | channel);

    // Start ADC conversion
    REG_WRITE(SENS_SAR_START_FORCE_REG, REG_READ(SENS_SAR_START_FORCE_REG) | (1 << 1));

    // Wait for ADC conversion to complete with timeout
    uint32_t timeout = 100000; // Increased timeout for stability
    while ((REG_READ(SENS_SAR_START_FORCE_REG) & (1 << 1)) && timeout > 0) {
        timeout--;
    }

    if (timeout == 0) {
        Serial.println("ADC conversion timeout!");
        return 0;
    }

    // Read ADC value
    uint16_t adc_result = REG_READ(SENS_SAR1_STATUS_REG) & 0xFFF; // Extract 12-bit value
    return adc_result;
}

void loop() {
    Serial.println("Reading ADC...");
    uint16_t adc_value = read_adc_channel(channel); // ADC1_CHANNEL_0 (GPIO36)
    Serial.printf("ADC Value: %d\n", adc_value);
    delay(500);
}
