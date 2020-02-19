

/* 
    Bandsaw Controller

    Arduino UNO code to manage a customized bandsaw with a 3 Phase motor driven with a VFD. Takes
    analog inputs from the VFD for Motor Speed and Output Current as well as analog inputs from
    two TMP36 linear temperature sensors for motor temp and electronics temp. Output is to a 20x4
    LCD over I2C and three LED's via Digital Ouput.

    The circuit diagram can be seen at: https://easyeda.com/mikecentola/bandsaw-control-circuit

    Created by Mike Centola
    Last Modified: 07 FEB 2020

    License is MIT 

    Source Code: https://github.com/appliedengdesign/bandsaw-controller


    Motor Speed comes from VFD as 0 - 10V for 0 to 2 * Rated Speed (1725 in this case) and goes
    through a voltage divider to provide 0 - 5V to A0. Therefore, 0V is 0 RPM and 5V is 3650 RPM.

    Output Current also comes from VFD as 0 - 10V for 0 to 2 * Max Rated current (20A) and goes
    through the voltage divider to provide 0 - 5V to A1. 0V is 0A and 5V is 40A

    Motor temp and Electronics temp comes from two TMP36 sensors which have a range of -40C to 150C
    and output rougly 0.15V @ -40C to about 1.75V @ 150C.

    The 10-bit ADC reads 0 - 1023 which can be converted to any of the values using the formulas.

*/

#define ARDUINO 10810

#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <PCF8574.h>
#include <Adafruit_MCP4725.h>

// Define LCD Parameters
#define LCD_I2C 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

// Define PCF8574 Parameters
#define PCF_I2C 0x20

// Define Analog Pins
#define MOTOR_SPD 0
#define OUT_CUR 1
#define MOTOR_TEMP 2
#define ELEC_TEMP 3

// Define digital LED Output
#define REDLED P0
#define YELLED P1
#define GRNLED P2

// Define Rotary Switch Inputs 
#define POS1 P3
#define POS2 P4 
#define POS3 P5 
#define POS4 P6
#define POS5 P7 

// Define VFD Max Values
#define SPDMAX 3650
#define CURMAX 40

// Define Speed Values in frequency
#define SPD1 45
#define SPD2 60
#define SPD3 75
#define SPD4 90
#define SPD5 105

#define FREQMAX 105

// Define DAC Address
#define DACADDR 0x62

// Define Pulley Sizes
#define SMPUL 3.75
#define LGPUL 7.75
#define SAWPUL 11.375

// Initialize LCD Library
LiquidCrystal_PCF8574 lcd(LCD_I2C);

// Initialize PCF8574 Expander
PCF8574 pcf(PCF_I2C);

// Initialize DAC
Adafruit_MCP4725 dac;


// Global Variables
int curswpos = 0;


void setup()
{
    int error;

    Serial.begin(115200);
    Serial.println("Initializing...");

    // Check for i2c comms
    Wire.begin();
    Wire.beginTransmission(LCD_I2C);
    error = Wire.endTransmission();

    if (error == 0) {
        // LCD Found, Initialize
        Serial.println("LCD Found");
        lcd.begin(LCD_COLS, LCD_ROWS);
        lcd.noAutoscroll();

        lcd.setCursor(5,1);
        lcd.print("BANDSAW 1.0");
        lcd.setCursor(0,2);
        lcd.print("APPLIED ENG & DESIGN");
        delay(2000);
        lcd.clear();
        lcd.setCursor(0,0);

        lcd.print("Initializing");
        delay(500);
    } else
    {
        // No i2c comms
        Serial.println("Unable to initialize LCD...");

    }

    Serial.println("Setting Digital Outputs...");

    // Setup LED Outputs
    pcf.pinMode(REDLED, OUTPUT);
    pcf.pinMode(YELLED, OUTPUT);
    pcf.pinMode(GRNLED, OUTPUT);

    digitalWrite(REDLED, HIGH);
    digitalWrite(YELLED, HIGH);
    digitalWrite(GRNLED, HIGH);

    lcd.print(".");
    delay(500);

    Serial.println("Setting Digital Inputs...");

    // Setup Rotary Switch
    pcf.pinMode(POS1, INPUT);
    pcf.pinMode(POS2, INPUT);
    pcf.pinMode(POS3, INPUT);
    pcf.pinMode(POS4, INPUT);
    pcf.pinMode(POS5, INPUT);

    lcd.print(".");
    delay(500);

    // Initialize PCF 
    Serial.println("Initializing PCF8574...");
    pcf.begin();

    lcd.print(".");
    delay(500);

    // Initialize DAC
    Serial.println("Initializing DAC");
    dac.begin(DACADDR);



    // Done Initializing
    Serial.println("Initialize Complete");
    digitalWrite(GRNLED, HIGH);
    lcd.clear();
    lcd.setCursor(0,0);

    Serial.println("Running...");
	
} // setup()

void loop()
{
    // GET CURRENT SPEED SWITCH POSITION
    int switchpos = getSwitchPos();

    if (switchpos != curswpos) {
        // Set Motor Speed
        setMotorSpeed(switchpos);
        curswpos = switchpos;
    }

    // READ MOTOR SPEED
    float mspeed = getMotorSpeed();

    // READ OUTPUT CURRENT 
    float outcur = getOutputCurrent();

    // READ MOTOR TEMP 
    float mtemp = getMotorTemp();

    // READ ELECTRONICS TEMP 
    float etemp = getElectronicsTemp();

    // UPDATE DISPLAY 
	


    delay(2000);

} // loop()


// Get Switch Position 
int getSwitchPos() {

    uint8_t val;

    val = pcf.digitalRead(POS1);
    if (val == HIGH) return 1;

    val = pcf.digitalRead(POS2);
    if (val == HIGH) return 2;

    val = pcf.digitalRead(POS3);
    if (val == HIGH) return 3;

    val = pcf.digitalRead(POS4);
    if (val == HIGH) return 4;

    val = pcf.digitalRead(POS5);
    if (val == HIGH) return 5;

}

void setMotorSpeed(int swpos) {

    uint32_t anout;
    float modifier = 0.0;

    // Set Analog Output based on switch position
    switch(swpos) {
        case 1:
            modifier = SPD1 / FREQMAX;
            break;
        
        case 2:
            modifier = SPD2 / FREQMAX;
            break;

        case 3:
            modifier = SPD3 / FREQMAX;
            break;

        case 4:
            modifier = SPD4 / FREQMAX;
            break;

        case 5:
            modifier = SPD5 / FREQMAX;
            break;
    }

    anout = modifier * 4095;

    // Set DAC voltage
    dac.setVoltage(anout, false);
}

// Convert ADC Functions
float getMotorSpeed() {
    int i;
    int sval = 0;

    for (i = 0; i < 5; i++) {
        sval = sval + analogRead(MOTOR_SPD);
    }

    // Take the average of 5 readings 
    float svalavg = sval / 5.0;
    float mspd = svalavg / 1024 * SPDMAX;

    return mspd;
}

float getOutputCurrent() {
    int i;
    int sval = 0;

    for (i = 0; i < 5; i++) {
        sval = sval + analogRead(OUT_CUR);
    }

    // Take the average of 5 readings 
    float svalavg = sval / 5.0;
    float ocur = svalavg / 1024 * CURMAX;

    return ocur;
}

float getMotorTemp() {
    int i;
    int sval = 0;

    for (i = 0; i < 5; i++) {
        sval = sval + analogRead(MOTOR_TEMP);
    }

    // Take average of 5 readings and convert to volts
    float svalavg = sval / 5.0;
    float volts = svalavg * 5.0;
    volts /= 1024.0;

    // Convert to temperature
    float tempC = (volts - 0.5) * 100;

    return tempC;

}

float getElectronicsTemp() {
    int i;
    int sval = 0;

    for (i = 0; i < 5; i++) {
        sval = sval + analogRead(ELEC_TEMP);
    }

    // Take average of 5 readings and convert to volts
    float svalavg = sval / 5.0;
    float volts = svalavg * 5.0;
    volts /= 1024.0;

    // Convert to temperature
    float tempC = (volts - 0.5) * 100;

    return tempC;
}

float convertMspdToSFM(float motorspeed) {

    float rpm = (motorspeed * (SMPUL / LGPUL)) / 20;

    float sfm = (3.14159 / (60 / rpm) * SAWPUL) * 60 / 12;

    return sfm;
}