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

#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>

// Define the i2c address for the LCD display
#define LCD_I2C 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

// Define Analog Pins
#define MOTOR_SPD 0
#define OUT_CUR 1
#define MOTOR_TEMP 2
#define ELEC_TEMP 3

// Define digital LED Output
#define REDLED 6
#define YELLED 7
#define GRNLED 8

// Define VFD Max Values
#define SPDMAX 3650
#define CURMAX 40

// Initialize LCD Library
LiquidCrystal_PCF8574 lcd(LCD_I2C);


// Global Variables
int val = 0;


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

        lcd.setCursor(5,1);
        lcd.print("BANDSAW 1.0");
        lcd.setCursor(0,2);
        lcd.print("APPLIED ENG & DESIGN");
        delay(2000);
        lcd.clear();
        lcd.setCursor(0,0);

        lcd.print("Initializing...");
    } else
    {
        // No i2c comms
        Serial.println("Unable to initialize LCD...");

    }
    

	
} // setup()

void loop()
{

    // READ MOTOR SPEED

    // READ OUTPUT CURRENT 

    // READ MOTOR TEMP 

    // READ ELECTRONICS TEMP 

    // UPDATE DISPLAY 
	
} // loop()


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
