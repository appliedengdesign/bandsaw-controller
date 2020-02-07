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

*/

#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>

// Define the i2c address for the LCD display
#define LCD_I2C 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

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
