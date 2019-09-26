# Data Logger (and using cool sensors!)

*A lab report by Michael Chan.*

## In The Report

Include your responses to the bold questions on your own fork of [this lab report template](https://github.com/FAR-Lab/IDD-Fa18-Lab2). Include snippets of code that explain what you did. Deliverables are due next Tuesday. Post your lab reports as README.md pages on your GitHub, and post a link to that on your main class hub page.

For this lab, we will be experimenting with a variety of sensors, sending the data to the Arduino serial monitor, writing data to the EEPROM of the Arduino, and then playing the data back.

## Part A.  Writing to the Serial Monitor
 
**a. Based on the readings from the serial monitor, what is the range of the analog values being read?**

The range of analog values is 0 to 1023.
 
**b. How many bits of resolution does the analog to digital converter (ADC) on the Arduino have?**

It has 10 bits of resolution. 

## Part B. RGB LED

**How might you use this with only the parts in your kit? Show us your solution.**


## Part C. Voltage Varying Sensors 
 
### 1. FSR, Flex Sensor, Photo cell, Softpot

**a. What voltage values do you see from your force sensor?**

It shows 0 to 1023 analog voltage which is about 0 to 5 volts.

**b. What kind of relationship does the voltage have as a function of the force applied? (e.g., linear?)**

The relationship between voltage and force applied is logarithmic with a minimum activation force needed to start outputting a voltage per the specification sheet.

**c. Can you change the LED fading code values so that you get the full range of output voltages from the LED when using your FSR?**

We can update the code to include the A0 pin as the FSR input.  Then we can create conditional statements to change the brightness of the LED depending on how much force is being applied to the FSR.  For example the code below.

```
int led = 9;           // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(led, OUTPUT);
  pinMode(A0, INPUT);
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
   Serial.println(analogRead(A0));
  // wait a bit for the analog-to-digital converter to stabilize after the last
  // reading:
  delay(2);
  // set the brightness of pin 9:
  analogWrite(led, brightness);
  if(analogRead(A0) >= 1000){
    brightness = 255;
  }
  else if (analogRead(A0) < 1000 && analogRead(A0) >= 500)  {
    brightness = 150;
  }
  else if (analogRead(A0) < 500 && analogRead(A0) >= 250){
    brightness = 50;
  }
  else{
    brightness = 0;
  }
  delay(30);
}
```
**d. What resistance do you need to have in series to get a reasonable range of voltages from each sensor?**

For the softpot I used 2 10k Ohm resistors, one on the ground side, and one on the power side to get 1/3 to 2/3 range of the 5 volts.
For the photo cell i used a 10K Ohm resistor to get a reasonable voltage. 

**e. What kind of relationship does the resistance have as a function of stimulus? (e.g., linear?)**

The softpot resistance increases/decreases linearly depending on what location of the strip you apply pressure to.
The photo cell resistance increases at a logarithimic rate compared to the amount light.

### 2. Accelerometer
 
**a. Include your accelerometer read-out code in your write-up.**
```
// Basic demo for accelerometer readings from Adafruit LIS3DH

//Accel Setup
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

//Display Setup
// include the library code:
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/*
Adafruit Arduino - Lesson 3. RGB LED
*/
 
int redPin = 7;
int greenPin = 9;
int bluePin = 8;
 
//uncomment this line if using a Common Anode LED
#define COMMON_ANODE

//#Void 


void setup(){
  setupAccel();
  setupDisplay();
  setupLED();
}

void loop(){
  loopAccel();
  loopDisplay();
}

void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}


//************Accel

void setupAccel(void) {
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println("LIS3DH test!");
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");
}

void loopAccel() {
  lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("X:  "); Serial.print(lis.x); 
  Serial.print("  \tY:  "); Serial.print(lis.y); 
  Serial.print("  \tZ:  "); Serial.print(lis.z); 

  /* Or....get a new sensor event, normalized */ 
  sensors_event_t event; 
  lis.getEvent(&event);
  
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y); 
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z); 
  Serial.println(" m/s^2 ");

  Serial.println();
 
  delay(200); 
}


//*************Display

void setupDisplay() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("X: ");
  lcd.setCursor(0,1);
  lcd.print("Y: ");
  lcd.setCursor(8,0);
  lcd.print("Z: ");
}

void loopDisplay() {
  lis.read();
  sensors_event_t event; 
  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;
  lcd.setCursor(2,0);
  lcd.print(x);
  lcd.setCursor(2,1);
  lcd.print(y);
  lcd.setCursor(10,0);
  lcd.print(z);
  if(abs(x-0) > abs(y-0) && abs(x-0) > abs(z-9.86)){
    setColor(255,0,0);
  }
  else if(abs(y-0) > abs(x-0) && abs(y-0) > abs(z-9.86)){
    setColor(0,255,0);
  }
  else if(abs(z-9.86) > abs(y-0) && abs(z-9.86) > abs(x-0)){
    setColor(0,0,255);
  } 
  delay(2000);

}

 
void setupLED()
{
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  
}
```

## Optional. Graphic Display

**Take a picture of your screen working insert it here!**

## Part D. Logging values to the EEPROM and reading them back
 
### 1. Reading and writing values to the Arduino EEPROM

**a. Does it matter what actions are assigned to which state? Why?**

**b. Why is the code here all in the setup() functions and not in the loop() functions?**

**c. How many byte-sized data samples can you store on the Atmega328?**

**d. How would you get analog data from the Arduino analog pins to be byte-sized? How about analog data from the I2C devices?**

**e. Alternately, how would we store the data if it were bigger than a byte? (hint: take a look at the [EEPROMPut](https://www.arduino.cc/en/Reference/EEPROMPut) example)**

**Upload your modified code that takes in analog values from your sensors and prints them back out to the Arduino Serial Monitor.**

### 2. Design your logger
 
**a. Insert here a copy of your final state diagram.**

### 3. Create your data logger!
 
**a. Record and upload a short demo video of your logger in action.**
