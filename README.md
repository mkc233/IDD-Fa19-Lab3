# Data Logger (and using cool sensors!)

*A lab report by John Q. Student.*

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

### 3. IR Proximity Sensor

**a. Describe the voltage change over the sensing range of the sensor. A sketch of voltage vs. distance would work also. Does it match up with what you expect from the datasheet?**

**b. Upload your merged code to your lab report repository and link to it here.**

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
