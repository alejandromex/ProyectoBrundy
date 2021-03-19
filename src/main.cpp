/*
Wiring from Sensor
green
  gpio to provide power via HIGH/LOW
grn/wht split
  1 - resistor to GPIO for ground via HIGH/LOW
  2 - analog read
*/

#include <Arduino.h>
#include <math.h>
#include <heltec.h>
#include <limits>

//*****Define Sensor Settings*****
// BOARD SPECIFIC OPTIONS
// Set pins used
const int analogReadPin=4;
const int digitalPin1=2;
const int digitalPin2=15;
// Set ADC value
//const int adcValue=255;   // For 8-bit adc
//const int adcValue=1024;  // For 10-bit adc
const int adcValue=4096;    // For 12-bit adc

const double calibrationValue=0.85; //This value set after testing 10k resistor to calibrate emaured value vs resistor value

#define num_of_read 3 //READ THE SENSOR X NUMBER OF TIMES..SHOULD THE SUCCESIVE READINGS VARY
const int Rx = 8000;  //fixed resistor attached in series to the sensor and ground...the same value repeated for all WM and Temp Sensor.
// const int Rx = 10000;  //fixed resistor attached in series to the sensor and ground...the same value repeated for all WM and Temp Sensor.
const long open_resistance=35000; //check the open resistance value by replacing sensor with an open and replace the value here...this value might vary slightly with circuit components 
const long short_resistance=200; // similarly check short resistance by shorting the sensor terminals and replace the value here.
const long short_CB=240;
const long open_CB=255;
int i;                            
int j=0;                          

double ARead_A1=0;

double SupplyV=3.3; // Assuming 3.3V output
int SenV10K=0;
int TempC=24;
double SenVWM1=0;
int WM1_CB=0;

void readSensor();
void wakeSensor(); 


void printMessage(String message, double value)
{
  Serial.println("");
  Serial.println("====="+message+"====");
  Serial.println((String)value);
  Serial.println("=================");
  Serial.println("");
}


void setup()
{
  Serial.begin(9600);
  wakeSensor();
}


bool repeating = true;

void loop()
{
  readSensor();
}


void wakeSensor() {
  Serial.println("Waking sensor...");

  // initialize the digital pin as an output.
  pinMode(digitalPin1, OUTPUT);  //Sensor Vs or GND
  pinMode(digitalPin2, OUTPUT);  //Sensor Vs or GND
  digitalWrite(digitalPin2, LOW);
  delay(100);   // time in milliseconds, wait 0.1 minute to make sure the OUTPUT is assigned
}

void readSensor () 
{
  Serial.println("Reading sensor...");

  // Start reading data
  ARead_A1=0;

  for (i=0; i<num_of_read; i++) {  //the num_of_read initialized above, controls the number of read successive read loops that is avereraged. 

    //**********LETS READ THE WM1 SENSOR**************
    digitalWrite(digitalPin1, HIGH);   //Set pin 5 as Vs

    delay(0.09); //wait 90 micro seconds and take sensor read...do not exceed 100uS
    ARead_A1+=analogRead(analogReadPin);   // read the sensor voltage right after the sensor and right before the series resistor Rx-7870
    Serial.print("Analog Read Val: ");
    Serial.println(ARead_A1);
    digitalWrite(digitalPin1, LOW);      //set the excitation voltage to OFF/LOW

    delay(100); //0.1 second wait before moving to next channel or switching MUX
    
    // Now lets swap polarity
    digitalWrite(digitalPin2, HIGH); //Set pin 11 as Vs
    delay(0.09); //wait 90 micro seconds and take sensor read...do not exceed 100uS

   digitalWrite(digitalPin2, LOW);      //set the excitation voltage to OFF/LOW
  }
  // adcEnd(analogReadPin);
  //assuming 12 bit adc, supply volt set above at 3.3

  

  //SENVWM1 tiene que valer 0.7695473251
  // ARead_A1 tiene que valer 2865.514403


  SenVWM1=((ARead_A1/adcValue)*SupplyV) / (num_of_read); //get the average of the readings and convert to volts
  Serial.println(SenVWM1);

  // if(ARead_A1 < 1)
  // {
  //   SenVWM1 = 00.7695473251;
  // }
  double test = (RX*(SupplyV - (ARead_A1/3))/(ARead_A1/3));
  printMessage("TEST",test);
  double WM1_Resistance = (Rx*(SupplyV-SenVWM1)/SenVWM1)*calibrationValue; //do the voltage divider math, using the Rx variable representing the known resistor  used
  
  printMessage("WMI_Resistence", WM1_Resistance);
  Serial.println(WM1_Resistance);

  //*****************CONVERSION OF RESISTANCE TO ACTUAL VALUE ************************************

  //convert WM1 Reading to Centibars or KiloPascal
  // The trailing 00 for numbers are very critical in the formula below eg: 1 is written as 1.00, else code messes math due to int vs double for numbers
  if (WM1_Resistance>550.00) {
    
    if (WM1_Resistance>8000.00) 
    {
      WM1_CB=-2.246-5.239*(WM1_Resistance/1000.00)*(1+.018*(TempC-24.00))-.06756*(WM1_Resistance/1000.00)*(WM1_Resistance/1000.00)*((1.00+0.018*(TempC-24.00))*(1.00+0.018*(TempC-24.00))); 
      printMessage("WM1_CB", WM1_CB);
      Serial.print("Entered WM1 >8000 Loop \n");
      
    } else if (WM1_Resistance>1000.00) {

      WM1_CB=(-3.213*(WM1_Resistance/1000.00)-4.093)/(1-0.009733*(WM1_Resistance/1000.00)-0.01205*(TempC)) ;
      Serial.print("Entered WM1 >1000 Loop \n");

    } else {

      WM1_CB=-20.00*((WM1_Resistance/1000.00)*(1.00+0.018*(TempC-24.00))-0.55);
      Serial.print("Entered WM1>550 Loop \n");
    }
    
  } else {

    if(WM1_Resistance>300.00)  {
      WM1_CB=0.00;
      Serial.print("Entered 550<WM1>0 Loop \n");
    }
    
    if(WM1_Resistance<300.00 && WM1_Resistance>=short_resistance) {
      
      WM1_CB=short_CB; //240 is a fault code for sensor terminal short
      Serial.print("Entered Sensor Short Loop WM1 \n");
    }
  }

  if(WM1_Resistance>=open_resistance) {
    WM1_CB=open_CB; 
    printMessage("WM1_CB", WM1_CB);
    Serial.print("Entered Open or Fault Loop for WM1 \n");
  }

  Serial.print("WM1 Resistance(Ohms)= ");
  Serial.print(WM1_Resistance);
  Serial.print("\n");
  Serial.print("WM1(CB)= ");
  Serial.print(abs(WM1_CB));
  Serial.print("\n");
  //****************END CONVERSION BLOCK********************************************************
}
