/*

IVy: Arduino based Solar Cell Test Platform based on original IVy tester by 
Rupak Chakraborty and David Berney Needleman, MIT PV Lab.

This software is meant as a alternative for using Keithley test equipment with the 
addition of extra inputs for Temperature sensors and Irradiance measurements.

Tester is based on Arduino shield running of 3.3V or 5V with only power and I2C 
used from the Arduino.

DAC
  Output is based on a MCP4728 Quad 12bit Digital-to-Analog Converter (DAC) 
  Internal reference voltage of 2.048V and a selectable 1x /2x PGA

  The first two channels on the DAC are used as V+ and V- on the device under test (DUT).
  Both are buffered through a TLV4113 two channel singly-supply rail-to-rail op-amp


ADC
  The measurement is completed using a MCP3424 four channel low-noise delta-sigma 
  Analog-to-Digital Converter (ADC). 
  
  The ADC has four ±2.048V differential inputs with a 1x, 2x, 4x and 8x PGA, enabling
  a differential full range down to ±0.256V
  The resolution can be set between 12bit and 18 bit with samples per second (SPS) varying.
  3.75 SPS (18 bits)
  15 SPS (16 bits)
  60 SPS (14 bits)
  240 SPS (12 bits)

*/

/*

//Setting DAC channels
  dac.analogWrite(500,500,500,500); // write to input register of DAC four channel (channel 0-3) together. Value 0-4095
  dac.analogWrite(0,700); // write to input register of a DAC. Channel 0-3, Value 0-4095
  dac.voutWrite(1800, 1800, 1800, 1800); // write to input register of DAC. Value(mV) (V < VDD)
  dac.voutWrite(2, 1600); // write to input register of DAC. Channel 0 - 3, Value(mV) (V < VDD)

//and current values from DAC
  int value = dac.getValue(0); // get current input register value of channel 0
  int vout = dac.getVout(1); // get current voltage out of channel 1

  Channels are used for:
      Va (0) : Voltage on Negative pin on DUT (unity gain through OP-AMP)
      Vb (1) : Voltage on Positive pin on DUT (unity gain through OP-AMP)
      Vc (2) : Voltage on Negative pin on Photodiode (possible to keep it negatively biased)
      Vd (3) : Voltage on connector VC-GND-VD


//Reading ADC channels
  // get channel data with function
  double CH1 = adc.getChannelmV(0);
  double CH2 = adc.getChannelmV(1);
  Serial.print("Channel 1: ");Serial.print(CH1);Serial.println(" mV");
  
  Channels are used for:
    Channel 1 (0) : Voltage of CN1 (DUT); taken over voltage divider
    Channel 2 (1) : Voltage over Rshunt ; Current through DUT
    Channel 3 (2) : Voltage of R1 on PH1; Current through photodiode (R1 = 1ohm)
    Channel 4 (3) : Direct voltage measurement of V_mes; could be used for pyrometer
*/


// IVy Original variables
  // assign default values for center, start and stop voltages and number of data points:
  float startV = -2.0;
  float stopV = 2.0;
  int numPoints = 80;
  
  // assign voltage limits
  float minV = -4.0;
  float maxV = 4.0;
  
  // assign numPoints limits - MUST MATCH WITH CORRESPONDING VARIABLES IN PROCESSING CODE
  float minNumPoints = 0;
  float maxNumPoints = 254;
  
  // for establishing serial contact
  int inByte = 0;
  boolean firstContact = false;
  
  //current limit: the op-amp has a 200mA rating
  float currentLimit = 300; // in mA, will take +/- currentLimit as upper and lower limits


  boolean enableSDCard = 0;     // Enables saving data to SDCard.
  boolean enableRTC = 0;        // Enables Real Time Clock module for storage time and date for the measurement
  boolean enableEthernet = 0;   // Enables the ethernet interface for sending measurements to a server.
  
  
// Include libraries
#include <Wire.h>
#include <SPI.h>
#include "mcp4728.h" //DAC chip
//#include "mcp3424.h" //ADC chip

///*
#include <OneWire.h>
#include <DallasTemperature.h>

#include <Ethernet.h>
#include <SD.h>

//#include <dht.h>
//*/



// I2C address for MCP3422 - base address for MCP3424 = 0x68
#define MCP342X_ADDRESS 0X68    // changed to avoid conflict with DS1307 clock in logger

// fields in configuration register
#define MCP342X_GAIN_FIELD 0X03 // PGA field
#define MCP342X_GAIN_X1    0X00 // PGA gain X1
#define MCP342X_GAIN_X2    0X01 // PGA gain X2
#define MCP342X_GAIN_X4    0X02 // PGA gain X4
#define MCP342X_GAIN_X8    0X03 // PGA gain X8

#define MCP342X_RES_FIELD  0X0C // resolution/rate field
#define MCP342X_RES_SHIFT  0X02 // shift to low bits
#define MCP342X_12_BIT     0X00 // 12-bit 240 SPS
#define MCP342X_14_BIT     0X04 // 14-bit 60 SPS
#define MCP342X_16_BIT     0X08 // 16-bit 15 SPS
#define MCP342X_18_BIT     0X0C // 18-bit 3.75 SPS

#define MCP342X_CONTINUOUS 0X10 // 1 = continuous, 0 = one-shot

#define MCP342X_CHAN_FIELD 0X60 // channel field
#define MCP342X_CHANNEL_1  0X00 // select MUX channel 1
#define MCP342X_CHANNEL_2  0X20 // select MUX channel 2
#define MCP342X_CHANNEL_3  0X40 // select MUX channel 3
#define MCP342X_CHANNEL_4  0X60 // select MUX channel 4

#define MCP342X_START      0X80 // write: start a conversion
#define MCP342X_BUSY       0X80 // read: output not ready
uint8_t chan = 0XFF, gain = 0XFF, res = 0XFF;

//------------------------------------------------------------------------
// default adc configuration register - resolution and gain added in setup()
uint8_t adcConfig = MCP342X_START | MCP342X_CHANNEL_1 | MCP342X_CONTINUOUS;
// divisor to convert ADC reading to milivolts
uint16_t mvDivisor;

//------------------------------------------------------------------------------
void halt(void)
{
   Serial.println("Halted - check address and wiring");
   while(1);
}
//------------------------------------------------------------------------------
// read mcp342x data - updated 10mar11/wbp
uint8_t mcp342xRead(int32_t &data)
{
   // pointer used to form int32 data
   uint8_t *p = (uint8_t *)&data;
   // timeout - not really needed?
   uint32_t start = millis();
   if ((adcConfig & MCP342X_RES_FIELD) == MCP342X_18_BIT)  // in 18 bit mode?
   {
      do {   // 18-bit mode
         Wire.requestFrom(MCP342X_ADDRESS, 4);
         if (Wire.available() != 4) {
            Serial.println("read failed");
            return false;
         }
         for (int8_t i = 2; i >= 0; i--) {
            p[i] = Wire.read();
         }
         // extend sign bits
         p[3] = p[2] & 0X80 ? 0XFF : 0;
         // read config/status byte
         uint8_t s = Wire.read();
         if ((s & MCP342X_BUSY) == 0) return true;  // escape here
      } 
      while (millis() - start < 500);   // allows rollover of millis()
   }
   else
   {
      do {  // 12-bit to 16-bit mode
         Wire.requestFrom(MCP342X_ADDRESS, 3);
         if (Wire.available() != 3) {
            Serial.println("read failed");
            return false;
         }
         p[1] = Wire.read();
         p[0] = Wire.read();
         // extend sign bits
         p[2] = p[1] & 0X80 ? 0XFF : 0;
         p[3] = p[2];
         // read config/status byte
         uint8_t s = Wire.read();
         if ((s & MCP342X_BUSY) == 0) return true;  // or escape here
      } 
      while (millis() - start < 500);   // allows rollover of millis()
   }
   Serial.println("read timeout");      // dang it
   return false;
}

//------------------------------------------------------------------------------
// write mcp342x configuration byte
uint8_t mcp342xWrite(uint8_t config)
{
   Wire.beginTransmission(MCP342X_ADDRESS);
   Wire.write(config);
   Wire.endTransmission();
}



// Initiate DAC
  mcp4728 dac = mcp4728(0); // instantiate mcp4728 object, Device ID = 0


void setup() {

  Serial.begin(9600);
  //Wire.begin();
  
  dac.begin();  // initialize i2c interface
  dac.vdd(5000); // set VDD(mV) of MCP4728 for correct conversion between LSB and Vout

  resetVOpAmp();
  // send a byte to establish contact with the Processing applet until it responds
  establishContact(); 
}

void resetVOpAmp() {
  //Reset DAC
  dac.analogWrite(0,0,0,0);

}

/* read in four-byte array from Processing applet */
float readSerial() {
  long byteOne = (long)Serial.read();
  long byteTwo = (long)Serial.read();
  long byteThree = (long)Serial.read();
  long byteFour = (long)Serial.read();
        
  long byteTwoShift = byteTwo << 8;
  long byteThreeShift = byteThree << 16;
  long byteFourShift = byteFour << 24;
        
  long inputLong = byteOne | byteTwoShift | byteThreeShift | byteFourShift;
  float input = (float)inputLong / 1000000.0;
  return input;
}

/* this loop handles handshaking with the Processing applet */
void loop(){
  //For running with the Processing Applet
    AppletConnected();

  
}


void AppletConnected(){
  if (Serial.available() > 0){
    int firstByte = Serial.read();
    
    if (firstByte == 'C'){  // the char 'C' from the Processing applet indicates it is trying to reconnect
      Serial.println("CONNECT");  // signal to applet that it's connected
    }
    else if(firstByte == 'S'){  // the char 'S' from the Processing applet indicates that the applet is about to send three bytes indicating startV, stopV, numPoints
        delay(40);  // give the applet time to send the three bytes
        // read in all the bytes from the applet
        
        /*
        startV = readSerial();
        stopV = readSerial();
        numPoints = (int)readSerial();
        
        */
        /*
        // for debug
        Serial.print("startV: ");
        Serial.println(startV,6);
        Serial.print("stopV: ");
        Serial.println(stopV,6);
        Serial.print("numPoints: ");
        Serial.println(numPoints,4);      
        */
        runVoltageSweep(); // get and send data
    }
    else{
      Serial.flush();  // in case of garbage serial data, flush the buffer
    }
  }
}


void establishContact() {
  // send CONNECT signal until the applet responds
  while (Serial.available() <= 0) {
    Serial.println("CONNECT");
    delay(300);
  }
  
  // after response is received, make sure the response is a 'C'
  inByte = Serial.read();
  Serial.println("CONNECT");
  if( inByte != 'C' ){
    establishContact();
  }
}

void runVoltageSweep() {
  
  boolean currentOverload = false; // indicates currentOverload during sweep
  
  // create variable for start and stop voltage DAC level:
  int startVLevel = startV * 1000;
  int stopVLevel = stopV * 1000;
  
  // create increment to obtain numPoints number of data points
  int stepLevel = (stopVLevel - startVLevel) / (numPoints);

  Serial.println("\n **** Starting new measurement **** \n");
  
  Serial.print("Starting Voltage = ");
  Serial.print(startVLevel);
  Serial.println(" mV");
  Serial.print("Stop Voltage \t =  ");
  Serial.print(stopVLevel);
  Serial.println(" mV");
  Serial.print("Voltage Step \t =    ");
  Serial.print(stepLevel);
  Serial.println(" mV");  
  
  // define current and voltage variables:
  float current;
  float deviceVoltage;
  int sweep; //voltage sweep function
  int t_start = millis();
  //Offset on the DAC to lift the Virtual GND - One terminal at offset instead of zero voltage.
  int offset = 50;
  
  Serial.println("DATA");  // signal to applet that data is coming
  // sweep voltage on channel A:
  
  
  for (int level = startVLevel; level <= stopVLevel; level += stepLevel) {
    if (level<0){
      int Vpos = dac.analogWrite(0,0+offset);
      int Vneg = dac.analogWrite(1,-level+offset);
    }
    else{
      int Vpos = dac.analogWrite(0,level+offset);
      int Vneg = dac.analogWrite(1,0+offset);
    }
    
    ///*
    Serial.print("Level= ");
    Serial.print(level);
    Serial.print("\t VoutA= ");
    Serial.print(dac.getVout(0));
    Serial.print("\t VoutB= ");
    Serial.print(dac.getVout(1));
    Serial.print("\t VoutMeas= \t");
    //*/
    
    measureinput(' ');
  }
  Serial.println("Voltage sweep complete."); // successful sweep.
  Serial.print("Time: ");
  Serial.print((millis()-t_start)*0.001,2);
  Serial.println(" seconds");
  
  resetVOpAmp();
}



void measureinput(char channel) {

 if (channel = 'U'){
    chan = 0; // Channel (Voltage)
    res = 1; // set resolution
    gain = 0; // x1
    mvDivisor = 1 << (gain + 2*res);
    adcConfig = 0;  // if adcConfig not reset, chan gets stuck at 3
    adcConfig |= chan << 5 | res << 2 | gain | MCP342X_START;
    mcp342xWrite(adcConfig);
    int32_t data;
    
    if (!mcp342xRead(data)) halt();
    
    // voltage in millivolts
    double mv = (double)data/mvDivisor*0.001;
    Serial.print(mv*2,5);               // measured over a ½x voltage divider
 }
 
 Serial.print(",");
 
 if (channel = 'I'){
    chan = 1; // Channel (Current)
    res = 3; // set resolution
    gain = 3; // x8
    mvDivisor = 1 << (gain + 2*res);
    adcConfig = 0;  // if adcConfig not reset, chan gets stuck at 3
    adcConfig |= chan << 5 | res << 2 | gain | MCP342X_START;
    mcp342xWrite(adcConfig);
    int32_t data;
    
    if (!mcp342xRead(data)) halt();
    
    // voltage in millivolts
    double mv = (double)data/mvDivisor;
    // current in mA
    // Serial.print(mv*10,4);               // measured over a 0.1 Ohm shunt resistor
    Serial.println(mv,4);               // measured over a 1 Ohm shunt resistor
 }

}
