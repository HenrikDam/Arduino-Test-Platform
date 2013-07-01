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
  
  The ADC has four Â±2.048V differential inputs with a 1x, 2x, 4x and 8x PGA, enabling
  a differential full range down to Â±0.256V
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

//and getting current values from DAC
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
  float startV = -1.0;
  float stopV = 1.0;
  int numPoints = 40;
  
  // assign voltage limits
  float minV = -4.0;
  float maxV = 4.0;
  
  // assign numPoints limits - MUST MATCH WITH CORRESPONDING VARIABLES IN PROCESSING CODE
  int minNumPoints = 0;
  int maxNumPoints = 254;
  
  // for establishing serial contact
  int inByte = 0;
  boolean Cmode = false;
  boolean firstContact = false;
  
  //current limit: the op-amp has a 200mA rating -- opamp not connected
  float currentLimit = 10; // in mA, will take +/- currentLimit as upper and lower limits


  boolean enableSDCard = false;     // Enables saving data to SDCard.
  boolean enableRTC = false;        // Enables Real Time Clock module for storage time and date for the measurement
  boolean enableEthernet = false;   // Enables the ethernet interface for sending measurements to a server.
  
  char inData[4]; // Allocate some space for the string
  char inChar=-1; // Where to store the character read
  byte index = 0; // Index into array; where to store the character


  boolean debug = false;
  
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

void softReset(){
  asm volatile ("  jmp 0");
}

//------------------------------------------------------------------------------
void halt(void)
{
   Serial.println(F("*ERROR* - Halted - check address and wiring of MCP3424 ADC"));
   Serial.println(F("Press 'r' to reset the program"));
   while(1){
//     delay(100); Serial.print(".");
     if( Serial.available()>0){
       if( Serial.read() == 'r' ){
         Serial.println(F("resetting program \n")); delay(100); softReset();
       }
     }
    }
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
            Serial.println(F("*ERROR* - MCP3424 ADC 18bit read failed"));
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
            Serial.println(F("*ERROR* - MCP3424 ADC 12-16bit read failed"));
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
   Serial.println(F("*ERROR* - ADC read timeout"));      // dang it
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

  Serial.begin(115200);
  //Wire.begin();
  
  dac.begin();  // initialize i2c interface
  dac.vdd(5000); // set VDD(mV) of MCP4728 for correct conversion between LSB and Vout

  resetVOpAmp();
  // send a byte to establish contact with the Processing applet until it responds
  establishContact(); 
  
  pinMode (8, OUTPUT); // initiate opamp
  digitalWrite(8,LOW);  // set shutdown
}

void resetVOpAmp() {
  //Reset DAC - Setting Vref and Gain is needed for correct conversion of bits to voltages.
  dac.analogWrite(0,0,0,0);
  dac.setVref(1,1,1,1);
  dac.setGain(1,1,1,1);
  dac.setPowerDown(0, 0, 3, 3); //( 0 - On; 3 - 500k resistance to GND)
  digitalWrite(8,LOW);
}


void CommandMode(){
   // 4 byte command mode initiated by sending a "*" on the serial
  if(Cmode == 1) {
    Serial.println(F("// Commandmode entered \n "));
    Serial.println(F("// Commands in Commandmode. ('*' is not used when already in command mode)"));
    Serial.println(F("// ---------------------------"));
    Serial.println(F("//  *IDN?  ID of the arduino"));
    Serial.println(F("//  *DBG0  Disable debugging"));
    Serial.println(F("//  *DBG1  Enable debugging"));
  }  

  while (Serial.available() < 4) {
    if(Cmode) { //print periods to show alive
      Serial.print(" .");
      delay(300);
    }
  }
  if(Cmode) {Serial.println();}
  Serial.readBytes(inData,4);
  Serial.flush();  

  if(strcmp(inData,"IDN?")==0){ // Send ID of arduino
      Serial.println(F("Arduino Sourcemeter v.0.95 200mA"));
    }
  if(strcmp(inData,"DBG1")==0){
      debug = true; Serial.println(F("Enabling Debugging"));
    }
  if(strcmp(inData,"DBG0")==0){
      debug = false; Serial.println(F("Disabling Debugging"));
    }
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
    else if(firstByte == 'S'){  // 'S' indicates starting a measurement
      delay(40);
      
        startV = Serial.parseFloat();
        stopV = Serial.parseFloat();
        numPoints = (int)Serial.parseFloat();
        

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
    if (debug) {Serial.println("CONNECT");}
    delay(100);
  }
  
  // after response is received, make sure the response is a 'C' for connecting or * for commands
  inByte = Serial.read();
  if( inByte == '*' ){
    if( Serial.available()<=0) {
      Cmode = true;
      Serial.println(F("ready for command (max 4 byte)"));
    }
    else{ 
      Cmode = false;
    }
    CommandMode();
  }  

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

  Serial.println(F("\n **** Starting new measurement **** \n"));
  
  Serial.print(F("Starting Voltage = "));
  Serial.print(startVLevel);
  Serial.println(" mV");
  Serial.print(F("Stop Voltage \t =  "));
  Serial.print(stopVLevel);
  Serial.println(" mV");
  Serial.print(F("Voltage Step \t =    "));
  Serial.print(stepLevel);
  Serial.println(" mV");  
  
  // define current and voltage variables:
  float current;
  float deviceVoltage;
  int sweep; //voltage sweep function
  //int t_start = millis();

  //Offset on the DAC to lift the Virtual GND - One terminal at offset instead of zero voltage.
  int offset = 1000; // lifted 1V; for voltages above 3 or below -3 offset is zero
  if ((int)abs(startVLevel)>= 3000 || (int)abs(stopVLevel)>= 3000) {
    offset = 0;
  }

  Serial.print(F("Voltage Offset \t =    "));
  Serial.print(offset);
  Serial.println(" mV");  
  Serial.println(F("Enabling OpAmp"));
  digitalWrite(8,HIGH);
  
  Serial.println("DATA");  // signal that data is coming
    
  for (int level = startVLevel; level <= stopVLevel; level += stepLevel) {
    
    // Possibility of stopping measurement
    if(Serial.available()>=4){
      Serial.readBytes(inData,4);
      if(strcmp(inData,"STOP")==0){
        Serial.println(F("Stopping measurement"));
        establishContact();
      }
    }
    if (level<0){
      int Vpos = dac.analogWrite(0,0+offset);
      int Vneg = dac.analogWrite(1,-level+offset);
    }
    else{
      int Vpos = dac.analogWrite(0,level+offset);
      int Vneg = dac.analogWrite(1,0+offset);
    }
    
    if (debug){
      Serial.print(F("Level= "));
      Serial.print(level);
      Serial.print(F("\t VoutA= "));
      Serial.print(dac.getValue(0));
      Serial.print(F("\t VoutB= "));
      Serial.print(dac.getValue(1));
      Serial.print(F("\t VoutMeas= \t"));
    }
    
    measureinput();
  }
  Serial.println(F("Voltage sweep complete.")); // successful sweep.
//  Serial.print("Time: ");
//  Serial.print((millis()-t_start)*0.001,2);
//  Serial.println(" seconds");
  
  resetVOpAmp();
}



void measureinput() {

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
    Serial.print(mv*2,5);               // measured over a Â½x voltage divider
  
    Serial.print(",");
 
    chan = 1; // Channel (Current)
    res = 3; // set resolution
    gain = 3; // x8
    mvDivisor = 1 << (gain + 2*res);
    adcConfig = 0;  // if adcConfig not reset, chan gets stuck at 3
    adcConfig |= chan << 5 | res << 2 | gain | MCP342X_START;
    mcp342xWrite(adcConfig);
//    int32_t data;
    
    if (!mcp342xRead(data)) halt();
    
    // voltage in millivolts
    double mA = (double)data/mvDivisor;
    // current in mA
    Serial.println(mA*10,4);               // measured over a 0.1 Ohm shunt resistor
    //Serial.println(mA,4);               // measured over a 1 Ohm shunt resistor

}

