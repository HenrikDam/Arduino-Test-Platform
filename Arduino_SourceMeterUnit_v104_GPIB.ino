/*

Arduino SourceMeterUnit software
v. 1.03 - HFDA DEBUG FOR BEN
2017/12/5

*/
const float SMUvers = 1.03;   // Version of the SourceMeterUnit software

/*
  DAC : MCP4728; 4 Channel DAC with 4096mV range and 4096 steps (mV). I2C control
  DAC Channels are used for:
      Va (0) : Drain voltage- Channel is looped through transimpedance current measurement circuit with MCP6001 amplifier
      Vb (1) : Source voltage
      Vc (2) : Gate voltage
      Vd (3) : Broken out to Vd via on PCB


//Reading ADC channels
  ADC : MCP3428; 4 Channel differential ADC with +-2048mV range. selectable resolution from 12bit to 16 bit.
  Channels are used for:
    Channel 1 (0) : Voltage between SOURCE A AND B
    Channel 2 (1) : VOLTAGE DROP OVER R_SHUNT (measured over "resistor_size" "4.46 Ohm")
    Channel 3 (2) : CON: CON_PH1 
    Channel 4 (3) : CON: V_MES

Serial command are:

      "*IDN?"    // Send ID of arduino; standard SCPI identifier
      "*RST"     // Reset
      ":SOUR:VOLT:LEV"    // Set voltage for Drain (Channel 0) with SCPI command + standard gate voltage  
      ":SET"     // Set voltage for Drain and Gate to default -1V and -0.3V.
      ":READ?"   // Measure and print to serial.
      ":CONT:ENA(BLE)"  // Enable continuous measurement - Measure every second
      ":CONT:DIS(ABLE)" // Disable continuous measurement  
      ":TEST"   // Run a test
      ":IVC"   // IV curve
*/


// Defining ADC parameters
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
    
  // I2C address for MCP3422 - base address for MCP3424 = 0x68
  #define MCP342X_ADDRESS 0X68
  uint8_t chan = 0XFF, gain = 0XFF, res = 0XFF;
  // default adc configuration register - resolution and gain added in setup()
  uint8_t adcConfig = MCP342X_START | MCP342X_CHANNEL_1 | MCP342X_CONTINUOUS;
  // divisor to convert ADC reading to milivolts
  uint16_t mvDivisor;
// ADC parameter end

// Include libraries
#include <Wire.h>
#include "mcp4728.h" //DAC chip
#include "SSD1306.h" // OLED display driver for esp8266

long int Time = 0;

// Variables for the DAC
int Voltage = 0;
int Volt_on = 0;
bool set_cont = 0;

int analoglevel = 0;

// DAC start
  mcp4728 dac = mcp4728(0); // instantiate mcp4728 object, Device ID = 0

// Display start (address, SDApin, SCLpin)
//  SSD1306  display(0x3c, D4, D3);   //when using the ESP8266 WeMOS D1
  SSD1306  display(0x3c, D2, D1);   //when using the ESP8266 NodeMCU

void setup() {

  Serial.begin(115200);
  Wire.begin();
  dac.begin();  // initialize i2c interface
  dac.vdd(3300); // set VDD(mV) of MCP4728 for correct conversion between LSB and Vout
  resetDAC();

  displayinit();

  Serial.println(" Entering loop");

}

void loop(){


  //analoglevel = analogRead(A0);
  //Serial.println(analoglevel);
  //SetDACvalue(analoglevel);
  //measureinput();

  SerialCheck();
  delay(50);
 
}

void SerialCheck(){
     //Init serial data variables
   char data [21];
   int number_of_bytes_received;

   //If serial data available, check if asked for ID check or assume one wants a measurement. 
   if(Serial.available() > 0)
   {
      delay(40);
      number_of_bytes_received = Serial.readBytesUntil (32,data,20); // read bytes (max. 20) from buffer, untill <CR> (13) (or <SPACE> (32) ). store bytes in data. count the bytes recieved.
      data[number_of_bytes_received] = 0; // add a 0 terminator to the char array
      //Serial.println('\n'); 
      //Serial.println(data);
      
      if(strstr(data,"*IDN?")){ // Send ID of arduino; standard SCPI identifier
          Serial.println("Arduino SourceMeasureUnit 200mA -  v." + String(SMUvers,2));
      }
      else if(strstr(data,"*RST")){ // Reset
          Serial.println(F("Resetting.."));
          //ESP.reset();
          resetDAC();
          Volt_on = 0;  // Voltage on indicator
      }
      else if(strstr(data,":SOUR:VOLT:LEV")){       // Set voltage for Drain (Channel 0) with SCPI command + standard gate voltage  
        digitalWrite(8,HIGH);
        Voltage = ReadSCPIVOltage();
        SetDACvalue(Voltage); // Set voltage output 
        Volt_on = 1;  // Voltage on indicator    
      }
      else if(strstr(data,":SET")){       // Set voltage for Drain and Gate to default -1V and -0.3V.
        SetDACvalue(Voltage); // Set voltage output  
        Volt_on = 1;  // Voltage on indicator    
      }
      
      else if(strstr(data,":READ?")){ // Measure and print to serial.
        READ();
        //measureinput();   // Read voltage and print to serial  
        Volt_on = 2;
      }
      else if(strstr(data,":IVC")){ // Testing procedure
        Serial.println("starting IV scan");
        digitalWrite(8,HIGH);
        Serial.println("DATA");
        for (int level = -1000; level <= 2000; level += 50) {
          //delay(2000);
          SetDACvalue(level);
          measureinput();
        }
        
      }
      else{}
    }
    else{
      Serial.flush();  // in case of garbage serial data, flush the buffer
    }
}


void SetDACvalue(int level) {
  int V_offset = 1000;
  // Function for setting the actual voltage levels on the outputs.
  Serial.print(level);
  Serial.print(" , ");
  dac.setPowerDown(0, 0, 3, 3);         //( 0 - On; 3 - 500k resistance to GND)

  dac.analogWrite(0,V_offset+level);      // Channel A set to Offset + level
  dac.analogWrite(1,V_offset);            // Channel B set to Offset
  
  //DACprintStatus();
}

void READ(){
    int32_t data;
    chan = 1; // Channel 2 (Current, R_shunt measurement)
    res = 2; // set resolution
    gain = 3; // [0,1,2,3] corresponds to [1x,2x,4x,8x] 
    mvDivisor = 1 << (gain + 2*res);
    adcConfig = 0;  // if adcConfig not reset, chan gets stuck at 3
    adcConfig |= chan << 5 | res << 2 | gain | MCP342X_START;
    mcp342xWrite(adcConfig);
    
    if (!mcp342xRead(data)) halt();     // Reads the data from channel 2 (Current, Drain-Iout)
    
    // Current in milli Amp
    double mv2 = (double)data/mvDivisor/4.46;
    Serial.print(mv2,4);               // measured over a "#define resistor_size" Ohm resistor -- ADDED 10* FOR 0.1 OHM
    Serial.println("E-3");
}

void measureinput() {
    //Serial.println("measuring");
    chan = 0; // Channel 1 (Voltage, Voltage difference between A and B )
    res = 1; // set resolution
    gain = 0; // [0,1,2,3] corresponds to [1x,2x,4x,8x] 
    mvDivisor = 1 << (gain + 2*res);
    adcConfig = 0;  // if adcConfig not reset, chan gets stuck at 3
    adcConfig |= chan << 5 | res << 2 | gain | MCP342X_START;
    mcp342xWrite(adcConfig);
    int32_t data;
    
    if (!mcp342xRead(data)) halt();     // Reads the data from channel 1 (Voltage, Drain-Source )
    
    // voltage in millivolts
    double mv1 = (double)data/mvDivisor*2; // Measured over a 1/2 voltage divider
    Serial.print(mv1,2);
    Serial.print(" mV, ");
        
    chan = 1; // Channel 2 (Current, R_shunt measurement)
    res = 2; // set resolution
    gain = 3; // [0,1,2,3] corresponds to [1x,2x,4x,8x] 
    mvDivisor = 1 << (gain + 2*res);
    adcConfig = 0;  // if adcConfig not reset, chan gets stuck at 3
    adcConfig |= chan << 5 | res << 2 | gain | MCP342X_START;
    mcp342xWrite(adcConfig);
    
    if (!mcp342xRead(data)) halt();     // Reads the data from channel 2 (Current, Drain-Iout)
    
    // Current in milli Amp
    double mv2 = (double)data/mvDivisor/4.46;
    Serial.print(mv2,4);               // measured over a "#define resistor_size" Ohm resistor -- ADDED 10* FOR 0.1 OHM
    Serial.println(" mA ");

    // DACprintStatus();
}


int ReadSCPIVOltage(){
  // Parameters for loading numbers from serial - SCPI combatible
        float Voltage = 0;
        int VoltageOrder = 0;
        int number;
        char indata[5];
        Voltage = Serial.parseFloat(); //Comes in EXPONENTIAL FORMAT [volts]..
        number = Serial.readBytesUntil('E',indata,5);
        VoltageOrder = Serial.parseInt(); // Parse exponential order..
        Serial.print(Voltage);
        Serial.print("E");
        Serial.println(VoltageOrder);
        return int(Voltage*1000*pow(10,VoltageOrder));
}

void resetDAC() {       //Reset DAC - Setting Vref and Gain is needed for correct conversion of bits to voltages.
  dac.analogWrite(0,0,0,0);
  dac.setVref(1,1,1,1);
  dac.setGain(1,1,1,1);
  dac.setPowerDown(3, 3, 3, 3); //( 0 - On; 3 - 500k resistance to GND)
}

//------------------------------------------------------------------------------
void halt(void){Serial.println(F("*ERROR* MCP3424 ADC"));}

//------------------------------------------------------------------------------

uint8_t mcp342xRead(int32_t &data)     //   read mcp342x data - No 18bit
{
   // pointer used to form int32 data
   uint8_t *p = (uint8_t *)&data;
   uint32_t start = millis();
      do {  // 12-bit to 16-bit mode
         Wire.requestFrom(MCP342X_ADDRESS, 3);
         if (Wire.available() != 3) {
            Serial.println(F("*ERROR* - MCP3424 ADC read failed"));
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
   Serial.println(F("*ERROR* - ADC read timeout"));      // dang it
   return false;
}

//------------------------------------------------------------------------------

uint8_t mcp342xWrite(uint8_t config)   //   write mcp342x configuration byte
{
   Wire.beginTransmission(MCP342X_ADDRESS);
   Wire.write(config);
   Wire.endTransmission();
}

void DACprintStatus()      // DAC status command for debug of DAC
{
  Serial.println("NAME     Vref  Gain  PowerDown  Value");
  for (int channel=0; channel <= 1; channel++)
  { 
    Serial.print("DAC");
    Serial.print(channel,DEC);
    Serial.print("   ");
    Serial.print("    "); 
    Serial.print(dac.getVref(channel),BIN);
    Serial.print("     ");
    Serial.print(dac.getGain(channel),BIN);
    Serial.print("       ");
    Serial.print(dac.getPowerDown(channel),BIN);
    Serial.print("       ");
    Serial.println(dac.getValue(channel),DEC);

//    Serial.print("EEPROM");
//    Serial.print(channel,DEC);
//    Serial.print("    "); 
//    Serial.print(dac.getVrefEp(channel),BIN);
//    Serial.print("     ");
//    Serial.print(dac.getGainEp(channel),BIN);
//    Serial.print("       ");
//    Serial.print(dac.getPowerDownEp(channel),BIN);
//    Serial.print("       ");
//    Serial.println(dac.getValueEp(channel),DEC);
  }
  Serial.println(" ");
}

void displaystatus(String line){
  // Display : ESP8266
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(40, 46, "Ben");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, line );
  display.display();
}

void displayinit(){
   // Display : ESP8266
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.clear();
}
