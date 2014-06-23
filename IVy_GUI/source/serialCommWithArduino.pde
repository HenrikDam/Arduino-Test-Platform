/*

IVy: created by Rupak Chakraborty and David Berney Needleman, MIT PV Lab.

This file accompanies IVy_GUI.pde and contains most of the logic for data handling and serial communication.

*/

import processing.serial.*;

Serial myPort; // The serial port
String portName; 
boolean serialPortCreated = false;
boolean firstContact = false;  // whether we've heard from the arduino
boolean readyForData = false;  // whether we are ready to receive data from arduino
boolean isReverseBias = false; // whether we should tell the arduino to sweep in reverse bias
float minV = -4.0; // minimum voltage that we can send to the arduino
float maxV = 4.0; // maximum voltage that we can send to the arduino
int minNumPoints = 0; // minimum number of data points that we should receive from the arduino
int maxNumPoints = 254; // maximum number of data points that we should receive from the arduino
String dataString = ""; // where all of the most recent data is stored in a string format
ArrayList<Float> dataV = new ArrayList(); // where the most recent voltage data is stored as floats
ArrayList<Float> dataI = new ArrayList(); // where the most recent current data is stored as floats
ArrayList<Float> dataVAll = new ArrayList(); // where all historical voltage data is stored as floats
ArrayList<Float> dataIAll = new ArrayList(); // where all historical current data is stored as floats
float dataVMin = -1; // minimum voltage value in data
float dataVMax = 1; // maximum voltage value in data
float dataIMax = 5; // maximum current value in data
float dataIMin = -5; // minimum current value in data
float maxAbsCurrent; // maximum magnitude of current
float maxAbsVoltage; // maximum magnitude of voltage

PrintWriter output; // for writing data file


/* This function is called whenever we receive a byte from the arduino */
void serialEvent(Serial myPort) {
  String inString = new String(myPort.readBytesUntil('\n'));
  // String inString = myPort.readStringUntil('\n'); // read in a string - the arduino will always send strings ending in a newline char
  // bug with the readStringUntil missing in Processing 2.1 
  inString = trim(inString); // get rid of whitespace
  println(inString); // debug

  if (firstContact == false) { // if this is the first time talking to the arduino
    if (inString.equals("CONNECT")) { // check if arduino is trying to connect
      myPort.write('C');  // tell arduino that we want to connect
      myPort.clear();  // clear the serial port buffer
      firstContact = true;  // we've had first contact with the arduino
      updateStatusBar("Connected!");
    }
  }
  else { // if this is not the first time talking to the arduino
    if (!readyForData) { // check if we are ready for data
      if (inString.equals("DATA")) { // check if the arduino is about to give us data
        readyForData = true; // we are ready for receiving data
        dataString = ""; // clear the data string
        resetData(); // clear all data arrays
        updateStatusBar("Running...");
      }
    }
    else { // if we have gotten to this point, then the arduino must be trying sending data to us
      if (inString != null) {  // check that we have good data
        inString = trim(inString);  // trim off any whitespace
        if (inString.equals("Current overload.")) {  // check if the arduino is telling us that the current has overloaded
          updateStatusBar("Current overload!");
          readyForData = false;
        }
        else if (inString.equals("Voltage sweep complete.")) { // check if the arduino is done sending us data
          updateStatusBar("Voltage sweep complete.");
          readyForData = false;
        }
        else {  // if we have gotten to this point, then the arduino is sending us pairs of comma delimited numbers as a string "voltage, current"
          dataString = dataString + inString + "\n" ; // append data to data string
          String[] dataStrings = split(inString, ','); // split into voltage and current strings
          float[] dataPoint = float(dataStrings); // convert strings to float
          
          // append floats to data arrays
          dataV.add(dataPoint[0]);
          dataI.add(dataPoint[1]);
          dataVAll.add(dataPoint[0]);
          dataIAll.add(dataPoint[1]);
          
          // update min and max voltage and current
          dataVMin = dataPoint[0] < dataVMin ? dataPoint[0] : dataVMin;
          dataVMax = dataPoint[0] > dataVMax ? dataPoint[0] : dataVMax;
          dataIMin = dataPoint[1] < dataIMin ? dataPoint[1] : dataIMin;
          dataIMax = dataPoint[1] > dataIMax ? dataPoint[1] : dataIMax;          
        }
      }
    }
  }
}

void createSerialPort() {
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n'); // set default to read until newline char
  serialPortCreated = true;
  firstContact = false;
}

void resetData(){
  dataV = new ArrayList();
  dataI = new ArrayList();
}

void resetGraphLimits(){
  dataVMin = -5;
  dataVMax = 5;
  dataIMax = 50;
  dataIMin = -50;
}

