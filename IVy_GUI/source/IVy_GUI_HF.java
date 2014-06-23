import processing.core.*; 
import processing.xml.*; 

import processing.serial.*; 
import guicomponents.*; 
import processing.serial.*; 

import java.applet.*; 
import java.awt.Dimension; 
import java.awt.Frame; 
import java.awt.event.MouseEvent; 
import java.awt.event.KeyEvent; 
import java.awt.event.FocusEvent; 
import java.awt.Image; 
import java.io.*; 
import java.net.*; 
import java.text.*; 
import java.util.*; 
import java.util.zip.*; 
import java.util.regex.*; 

public class IVy_GUI_HF extends PApplet {

/*

IVy: created by Rupak Chakraborty and David Berney Needleman, MIT PV Lab.

This is a graphical user interface that communicates with a simple arduino-based IV tester designed for solar cells.  
The accompanying hardware schematics and arduino code are located at pv.mit.edu .

The GUI uses the guicomponents library for processing, written by Peter Lager.

NOTES:
- must have serialCommWithArduino.pde in the same folder.  This other file has most of the logic for data handling and serial communication.
- must have 'ArialMT-48.vlw' and 'TimesNewRomanPSMT-18.vlw' font files in 'data' folder of sketch
- must have the guicomponents library installed
-- to install, place the guicomponents folder (located in "libraries" folder that came with this source) in the libraries folder of Processing
-- guicomponents is also available for download at http://www.lagers.org.uk/g4p/distribution/web/index.html#download
*/




// G4P components for main window
GPanel pnlControls;
GLabel lblTitle, lblStatus, lblCredits, lblPickPort, lblSettings, lblStartV, lblStopV, lblNumPoints, lblVoltage, lblCurrent;
GLabel lblSave, lblFilename,lblConnection;
GTextField txfStartV, txfStopV, txfNumPoints, txfFilename;
GOptionGroup opgBias;
GCombo cboSerial;
GOption optForward;
GOption optReverse;
GActivityBar acyBar;
GButton btnConnect, btnStartSweep, btnSave, btnClearGraph;

// positions and sizes of components
int pX = 0;
int pY = 0;
int pWidth = 800;
int pHeight = 600;
int leftPanelWidth = 220;
int leftMargin = 10;
int pYConnection = 100;
int pYSettings = 250;
int pYSave = 440;
int filenameLabelWidth = 70;
int pYStatus = pHeight-30;
int pXGraph = leftPanelWidth;
int pYGraph = 65;
int pWidthGraph = pWidth - leftPanelWidth;
int pHeightGraph = pYStatus - pYGraph;
int graphMargin = 50;
int pPlotLeft = pXGraph + graphMargin;
int pPlotRight = pXGraph + pWidthGraph - graphMargin;
int pPlotTop = pYGraph + graphMargin;
int pPlotBottom = pYGraph + pHeightGraph - graphMargin;
int pPlotMiddle = (pPlotTop + pPlotBottom)/2; // vertical center of graph
int pPlotCenter = (pPlotLeft + pPlotRight)/2; // horizontal center of graph

PFont f;

public void setup(){
  // set size and font
  size(800,600);
  f = loadFont("TimesNewRomanPSMT-18.vlw");
  
  // initialize GUI
  G4P.setColorScheme(this, GCScheme.GREY_SCHEME);
  G4P.messagesEnabled(false);

  // create GUI
  createPanelAndLabels();
  createSerialListBox();
  createButtons();
  createTextFields();
  //lblCredits.setFocus(true);
  
  // redefine bottom and left to match pixel counts on either side
  pPlotBottom = pPlotMiddle + (pPlotMiddle - pPlotTop);
  pPlotLeft = pPlotCenter - (pPlotRight - pPlotCenter);
  
  // create graph area
  createGraphArea();
  
  // Enable mouse-over changes for buttons
  G4P.setMouseOverEnabled(true);
}

public void createGraphArea(){
  // draw background
  fill(255);
  noStroke();
  rect(pXGraph, pYGraph, pWidthGraph, pHeightGraph);
  // draw axes
  stroke(0);
  line(pPlotCenter, pPlotTop, pPlotCenter, pPlotBottom); // defines y axis
  line(pPlotLeft, pPlotMiddle, pPlotRight, pPlotMiddle); // defines x axis
  // draw labels
  textFont(f, 16);
  fill(0);
  text("V [V]", (pPlotRight + 10), pPlotMiddle + 5);
  text("I [mA]", pPlotCenter - 5, pPlotTop - 15);
}


public void handleOptionEvents(GOption selected, GOption deselected){
  // change the visible sign in front of the start and stop voltages according to the selected bias direction
  if(selected == optForward){
    isReverseBias = false;
    lblStartV.setText("Start voltage (V):  +");
    lblStopV.setText("Stop voltage (V):   +");
  }
}

public void createPanelAndLabels(){
  // initialize left panel controls
  pnlControls = new GPanel(this,"",pX,pY, leftPanelWidth,pHeight);
  pnlControls.setOpaque(true);
  pnlControls.setCollapsed(false);
  
  // initialize status bar label
  lblStatus = new GLabel(this, "Status", 0, pYStatus, pWidth, 30);
  lblStatus.setBorder(0);
  lblStatus.setOpaque(true);
  lblStatus.setColorScheme(GCScheme.GREEN_SCHEME);
  lblStatus.setFont("Arial", 15);
  
  // initialize title label
  lblTitle = new GLabel(this, "IVy", 0, 0, pWidth, 35);
  lblTitle.setBorder(0);
  lblTitle.setOpaque(true);
  lblTitle.setTextAlign(GAlign.CENTER);
  lblTitle.setTextAlign(GAlign.MIDDLE);
  lblTitle.setColorScheme(GCScheme.BLUE_SCHEME);
  lblTitle.setFont("Times New Roman", 22);

  // initialize credits label 
  lblCredits = new GLabel(this, "created by R. Chakraborty and D. B. Needleman, mod. by H. F. Dam", 0, 45, pWidth, 20);
  lblCredits.setBorder(0);
  lblCredits.setOpaque(true);
  lblCredits.setTextAlign(GAlign.CENTER);
  lblCredits.setTextAlign(GAlign.MIDDLE);
  lblCredits.setColorScheme(GCScheme.BLUE_SCHEME);
  lblCredits.setFont("Arial", 12);

  // initialize 'Connection' label   
  lblConnection = new GLabel(this, "Connection", leftMargin, pYConnection, 75, 12);
  lblConnection.setBorder(0);
  lblConnection.setOpaque(false);
  lblConnection.setFont("Arial", 18);
  
  // initialize 'Pick Port' label
  lblPickPort = new GLabel(this, "Pick COM Port", leftMargin, pYConnection + 40, 75, 12);
  lblPickPort.setBorder(0);
  lblPickPort.setOpaque(false);
  lblPickPort.setFont("Arial", 12);
  
  // initialize 'Settings' label
  lblSettings = new GLabel(this, "Settings", leftMargin, pYSettings, 75, 12);
  lblSettings.setBorder(0);
  lblSettings.setOpaque(false);
  lblSettings.setFont("Arial", 18);

  // initialize Start Voltage label  
  lblStartV = new GLabel(this, "Start voltage (V):  +", leftMargin, pYSettings + 40, 100, 12);
  lblStartV.setBorder(0);
  lblStartV.setOpaque(false);
  lblStartV.setFont("Arial", 12);

  // initialize Stop Voltage label  
  lblStopV = new GLabel(this, "Stop voltage (V):  +", leftMargin, pYSettings + 60, 100, 12);
  lblStopV.setBorder(0);
  lblStopV.setOpaque(false);
  lblStopV.setFont("Arial", 12);

  // initialize '# data points' label
  lblNumPoints = new GLabel(this, "# data points:", leftMargin, pYSettings + 80, 100, 12);
  lblNumPoints.setBorder(0);
  lblNumPoints.setOpaque(false);
  lblNumPoints.setFont("Arial", 12);

  // initialize 'Save data' label
  lblSave = new GLabel(this, "Save data", leftMargin, pYSave, 75, 12);
  lblSave.setBorder(0);
  lblSave.setOpaque(false);
  lblSave.setFont("Arial", 18);

  // initialize 'Filename' label  
  lblFilename = new GLabel(this, "Filename: ", leftMargin, pYSave + 40, filenameLabelWidth, 12);
  lblFilename.setBorder(0);
  lblFilename.setOpaque(false);
  lblFilename.setFont("Arial", 12);
  
  // create all labels
  pnlControls.add(lblTitle);
  pnlControls.add(lblStatus);
  pnlControls.add(lblCredits);
  pnlControls.add(lblConnection);
  pnlControls.add(lblPickPort);
  pnlControls.add(lblSettings);
  pnlControls.add(lblStartV);
  pnlControls.add(lblStopV);
  pnlControls.add(lblNumPoints);
  pnlControls.add(lblVoltage);
  pnlControls.add(lblSave);
}

public void createSerialListBox() {
  String[] serialPortList = Serial.list(); // find all serial ports
  try{
    cboSerial = new GCombo(this, serialPortList, 20, leftMargin + 3, pYConnection + 60, 80); // create serial list box
  } catch (NullPointerException e){
    serialPortList = new String[] {"No COMs"};
    cboSerial = new GCombo(this, serialPortList, 20, leftMargin + 3, pYConnection + 60, 80); // create serial list box
  }
  cboSerial.setSelected(1); // set default to first entry
  portName = cboSerial.selectedText(); // set global portName
  pnlControls.add(cboSerial); // add list box to left panel
}

public void createButtons(){
  // create 'Connect' button
  btnConnect = new GButton(this, "Connect", 117, pYConnection + 40, 75, 40);	
  pnlControls.add(btnConnect);
  
  // create 'Start Sweep' button
  btnStartSweep = new GButton(this, "Start Sweep", leftMargin, pYSettings + 130, 147, 30);
  pnlControls.add(btnStartSweep);
  
  // create 'Save data' button
  btnSave = new GButton(this, "Save data", leftMargin, pYSave + 70, 100, 30);
  pnlControls.add(btnSave);
  
  // create 'Clear Graph' button
  btnClearGraph = new GButton(this, "Clear Graph", leftMargin + 120, pYSave + 70, 75, 30);
  pnlControls.add(btnClearGraph);
}

public void createTextFields(){
  // create the start voltage text field
  txfStartV = new GTextField(this, "0.0", 115, pYSettings + 40, 50, 15);
  txfStartV.setFont("Arial", 12);
  pnlControls.add(txfStartV);
  
  // create the stop voltage text field
  txfStopV = new GTextField(this, "0.7", 115, pYSettings + 60, 50, 15);
  txfStopV.setFont("Arial", 12);
  pnlControls.add(txfStopV);
  
  // create the # data points text field
  txfNumPoints = new GTextField(this, "100", 115, pYSettings + 80, 50, 15);
  txfNumPoints.setFont("Arial", 12);
  pnlControls.add(txfNumPoints);
  
  // create the filename text field
  txfFilename = new GTextField(this, "data.txt", filenameLabelWidth + 5, pYSave + 40, 120, 15);
  txfFilename.setFont("Arial", 12);
  pnlControls.add(txfFilename);
}

// event handler for the serial list box
public void handleComboEvents(GCombo combo){
  if(cboSerial == combo){
      cboSerial.setFocus(true);
      portName = cboSerial.selectedText();
  }
}


// event handler for buttons
public void handleButtonEvents(GButton button){
  // handle Connect button
  if(btnConnect == button && button.eventType == GButton.CLICKED){
    if (serialPortCreated == false){ // if this is the first time connecting
      updateStatusBar("Connecting..."); // display connecting status
      createSerialPort(); // make a serial connection
      myPort.write('C'); // attempt to connect with arduino by writing a 'C' to the comm port
    }
    else { // if trying to reconnect
      firstContact = false;
      updateStatusBar("Reconnecting..."); // display status
      myPort.write('C'); // attempt to connect with arduino by writing a 'C' to the comm port
    }
  }
  
  // handle Sweep button
  if(btnStartSweep == button && button.eventType == GButton.CLICKED){
    float startV = PApplet.parseFloat(txfStartV.viewText()); // get start voltage from text field
    float stopV = PApplet.parseFloat(txfStopV.viewText()); // get stop voltage from text field
    int numPoints = round(PApplet.parseFloat(txfNumPoints.viewText())); // get number of data points from text field
    if (serialPortCreated == true) { // if already connected
      if (str(startV).equals("NaN") || str(stopV).equals("NaN") || str(numPoints).equals("NaN")){ // error check for numbers
        updateStatusBar("ERROR! Enter numbers under Settings.");
      }
      else{
        myPort.write("S "); // to tell arduino that settings are coming next
        myPort.write(str(startV)+' '); // to tell arduino the start voltage of the sweep 
        myPort.write(str(stopV)+' '); // to tell arduino the stop voltage of the sweep
        myPort.write(str(numPoints)); // to tell arduino the number of data points in the sweep
      }
    }
    else{
      updateStatusBar("ERROR! Connect to device first.");
    }
  }
  
  // handle Save button
  if(btnSave == button && button.eventType == GButton.CLICKED){
    if(dataString.equals("")){ // check that data was taken
      updateStatusBar("No data to save.");
    }
    else if (txfFilename.viewText().equals("")){ // check that filename is given
      updateStatusBar("Please enter a filename.");
    }
    else{
      output = createWriter(txfFilename.viewText()); // create output stream
      output.print(dataString); // print data
      output.flush(); // flush output stream
      output.close(); // close file
      
      updateStatusBar("File Saved!");
    }
  }
  
  // handle Clear Graph button
  if(btnClearGraph == button && button.eventType == GButton.CLICKED){
    dataVAll = new ArrayList(); // re-initialize array of voltage data located in other pde file
    dataIAll = new ArrayList(); // re-initialize array of current data located in other pde file
    resetGraphLimits(); // this function located in other pde
    updateGraph(); // re-draw graph
  }
}

public void draw(){
  updateGraph(); // re-draw graph
}

public void updateStatusBar(String s){
  lblStatus.setText("Status: " + s);
}

public void updateGraph(){
  // re-create graph area, drawing over previous data
  createGraphArea();
  textFont(f, 12);
  fill(0);
  
  // define limits of data
  maxAbsCurrent = max(abs(dataIMax), abs(dataIMin));
  maxAbsVoltage = max(abs(dataVMax), abs(dataVMin));
  
  // find nice round numbers for limits of graph
  float maxGraphCurrent = 10.0f * ceil(maxAbsCurrent / 10.0f);
  float maxGraphVoltage = ceil(maxAbsVoltage * 10.0f) / 10.0f;
  
  // figure out tick spacing
  int tickX;
  int tickY;
  float tickXSpacing = (pPlotRight - pPlotCenter)/5.0f;
  float tickYSpacing = (pPlotMiddle - pPlotTop)/5.0f;
  float currentSpacing = maxGraphCurrent/5.0f;
  float voltageSpacing = maxGraphVoltage/5.0f;
  
  // draw tick marks
  for (int i = -5; i <= 5; i++){
    tickX = round(pPlotCenter + tickXSpacing * i);
    tickY = round(pPlotMiddle - tickYSpacing * i);
    line(tickX, pPlotMiddle - 5, tickX, pPlotMiddle + 5);
    line(pPlotCenter - 5, tickY, pPlotCenter + 5, tickY);
    
    // draw axes labels
    if (i != 0){
      textAlign(LEFT);
      text(nf(i*voltageSpacing, 1, 1), tickX - 5, pPlotMiddle + 20);
      text(nf(i*currentSpacing, 1, 0), pPlotCenter +10, tickY + 5);
    }
  }
  
  // draw data points
  fill(0, 30, 255);
  noStroke();
  for (int i = 0; i < dataVAll.size(); i++){
    float voltage = (float)dataVAll.get(i);
    float current = (float)dataIAll.get(i);
    
    float dataPointX = map(voltage, (-1)*maxAbsVoltage, maxAbsVoltage, pPlotLeft, pPlotRight);
    float dataPointY = map(current, (-1)*maxAbsCurrent, maxAbsCurrent, pPlotBottom, pPlotTop);
    
    ellipse(dataPointX, dataPointY, 5, 5);
  }
  
}

/*

IVy: created by Rupak Chakraborty and David Berney Needleman, MIT PV Lab.

This file accompanies IVy_GUI.pde and contains most of the logic for data handling and serial communication.

*/



Serial myPort; // The serial port
String portName; 
boolean serialPortCreated = false;
boolean firstContact = false;  // whether we've heard from the arduino
boolean readyForData = false;  // whether we are ready to receive data from arduino
boolean isReverseBias = false; // whether we should tell the arduino to sweep in reverse bias
float minV = -4.0f; // minimum voltage that we can send to the arduino
float maxV = 4.0f; // maximum voltage that we can send to the arduino
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
public void serialEvent(Serial myPort) {
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
          float[] dataPoint = PApplet.parseFloat(dataStrings); // convert strings to float
          
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

public void createSerialPort() {
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n'); // set default to read until newline char
  serialPortCreated = true;
  firstContact = false;
}

public void resetData(){
  dataV = new ArrayList();
  dataI = new ArrayList();
}

public void resetGraphLimits(){
  dataVMin = -5;
  dataVMax = 5;
  dataIMax = 50;
  dataIMin = -50;
}

  static public void main(String args[]) {
    PApplet.main(new String[] { "--bgcolor=#F0F0F0", "IVy_GUI_HF" });
  }
}
