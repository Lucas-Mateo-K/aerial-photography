import processing.serial.*;
Serial myPort;  // The serial port
String val;     // Data received from the serial port
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

PShape model;
PGraphics topView, frontView, sideView;
float scale = 1;

float translationtionX;
float translationtionY;
float translationtionZ;
float rotationX = 0;
float rotationY = 0;
float rotationZ = 0;

float rotationXtranslated = 0;
float rotationYtranslated = 0;
float rotationZtranslated = 0;

public void setup(){
  String portName = Serial.list()[0]; // change this to select your port
  myPort = new Serial(this, portName, 115200);
  size(2000, 1200, P3D);
  model = loadShape("motor frame v42.obj");
  topView = createGraphics(width/3, height, P3D);
  frontView = createGraphics(width/3, height, P3D);
  sideView = createGraphics(width/3, height, P3D);
}

public void draw(){
  
  while (myPort.available() > 0) {
    val = myPort.readStringUntil('\n');
    if (val != null){
      println(val);
      val = trim(val);
      if (val.startsWith("roll:")) {
        roll = float(val.substring(5));
      } else if (val.startsWith("pitch:")) {
        pitch = float(val.substring(6));
      } else if (val.startsWith("yaw:")) {
        yaw = float(val.substring(4));
      }
    }
  }
  
  background(0);
  
  float rotationY = radians(roll);
  float rotationX = radians(pitch);
  float rotationZ = -(radians(yaw));
  
  rotationXtranslated = (rotationY * sin(-rotationZ)) + (rotationX * cos(rotationZ));
  rotationYtranslated = (rotationY * cos(rotationZ)) + (rotationX * sin(rotationZ));
  rotationZtranslated = rotationZ; 
  
  // Top view
  topView.beginDraw();
  topView.background(0);
  topView.lights();
  topView.translate(topView.width/2, topView.height/2);
  topView.rotateX(rotationXtranslated);
  topView.rotateY(rotationYtranslated);
  topView.rotateZ(rotationZtranslated);
  topView.scale(scale);
  topView.shape(model);
  topView.endDraw();

  // Front view
  frontView.beginDraw();
  frontView.background(0);
  frontView.lights();
  frontView.translate(frontView.width/2, frontView.height/2);
  frontView.rotateX(PI/2 + rotationXtranslated);
  frontView.rotateY(rotationYtranslated);
  frontView.rotateZ(rotationZtranslated);
  frontView.scale(scale);
  frontView.shape(model);
  frontView.endDraw();

  // Side view
  /*
  sideView.beginDraw();
  sideView.background(0);
  sideView.lights();
  sideView.translate(sideView.width/2, sideView.height/2);
  sideView.rotateX(PI/2 + rotationXtranslated);
  sideView.rotateY(rotationYtranslated);
  sideView.rotateZ(-PI/2 + rotationZtranslated);
  sideView.scale(scale);
  sideView.shape(model);
  sideView.endDraw();
  */

  // Display views
  image(topView, 0, 0);
  image(frontView, width/3, 0);
  //image(sideView, 2*width/3, 0);
  
}
