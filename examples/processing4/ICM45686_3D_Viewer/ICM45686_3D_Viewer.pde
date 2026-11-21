import processing.serial.*;

// -1 selects the last serial port. If that is not the Pico, set the index
// shown in the Processing console after startup (0, 1, 2, ...).
final int PORT_INDEX = -1;
final int BAUD_RATE = 115200;
final int FILTER_BUTTON_X = 650;
final int FILTER_BUTTON_Y = 28;
final int FILTER_BUTTON_W = 220;
final int FILTER_BUTTON_H = 54;

Serial imuPort;
String portName = "not connected";

float rollDeg = 0;
float pitchDeg = 0;
float yawDeg = 0;
float zeroRollDeg = 0;
float zeroPitchDeg = 0;
float zeroYawDeg = 0;
boolean receivedOrientation = false;
boolean captureZeroOnNextPacket = true;
int filterMode = 1;  // 0=gyro only, 1=Madgwick, 2=Kalman roll/pitch fusion

void setup() {
  size(900, 650, P3D);
  smooth(8);
  textSize(18);

  String[] ports = Serial.list();
  println("Available serial ports:");
  printArray(ports);

  if (ports.length == 0) {
    println("No serial port found. Connect the Pico and restart this sketch.");
    return;
  }

  int selectedPort = PORT_INDEX;
  if (selectedPort < 0) {
    selectedPort = ports.length - 1;
  }
  if (selectedPort >= ports.length) {
    println("PORT_INDEX is outside the available port list.");
    return;
  }

  portName = ports[selectedPort];
  imuPort = new Serial(this, portName, BAUD_RATE);
  imuPort.clear();
  imuPort.bufferUntil('\n');
  println("Using: " + portName);
}

void draw() {
  background(20, 24, 32);

  lights();
  pushMatrix();
  translate(width * 0.5, height * 0.54, 0);

  float roll = wrappedDifference(rollDeg, zeroRollDeg);
  float pitch = wrappedDifference(pitchDeg, zeroPitchDeg);
  float yaw = wrappedDifference(yawDeg, zeroYawDeg);

  // Processing uses screen-oriented axes, so signs are adjusted for an
  // intuitive view. Change a sign here if your physical mounting is reversed.
  rotateZ(radians(-yaw));
  rotateX(radians(pitch));
  rotateY(radians(roll));

  drawAxes(180);

  noStroke();
  fill(70, 150, 235);
  box(260, 45, 150);

  // Red marker shows the +X/front side of the virtual object.
  pushMatrix();
  translate(142, 0, 0);
  fill(255, 75, 75);
  sphere(13);
  popMatrix();
  popMatrix();

  hint(DISABLE_DEPTH_TEST);
  camera();
  fill(235);
  text("ICM-45686 3D orientation", 24, 34);
  text("Port: " + portName, 24, 62);
  text("Roll: " + nf(rollDeg, 1, 1) + " deg", 24, 94);
  text("Pitch: " + nf(pitchDeg, 1, 1) + " deg", 24, 122);
  text("Yaw: " + nf(yawDeg, 1, 1) + " deg", 24, 150);
  text("Mode: " + filterModeName(), 24, 178);
  text("Press R to set zero. Press F to cycle the filter.", 24, height - 28);

  drawFilterToggle();

  if (!receivedOrientation) {
    fill(255, 190, 70);
    text("Waiting for ORI packets from Pico...", 24, 214);
  }
  hint(ENABLE_DEPTH_TEST);
}

void serialEvent(Serial source) {
  String line = source.readStringUntil('\n');
  if (line == null) {
    return;
  }

  String[] fields = split(trim(line), ',');
  if (fields.length != 5 || !fields[0].equals("ORI")) {
    return;
  }

  try {
    filterMode = constrain(Integer.parseInt(fields[1]), 0, 2);
    rollDeg = Float.parseFloat(fields[2]);
    pitchDeg = Float.parseFloat(fields[3]);
    yawDeg = Float.parseFloat(fields[4]);
  }
  catch (NumberFormatException error) {
    return;
  }

  receivedOrientation = true;
  if (captureZeroOnNextPacket) {
    zeroRollDeg = rollDeg;
    zeroPitchDeg = pitchDeg;
    zeroYawDeg = yawDeg;
    captureZeroOnNextPacket = false;
  }
}

void keyPressed() {
  if (key == 'r' || key == 'R') {
    captureZeroOnNextPacket = true;
  } else if (key == 'f' || key == 'F') {
    toggleFilter();
  }
}

void mousePressed() {
  if (mouseX >= FILTER_BUTTON_X && mouseX <= FILTER_BUTTON_X + FILTER_BUTTON_W &&
      mouseY >= FILTER_BUTTON_Y && mouseY <= FILTER_BUTTON_Y + FILTER_BUTTON_H) {
    toggleFilter();
  }
}

void toggleFilter() {
  if (imuPort == null) {
    return;
  }

  if (filterMode == 1) {
    filterMode = 2;
  } else if (filterMode == 2) {
    filterMode = 0;
  } else {
    filterMode = 1;
  }
  imuPort.write("FILTER," + filterMode + "\n");
  captureZeroOnNextPacket = true;
}

void drawFilterToggle() {
  noStroke();
  if (filterMode == 1) {
    fill(45, 170, 95);
  } else if (filterMode == 2) {
    fill(70, 115, 210);
  } else {
    fill(195, 75, 65);
  }
  rect(FILTER_BUTTON_X, FILTER_BUTTON_Y, FILTER_BUTTON_W, FILTER_BUTTON_H, 10);

  fill(255);
  textSize(16);
  if (filterMode == 1) {
    text("Madgwick - 6-axis", FILTER_BUTTON_X + 16, FILTER_BUTTON_Y + 34);
  } else if (filterMode == 2) {
    text("Kalman - Roll/Pitch", FILTER_BUTTON_X + 16, FILTER_BUTTON_Y + 34);
  } else {
    text("Gyro only - no fusion", FILTER_BUTTON_X + 16, FILTER_BUTTON_Y + 34);
  }
  textSize(18);
}

String filterModeName() {
  if (filterMode == 1) {
    return "Madgwick (accel + gyro)";
  }
  if (filterMode == 2) {
    return "Kalman (Roll/Pitch fusion; Yaw gyro only)";
  }
  return "Gyro only";
}

float wrappedDifference(float value, float zero) {
  float difference = value - zero;
  while (difference > 180) difference -= 360;
  while (difference < -180) difference += 360;
  return difference;
}

void drawAxes(float length) {
  strokeWeight(4);
  stroke(255, 80, 80);
  line(0, 0, 0, length, 0, 0);
  stroke(80, 255, 110);
  line(0, 0, 0, 0, length, 0);
  stroke(80, 150, 255);
  line(0, 0, 0, 0, 0, length);
}
