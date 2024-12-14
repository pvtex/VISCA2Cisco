#include "visca_controller_converter.h"

SoftwareSerial viscaOutput(VISCARX, VISCATX);
SoftwareSerial controllerInput(CONRX, CONTX);

String vers = "1.0";

bool debug = true;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("+--------------------------------+"));
  Serial.print  (F("| VISCA Converter for CISCO v"));
  Serial.print(String(vers));
  Serial.println(F(" |"));
  Serial.println(F("|      (c)2024 by  VID-PRO       |"));
  Serial.println(F("+--------------------------------+"));
  Serial.println(F(""));
  Serial.println(F("+--------------------+"));
  Serial.println(F("|     Starting up    |"));
  Serial.println(F("+--------------------+"));

  Serial.println(F("| Setting up Ports:"));
  Serial.println(F("|   VISCA"));
  viscaOutput.begin(9600);
  Serial.println(F("|   Controller"));
  controllerInput.begin(96000);

  Serial.println(F("| Init CAMs"));
  initCameras();

  Serial.println(F("|"));
  Serial.println(F("| Starting done"));
  Serial.println(F("+---------------------"));
  Serial.println(F(""));

  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  receiveViscaData();
  handleControllerSerial();
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void handleControllerSerial() {
  String conin = controllerInput.readStringUntil('EOF');
  String temp = conin.substring(13);

  if (conin.length() > 0) {
    if (debug == true) {
      Serial.print(F("CON-IN: '"));
      Serial.print(conin);
      Serial.println(F("'"));
      Serial.print(F("CON-IN.SUB: '"));
      Serial.print(temp);
      Serial.println(F("'"));
    }

    if (temp == "J-Z-WIDE:EOF") {
      if (debug == true) {
        Serial.println(F("CON-IN: ZOOM WIDE"));
      }
      sendZoomPacket(0x30, 10); //Wide
    }
    else if (temp == "J-Z-TELE:EOF") {
      if (debug == true) {
        Serial.println(F("CON-IN: ZOOM TELE"));
      }
      sendZoomPacket(0x20, 10); //Tele
    }
    else if (temp == "J-Z-STOP:EOF") {
      if (debug == true) {
        Serial.println(F("CON-IN: ZOOM STOP"));
      }
      sendViscaPacket(zoomStop, sizeof(zoomStop));
    }
    else if (temp == "J-ST-H00-V00:EOF") {
      if (debug == true) {
        Serial.println(F("CON-IN: PAN/TILT STOP"));
      }
      sendViscaPacket(panStop, sizeof(panStop));
    }
    else if ( temp.substring(0, 4) == "J-UU" || temp.substring(0, 4) == "J-DD" || 
              temp.substring(0, 4) == "J-RR" || temp.substring(0, 4) == "J-LL" ||
              temp.substring(0, 4) == "J-RU" || temp.substring(0, 4) == "J-RD" ||
              temp.substring(0, 4) == "J-LU" || temp.substring(0, 4) == "J-LD") {
      int pan;
      int tilt;
      String value = temp.substring(5);
      String panstr = getValue(value, '-', 0);
      String tiltstr = getValue(value, '-', 1);

      if (temp.substring(2, 2) == "DD" || temp.substring(2, 2) == "LD" || temp.substring(2, 2) == "RD") {
        pan = (panstr.substring(1, 2).toInt()) * (-1);
      } else {
        pan = panstr.substring(1, 2).toInt();
      }
      if (temp.substring(2, 2) == "LL" || temp.substring(2, 2) == "LU" || temp.substring(2, 2) == "LD") {
        tilt = (tiltstr.substring(1, 2).toInt()) * (-1);
      } else {
        tilt = tiltstr.substring(1, 2).toInt();
      }

      if (debug == true) {
        Serial.print(F("CON-IN: PAN/TILT: '"));
        Serial.print(pan);
        Serial.print(F("','"));
        Serial.print(tilt);
        Serial.println(F("'"));
      }

      uint8_t panSpeed;
      if (pan < 0) {
        // Left
        panSpeed = map(pan, -31, 0, 0, ptMaxSpeed);
        panTilt[6] = 0x01;
      } else {
        panSpeed = map(pan, 0, 31, 0, ptMaxSpeed);
        // Right
        panTilt[6] = 0x02;
      }

      uint8_t tiltSpeed;
      if (tilt < 0) {
        // Down
        tiltSpeed = map(tilt, -31, 0, 0, ptMaxSpeed);
        panTilt[7] = 0x02;
      } else {
        // Up
        tiltSpeed = map(tilt, 0, 31, 0, ptMaxSpeed);
        panTilt[7] = 0x01;
      }

      if (debug == true) {
        Serial.print(F("CON-IN PAN/TILT MAP: '"));
        Serial.print(panSpeed);
        Serial.print(F("','"));
        Serial.print(tiltSpeed);
        Serial.println(F("'"));
      }
      sendViscaPacket(panTilt, sizeof(panTilt));
    }
  }
}

void receiveViscaData() {
  static byte ndx = 0;
  while (viscaOutput.available() > 0) {
    byte rc = viscaOutput.read();

    if (rc != 0xFF) {
      viscaMessage[ndx] = rc;
      ndx++;
      if (ndx >= maxViscaMessageSize) {
        ndx = maxViscaMessageSize - 1;
      }
    } else {
      if (viscaMessage[0] == 0x90) {
        if (DEBUG_VISCA == 1) {
          if (viscaMessage[1] == 0x50) {
            Serial.println("Command: OK");
          }
        }

        if (viscaMessage[1] == 0x60) {
          switch (viscaMessage[2]) {
            case 0x01:
              Serial.println("Error: Message length error");
            case 0x02:
              Serial.println("Error: Syntax error");
            case 0x03:
              Serial.println("Error: Command buffer full");
            case 0x04:
              Serial.println("Error: Command cancelled");
            case 0x05:
              Serial.println("Error: No socket (to be cancelled)");
            case 0x41:
              Serial.println("Error: Command not executable");
            default:
              Serial.print("Unknown Error: ");
              for (uint8_t i = 0; i < ndx; i++) {
                Serial.print("0x");
                Serial.print(viscaMessage[i], HEX);
                Serial.print(" ");
              }
              Serial.println("0xFF");
          }
        }
      }
      ndx = 0;
    }
  }
}

void sendZoomPacket(byte zoomDir, int zoomSpeed) {
  uint8_t zoomDirSpeed = (uint8_t) zoomDir + zoomSpeed;
  zoomCommand[4] = zoomDirSpeed;
  sendViscaPacket(zoomCommand, sizeof(zoomCommand));
}

void toggleFocusControl() {
  sendViscaPacket(focusModeInq, sizeof(focusModeInq));
  delay(100);
  receiveViscaData();
  Serial.print("Current Focus Status: ");
  if (viscaMessage[2] == 2) {
    Serial.println("Auto, Toggling to manual");
    sendViscaPacket(focusManual, sizeof(focusManual));
  } else {
    Serial.println("Manual, Toggling to auto");
    sendViscaPacket(focusAuto, sizeof(focusAuto));
  }
}


void sendViscaPacket(byte *packet, int byteSize) {
  if (led) {
    digitalWrite(LED_BUILTIN, LOW);
    led = false;
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    true;
  }
  if (DEBUG_VISCA == 1) {
    Serial.print("Sending:");
  }
  for (int i = 0; i < byteSize; i++) {
    if (DEBUG_VISCA == 1) {
      Serial.print(" 0x");
      Serial.print(packet[i], HEX);
    }

    viscaOutput.write(packet[i]);
  }
  if (DEBUG_VISCA == 1) {
    Serial.println();
  }
}

void initCameras() {
  //Send Address command
  Serial.println("| Setting camera addresses...");
  sendViscaPacket(addressCommand, sizeof(addressCommand));
  delay(delayTime);  //delay to allow camera time for next command
  receiveViscaData();

  // Turn off IR control
  Serial.println("| Disabling IR control...");
  sendViscaPacket(irOff, sizeof(irOff));
  delay(delayTime);  //delay to allow camera time for next command
  receiveViscaData();

  // Set camera resolution.  Needed for camera w/o DIP switch control of video format.
  Serial.println("| Setting camera resolution...");
  sendViscaPacket(videoFormat, sizeof(videoFormat));
  delay(delayTime);  //delay to allow camera time for next command
  receiveViscaData();

  //Send IF_clear command
  Serial.println("| Sending IF_Clear...");
  sendViscaPacket(ifClear, sizeof(ifClear));
  delay(delayTime);  //delay to allow camera time for next command
  receiveViscaData();
}
