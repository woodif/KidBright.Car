// Chiang Mai Maker Club Drone Simulator for CMMC Remote
// CMMCDroneSimulator.ino
//
// Created by Pheeraphat Sawangphian on Thu Aug 11 2016.
//
// Remote Protocol
// 0x01 = idle (Power Off)
// 0xf0 = send Tuning Data to drone (KP, KI, KD)
// 0xfc = receive Tuning Data from drone (KP, KI, KD)
// 0xfe = send Trim or Control Data to drone (yaw, pitch, roll, throttle)
// 0xf1-0xf9 = send Function Data (F1-F9) to drone
// 0xf9 = receive Function Data (Shake) from drone
//
// Drone Protocol
// 0xf0 = receive Tuning Data from remote (KP, KI, KD)
// 0xfC = send Tuning Data to remote (KP, KI, KD)
// 0xfe = receive Trim or Control Data from remote (yaw, pitch, roll, throttle)
// 0xf1-0xf9 = receive Function Data (F1-F9) from remote
// 0xf9 = send Function Data (Shake) to remote

#define Serial_rec_Debug 0

#include <WiFi.h>
#include <WiFiUDP.h>
#include <EEPROM.h>
#include <Ticker.h>
//#include <SSD1306.h>

//#ifdef ARDUINO_ESP8266_ESPRESSO_LITE_V1
//#define LED 16
//#else
#define LED 0
//#endif

Ticker ticker;

#define DEFAULT_SSID_LENGTH 16

typedef struct {
  int8_t startByte;
  int16_t parameter;
  int16_t checksum;
  char ssid[DEFAULT_SSID_LENGTH];
} FunctionData;

IPAddress remoteIP;
unsigned int remotePort = -1;

typedef struct {
  int8_t startByte;
  int8_t roll;
  int8_t pitch;
  int8_t throttle;
  int8_t yaw;
  int8_t checksum;
  char ssid[DEFAULT_SSID_LENGTH];
} ControlData;

typedef struct {
  int8_t startByte;
  int8_t startByte2;
  int8_t yawPitchRoll;
  int16_t kp;
  int16_t ki;
  int16_t kd;
  int16_t checksum;
  char ssid[DEFAULT_SSID_LENGTH];
} TuningData;

WiFiUDP udp;
byte data[512] = {0};
unsigned int localPort = 12345;
TuningData tuningData[3] = {0};
String output = "";
String lastOutput = "";
String yawPitchRollText[3] = {"Yaw..:", "Pitch:", "Roll.:"};
char accessPointName[DEFAULT_SSID_LENGTH] = {'\0'};
char defaultESPWiFiName[DEFAULT_SSID_LENGTH] = {'\0'};
char accessPointPassword[DEFAULT_SSID_LENGTH] = {'\0'};
bool isPowerOn = false;

//SSD1306* display;
static const int numberOfLines = 5;
String line[numberOfLines] = {""};
int currentLine = 0;
bool updateOLED = true;

void addLine(String string);
String floatToString(float value, int length, int decimalPalces);
String hexToString(byte value);
String intToString(int value, int length);
String ipToString(IPAddress ip);
bool isSSID(char* ssid);
void loadTuningData();
String readEEPROM(int index, int length);
void saveTuningData(int i);
int writeEEPROM(int index, String text);
ControlData TFData = {0};
int8_t WatchDogCheck = 0;
void shakeRemote(int16_t milliseconds = 500); // the minimum duration on iOS devices is 400ms + 100

void blink();

void receriver_init() {
  delay(3000);
  Serial.begin(115200);
  EEPROM.begin(512);
  loadTuningData();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  byte mac[6] = {0};
  WiFi.macAddress(mac);

  sprintf(accessPointName, "KidBright%lu", 101);
  sprintf(accessPointPassword, "%lu", 12345678);

  if (accessPointName[13] == '\0')
  {
    accessPointName[13] =  '0';
    accessPointName[14] =  '0';
  }
  if (accessPointName[14] == '\0')
  {
    accessPointName[14] =  '0';
  }

  accessPointName[15] =  {'\0'};

  if (accessPointPassword[6] == '\0')
  {
    accessPointPassword[6] =  '0';
    accessPointPassword[7] =  '0';
  }

  if (accessPointPassword[7] == '\0')
  {
    accessPointPassword[7] =  '0';
  }
  accessPointPassword[8] = {'\0'};

  WiFi.softAP(accessPointName, accessPointPassword);
  WiFi.mode(WIFI_AP_STA);
  udp.begin(localPort);

  Serial.println();
  Serial.println(String("Name: ") + accessPointName);
  Serial.println(String("Pass: ") + accessPointPassword);
  Serial.println(String("Port: ") + localPort);
  Serial.println();

  //  display = new SSD1306(0x3c, 4, 5);
  //
  //  if (display) {
  //    display->init();
  //    display->flipScreenVertically();
  //  }

  pinMode(LED, OUTPUT);
  ticker.attach_ms(1000, blink);
}

void receriver_loop() {

  //  if (display && updateOLED) {
  //    updateOLED = false;
  //    display->clear();
  //    display->drawString(0, 0, "Name");
  //    display->drawString(27, 0, String(": ") + accessPointName);
  //    display->drawString(0, 8, "Port..");
  //    display->drawString(27, 8, String(": ") + localPort);
  //
  //    for (int i = 0; i < numberOfLines; i++) {
  //      display->drawString(0, 16 + (i * 8), line[i]);
  //    }
  //
  //    display->display();
  //  }

  int numberOfBytes = udp.parsePacket();

  if (numberOfBytes > 0) {
    udp.read(data, numberOfBytes);
    remoteIP = udp.remoteIP();
    remotePort = udp.remotePort();

    if (data[0] >= 0xf1 && data[0] <= 0xf9) { // functions
      FunctionData functionData = {0};
      memcpy(&functionData, data, sizeof(FunctionData));
      int16_t checksum = functionData.parameter;
      functionData.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';

      //      if (functionData.checksum == checksum && isSSID(functionData.ssid)) {
      //        shakeRemote((data[0] == 0xf2) ? 1000 : 40); // test shake
      //        String string = "Function " + hexToString(functionData.startByte);
      //        if(Serial_rec_Debug) Serial.println(string);
      //        addLine(string);
      //      }

    } else if (data[0] == 0xf0 && data[1] == 0xf0) { // tuning data
      TuningData tuningDataBuffer = {0};
      memcpy(&tuningDataBuffer, data, sizeof(TuningData));
      int16_t checksum = tuningDataBuffer.yawPitchRoll + tuningDataBuffer.kp + tuningDataBuffer.ki + tuningDataBuffer.kd;
      tuningDataBuffer.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';

      if (tuningDataBuffer.yawPitchRoll >= 0x01 && tuningDataBuffer.yawPitchRoll <= 0x03 && tuningDataBuffer.checksum == checksum && isSSID(tuningDataBuffer.ssid)) {
        int i = tuningDataBuffer.yawPitchRoll - 1;
        tuningData[i] = tuningDataBuffer;
        saveTuningData(i);

        float kp = tuningDataBuffer.kp / 10.0f;
        float ki = tuningDataBuffer.ki / 10.0f;
        float kd = tuningDataBuffer.kd / 10.0f;

        String string = "";

        if (i == 0) {
          string += String("Receive tuning data from ") + ipToString(udp.remoteIP()) + " port " + udp.remotePort() + ":";
          if (Serial_rec_Debug) Serial.println(string);
          addLine("Receive tuning data from");
          addLine(ipToString(udp.remoteIP()) + " Port " + udp.remotePort());
        }

        String kpString = floatToString(kp, 5, 1);
        String kiString = floatToString(ki, 5, 1);
        String kdString = floatToString(kd, 5, 1);
        string = String(yawPitchRollText[i]) + " KP " + kpString + ", KI " + kiString + ", KD " + kdString;
        if (Serial_rec_Debug) Serial.println(string);
        addLine(String(yawPitchRollText[i]) + kpString + "," + kiString + "," + kdString);
      }
    } else if (data[0] == 0xfc && data[1] == 0xfc) { // get tuning data
      TuningData tuningDataBuffer = {0};
      memcpy(&tuningDataBuffer, data, sizeof(TuningData));
      int16_t checksum = tuningDataBuffer.yawPitchRoll + tuningDataBuffer.kp + tuningDataBuffer.ki + tuningDataBuffer.kd;
      tuningDataBuffer.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';

      if (strlen(tuningDataBuffer.ssid) == 0) {
        memcpy(tuningDataBuffer.ssid, accessPointName, DEFAULT_SSID_LENGTH);
      }

      if (tuningDataBuffer.checksum == checksum && isSSID(tuningDataBuffer.ssid)) {
        // send tuning data to remote control
        for (tuningDataBuffer.yawPitchRoll = 0x01; tuningDataBuffer.yawPitchRoll <= 0x03; tuningDataBuffer.yawPitchRoll++) {
          int i = tuningDataBuffer.yawPitchRoll - 1;
          tuningDataBuffer.kp = tuningData[i].kp;
          tuningDataBuffer.ki = tuningData[i].ki;
          tuningDataBuffer.kd = tuningData[i].kd;
          tuningDataBuffer.checksum = tuningDataBuffer.yawPitchRoll + tuningDataBuffer.kp + tuningDataBuffer.ki + tuningDataBuffer.kd;
          memcpy(&data, &tuningDataBuffer, sizeof(TuningData));

          String string = "";

          if (i == 0) {
            string += String("Send tuning data to ") + ipToString(udp.remoteIP()) + " port " + udp.remotePort() + ":";
            if (Serial_rec_Debug) Serial.println(string);
            addLine("Send tuning data to");
            addLine(ipToString(udp.remoteIP()) + " Port " + udp.remotePort());
          }

          string = hexToString(tuningDataBuffer.startByte) + ", ";
          string += hexToString(tuningDataBuffer.startByte2) + ", ";
          string += hexToString(tuningDataBuffer.yawPitchRoll) + ", ";
          string += intToString(tuningDataBuffer.kp, 4) + ", " + intToString(tuningDataBuffer.ki, 4) + ", " + intToString(tuningDataBuffer.kd, 4) + ", " + intToString(tuningDataBuffer.checksum, 4);
          if (Serial_rec_Debug) Serial.println(string);

          String kpString = floatToString(tuningDataBuffer.kp / 10.0f, 5, 1);
          String kiString = floatToString(tuningDataBuffer.ki / 10.0f, 5, 1);
          String kdString = floatToString(tuningDataBuffer.kd / 10.0f, 5, 1);
          addLine(String(yawPitchRollText[i]) + kpString + "," + kiString + "," + kdString);

          udp.beginPacket(udp.remoteIP(), udp.remotePort());
          udp.write(data, sizeof(TuningData));
          udp.endPacket();
          delay(200); // don't forget to delay
        }
      }
    } else if (data[0] == 0xfe || data[0] == 0x01) { // control or idle
      ControlData controlData = {0};
      memcpy(&controlData, data, sizeof(ControlData));
      int8_t checksum = controlData.roll + controlData.pitch + controlData.throttle + controlData.yaw;
      controlData.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';

      if (controlData.checksum == checksum && isSSID(controlData.ssid)) {
        int8_t trimCommand = 0xff;
        int8_t controlCommand = 0xfe;

        if (controlData.roll == trimCommand && controlData.pitch == trimCommand && controlData.throttle == trimCommand && controlData.yaw == trimCommand) {
          if (Serial_rec_Debug) Serial.println("Trim");
          addLine("Trim");
          //          shakeRemote(200); // test shake
        } else {
          if (controlData.startByte == controlCommand) {
            blink();

            if (!isPowerOn) {
              isPowerOn = true;
              if (Serial_rec_Debug) Serial.println("Power On");
              addLine("Power On");
            }
          } else if (isPowerOn) {
            isPowerOn = false;
            if (Serial_rec_Debug) Serial.println("Power Off");
            addLine("Power Off");
          }
          blink();
          WatchDogCheck = 50;// reset watchdog
          memcpy(&TFData, data, sizeof(ControlData));

          String rollString = intToString(controlData.roll, 4);
          String pitchString = intToString(controlData.pitch, 4);
          String throttleString = intToString(controlData.throttle, 3);
          String yawString = intToString(controlData.yaw, 4);
          output = String("Roll ") + rollString + ", Pitch " + pitchString + ", Throttle " + throttleString + ", Yaw " + yawString;

          if (output != lastOutput) {
            lastOutput = output;
            if (Serial_rec_Debug) Serial.println(output);
            addLine(String("|") + rollString + "|" + pitchString + "|" + throttleString + "|" + yawString + "|");
          }
        }
      }
    }
  }
}

void addLine(String string) {
  if (currentLine == numberOfLines - 1 && line[currentLine].length() > 0) {
    for (int i = 0; i < numberOfLines - 1; i++) {
      line[i] = line[i + 1];
    }
  }

  line[currentLine] = string;

  if (++currentLine >= numberOfLines) {
    currentLine = numberOfLines - 1;
  }

  updateOLED = true;
}

String floatToString(float value, int length, int decimalPalces) {
  String stringValue = String(value, decimalPalces);
  String prefix = "";

  for (int i = 0; i < length - stringValue.length(); i++) {
    prefix += " ";
  }

  return prefix + stringValue;
}

String hexToString(byte value) {
  int length = 2;
  String stringValue = String(value, HEX);
  String prefix = "";

  for (int i = 0; i < length - stringValue.length(); i++) {
    prefix += "0";
  }

  return "0x" + prefix + stringValue;
}

String intToString(int value, int length) {
  String stringValue = String(value);
  String prefix = "";

  for (int i = 0; i < length - stringValue.length(); i++) {
    prefix += " ";
  }

  return prefix + stringValue;
}

String ipToString(IPAddress ip) {
  return String(ip[0]) + "." + ip[1] + "." + ip[2] + "." + ip[3];
}

bool isSSID(char* ssid) {
  return (strcmp(ssid, accessPointName) == 0 || strcmp(ssid, defaultESPWiFiName) == 0 || strlen(ssid) == 0);
}

void loadTuningData() {
  for (int i = 0; i < 3; i++) {
    int address = i * 12;

    String str = readEEPROM(address, 4);
    tuningData[i].kp = str.toInt();
    address += 4;

    str = readEEPROM(address, 4);
    tuningData[i].ki = str.toInt();
    address += 4;

    str = readEEPROM(address, 4);
    tuningData[i].kd = str.toInt();
  }
}

void saveTuningData(int i) {
  int address = i * 12;

  String str = String(tuningData[i].kp);
  writeEEPROM(address, str);
  address += 4;

  str = String(tuningData[i].ki);
  writeEEPROM(address, str);
  address += 4;

  str = String(tuningData[i].kd);
  writeEEPROM(address, str);
}

String readEEPROM(int index, int length) {
  String text = "";
  char ch = 1;

  for (int i = index; (i < (index + length)) && ch; ++i) {
    if (ch = EEPROM.read(i)) {
      text.concat(ch);
    }
  }

  return text;
}

int writeEEPROM(int index, String text) {
  for (int i = index; i < text.length() + index; ++i) {
    EEPROM.write(i, text[i - index]);
  }

  EEPROM.write(index + text.length(), 0);
  EEPROM.commit();

  return text.length() + 1;
}

void shakeRemote(int16_t milliseconds) {
  if (remotePort != -1) {
    FunctionData functionData = {0};
    functionData.startByte = 0xf9;
    functionData.parameter = milliseconds;
    functionData.checksum = functionData.parameter;
    memcpy(functionData.ssid, accessPointName, DEFAULT_SSID_LENGTH);
    memcpy(&data, &functionData, sizeof(FunctionData));

    String string1 = String("Send shake ") + functionData.parameter + " to ";
    String string2 = ipToString(remoteIP) + " port " + remotePort;
    if (Serial_rec_Debug) Serial.println(string1 + string2);

    addLine(string1);
    addLine(string2);

    udp.beginPacket(remoteIP, remotePort);
    udp.write(data, sizeof(FunctionData));
    udp.endPacket();
    delay(200);
  }
}

void blink() {
  static int8_t led = 0;
  led = 1 - led;
  digitalWrite(LED, led);
}

float Get_ChannelValue(int Ch) {

  if (WatchDogCheck > 0) {
    WatchDogCheck--;
    switch (Ch) {

      case 1:
        return TFData.roll;
        break;
      case 2:
        return TFData.pitch;
        break;
      case 3:
        return TFData.throttle;
        break;
      case 4:
        return TFData.yaw;
        break;
      default:
        return 0;
        break;
    }
  } else {
    return 0;
  }
}

