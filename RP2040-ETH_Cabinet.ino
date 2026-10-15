#include "CH9120.h"
#include <ArduinoJson.h>

// Software
#define Input_Interval 1000

// Hardware
#define TCP_Disconnected D17
#define Modbus_DE D2
#define Channel 6
#define TCP_Serial Serial2
#define Debug_Serial Serial
#define Modbus_Serial Serial1





byte ChModulAddr[Channel]   = {2, 2, 2, 1, 1, 2};
byte ChRelaisAddr[Channel]  = {3, 8, 13, 8, 3, 18};
byte ChInputBit[Channel]    = {0, 1, 2, 0, 1, 3};
byte ChInputState[Channel]  = {2, 2, 2, 2, 2, 2};

JsonDocument Json_Rx;
JsonDocument Json_Tx;

String TCP_Out;
String TCP_In;

unsigned long Input_LastInterval = 0;

// Dynamisch eindeutige Slave-Adressen berechnen
byte UniqueModbusSlaves[Channel];
int UniqueSlaveCount = 0;
int CurrentModbusSlave = 0;
bool isRegistered = false;

void setup() {
  CH9120_init();
  Debug_Serial.begin(9600);
  Modbus_Serial.begin(9600);
  pinMode(Modbus_DE, OUTPUT);
  digitalWrite(Modbus_DE, LOW);

  // Einzigartige Slave-Adressen extrahieren
  for (int i = 0; i < Channel; i++) {
    bool alreadyAdded = false;
    for (int j = 0; j < UniqueSlaveCount; j++) {
      if (UniqueModbusSlaves[j] == ChModulAddr[i]) {
        alreadyAdded = true;
        break;
      }
    }
    if (!alreadyAdded) {
      UniqueModbusSlaves[UniqueSlaveCount++] = ChModulAddr[i];
    }
  }
}

void loop() {
  // TCP Verbindung prüfen
  if (digitalRead(TCP_Disconnected)) {
    Debug_Serial.println("Warten auf Verbindung zum Server");
    isRegistered = false; // ✅ Registrierung verloren
    while (digitalRead(TCP_Disconnected)) {}
    Debug_Serial.println("Verbindung zum Server hergestellt");
  }

  // TCP Daten empfangen
  while (TCP_Serial.available()) {
    TCP_In = TCP_Serial.readStringUntil('\n');
    Debug_Serial.println(TCP_In);
    DeserializationError err = deserializeJson(Json_Rx, TCP_In);
    if (err == DeserializationError::Ok) {
      String cmd = Json_Rx["cmd"].as<String>();

      if (cmd == "welcome") {
        Json_Tx.clear();
        Json_Tx["cmd"] = "welcome";
        Json_Tx["token"] = rp2040.getChipID();
        for (int i = 0; i < Channel; i++) {
          Json_Tx["ch"][i] = "cabinet";
        }
        serializeJson(Json_Tx, TCP_Out);
        TCP_Serial.println(TCP_Out);
        Debug_Serial.println(TCP_Out);
        
      }

      if (cmd == "registered") {
          isRegistered = true; // ✅ Erst ab hier starten wir mit Modbus-Zyklus
      }

      if (cmd == "open") {
        int ch = Json_Rx["ch"].as<int>() - 1;
        if (ch >= 0 && ch < Channel) {
          SetModbusRelais(ChModulAddr[ch], ChRelaisAddr[ch], 10);
          Json_Tx.clear();
          Json_Tx["cmd"] = "opened";
          Json_Tx["ch"] = ch + 1;
          serializeJson(Json_Tx, TCP_Out);
          TCP_Serial.println(TCP_Out);
          Debug_Serial.println(TCP_Out);
        } else {
          Json_Tx.clear();
          Json_Tx["cmd"] = "error";
          Json_Tx["ch"] = ch + 1;
          serializeJson(Json_Tx, TCP_Out);
          TCP_Serial.println(TCP_Out);
          Debug_Serial.println(TCP_Out);
        }
      }
    } else {
      Json_Tx.clear();
      Json_Tx["error"] = "format";
      serializeJson(Json_Tx, TCP_Out);
      TCP_Serial.println(TCP_Out);
      Debug_Serial.println(TCP_Out);
    }
  }

  // Eingänge abfragen (alle 1 Sekunde)
  if (isRegistered && millis() - Input_LastInterval >= Input_Interval) {
    Input_LastInterval = millis();
    RequestModbusInput(UniqueModbusSlaves[CurrentModbusSlave]);
    CurrentModbusSlave = (CurrentModbusSlave + 1) % UniqueSlaveCount;
  }

  // Antwort empfangen und auswerten
  if (Modbus_Serial.available()) {
    byte rcvData[6] = {};
    Modbus_Serial.readBytes(rcvData, 6);

    byte respondingSlave = rcvData[0];
    int payloadByteIndex = 3;

    for (int i = 0; i < Channel; i++) {
      if (ChModulAddr[i] != respondingSlave) continue;

      bool newState = (rcvData[payloadByteIndex] >> ChInputBit[i]) & 0x01;

      if (newState != ChInputState[i]) {
        ChInputState[i] = newState;
        Json_Tx.clear();
        Json_Tx["cmd"] = "status";
        Json_Tx["ch"] = i + 1;
        Json_Tx["state"] = newState ? "closed" : "open";
        serializeJson(Json_Tx, TCP_Out);
        TCP_Serial.println(TCP_Out);
        Debug_Serial.printf("Kanal %d (Slave %d): Zustand = %s\n", i + 1, respondingSlave, newState ?  "geschlossen" : "offen");
        delay(50);
        break; // 
      }
    }
  }
}
