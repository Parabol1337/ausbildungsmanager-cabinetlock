/*
 * Schrankschloesser - Waveshare RP2040-ETH (CH9120 network bridge) with Modbus relay modules on RS485.
 *
 * Uses the AMLink library for the AusbildungsManager hardware interface (protocol v2). The device only opens the
 * cabinet the server tells it to open and reports whether each cabinet door is closed. Who may open which cabinet is
 * decided in the AusbildungsManager.
 *
 * Entities (N = 1..Channel):  cabinet.N (lock, action unlock)  and  cabinet.N.door (binary_sensor, on = door open).
 *
 * Needs: arduino-pico core. The library AMLink is part of this folder (src/AMLink, see DEPENDENCIES.md).
 * Setup: see README.md (target address and port of the server in CH9120.cpp, wiring table below, pairing).
 */
#include "src/AMLink/AMLink.h"

#include "CH9120.h"

// Software
#define Input_Interval 250        // ms between two input requests (modules are asked in turn)
#define Default_Pulse_Ms 1000     // relay time if the server does not ask for a specific duration
#define Max_Pulse_Ms 10000

// Hardware
#define TCP_Disconnected D17
#define Modbus_DE D2
#define Channel 6
#define TCP_Serial Serial2
#define Debug_Serial Serial
#ifndef Modbus_Serial
#define Modbus_Serial Serial1
#endif

// Wiring of every cabinet (channel 1..6): Modbus address of the relay module, number of the relay output, bit of the
// door sensor input on that module. Change this table when the wiring differs.
byte ChModulAddr[Channel]   = {2, 2, 2, 1, 1, 2};
byte ChRelaisAddr[Channel]  = {3, 8, 13, 8, 3, 18};
byte ChInputBit[Channel]    = {0, 1, 2, 0, 1, 3};

// The CH9120 holds the TCP connection; AMLink talks to it over the serial port. The device secret lives in the
// flash-backed EEPROM emulation of the RP2040.
amlink::SerialBridgeTransport transport(TCP_Serial, TCP_Disconnected);
amlink::EepromStorage storage;
AMLink amLink(transport, storage);

// Names of the entities are not copied by AMLink: they live here.
char lockKey[Channel][14];     // "cabinet.6"
char doorKey[Channel][20];     // "cabinet.6.door"
char lockLabel[Channel][14];   // "Schrank 6"
char doorLabel[Channel][20];   // "Schranktür 6"

bool lockOpen[Channel];        // relay is on (time runs)
uint32_t lockUntil[Channel];
int8_t doorReported[Channel];  // -1 unknown, 0 closed, 1 open

byte UniqueModbusSlaves[Channel];
int UniqueSlaveCount = 0;
int CurrentModbusSlave = 0;
uint32_t Input_LastInterval = 0;

// The server asks to open a cabinet: the relay module switches the relay for the time by itself, we only remember
// when it is over to report "locked" again. The context is the channel index.
int onLock(void* context, amlink::Call& call) {
  int ch = int(reinterpret_cast<intptr_t>(context));
  if (!call.is("unlock")) return amlink::AM_UNSUPPORTED;
  long ms = call.intArg("pulse_ms", Default_Pulse_Ms);
  if (ms < 100) ms = 100;
  if (ms > Max_Pulse_Ms) ms = Max_Pulse_Ms;
  byte flash = byte((ms + 50) / 100);  // 100 ms units
  SetModbusRelais(ChModulAddr[ch], ChRelaisAddr[ch], flash);
  lockUntil[ch] = millis() + uint32_t(flash) * 100;
  lockOpen[ch] = true;
  amLink.setState(lockKey[ch], "unlocked");
  return amlink::AM_OK;
}

// Pairing button: BOOTSEL (the unit has no other button).
bool pairingPressed(void*) { return BOOTSEL; }

void onStatus(void*, amlink::Status status) {
  switch (status) {
    case amlink::StatusOnline: Debug_Serial.println("Server: verbunden"); break;
    case amlink::StatusConnecting: Debug_Serial.println("Server: Anmeldung"); break;
    case amlink::StatusOffline: Debug_Serial.println("Server: nicht erreichbar"); break;
    case amlink::StatusPairing:
      Debug_Serial.print("Warten auf Freigabe, Kopplungscode: ");
      Debug_Serial.println(amLink.pairingCode());
      break;
  }
}

void setup() {
  CH9120_init();  // sets up the bridge; it connects to the server by itself
  Debug_Serial.begin(9600);
  Modbus_Serial.begin(9600);
  pinMode(Modbus_DE, OUTPUT);
  digitalWrite(Modbus_DE, LOW);

  // Which modules exist (each address once)
  for (int i = 0; i < Channel; i++) {
    bool alreadyAdded = false;
    for (int j = 0; j < UniqueSlaveCount; j++) {
      if (UniqueModbusSlaves[j] == ChModulAddr[i]) alreadyAdded = true;
    }
    if (!alreadyAdded) UniqueModbusSlaves[UniqueSlaveCount++] = ChModulAddr[i];
  }

  if (!amLink.begin(amlink::Identity("rp2040-eth-cabinet", "2.0.0"))) {
    Debug_Serial.println("AMLink konnte nicht gestartet werden (Geheimnis nicht speicherbar?)");
  }
  amLink.setPairingButton(pairingPressed);
  amLink.onStatus(onStatus);

  for (int i = 0; i < Channel; i++) {
    doorReported[i] = -1;
    snprintf(lockKey[i], sizeof(lockKey[i]), "cabinet.%d", i + 1);
    snprintf(doorKey[i], sizeof(doorKey[i]), "cabinet.%d.door", i + 1);
    snprintf(lockLabel[i], sizeof(lockLabel[i]), "Schrank %d", i + 1);
    snprintf(doorLabel[i], sizeof(doorLabel[i]), "Schranktür %d", i + 1);
    amLink.entity(lockKey[i], "lock")->label(lockLabel[i]).feature("unlock").attr("max_pulse_ms", (long)Max_Pulse_Ms)
        .onAction(onLock, reinterpret_cast<void*>(intptr_t(i)));
    amLink.entity(doorKey[i], "binary_sensor")->label(doorLabel[i]).cls("door");
    amLink.setState(lockKey[i], "locked");
  }

  Debug_Serial.println("Client gestartet");
}

void loop() {
  amLink.loop();  // connection, login, signatures, ping - never blocks
  uint32_t now = millis();

  // End of the relay times
  for (int i = 0; i < Channel; i++) {
    if (lockOpen[i] && int32_t(now - lockUntil[i]) >= 0) {
      lockOpen[i] = false;
      amLink.setState(lockKey[i], "locked");
    }
  }

  // Ask the modules for their inputs in turn (only while the server is connected: nobody else listens)
  if (amLink.online() && now - Input_LastInterval >= Input_Interval) {
    Input_LastInterval = now;
    RequestModbusInput(UniqueModbusSlaves[CurrentModbusSlave]);
    CurrentModbusSlave = (CurrentModbusSlave + 1) % UniqueSlaveCount;
  }

  // Answer: input bit 1 = door closed. Reported on change only.
  byte slave, inputs;
  if (modbusPoll(slave, inputs)) {
    for (int i = 0; i < Channel; i++) {
      if (ChModulAddr[i] != slave) continue;
      int open = ((inputs >> ChInputBit[i]) & 0x01) ? 0 : 1;
      if (open != doorReported[i]) {
        doorReported[i] = open;
        amLink.setState(doorKey[i], open ? "on" : "off");
        Debug_Serial.printf("Schrank %d (Modul %d): %s\n", i + 1, slave, open ? "offen" : "geschlossen");
      }
    }
  }
}
