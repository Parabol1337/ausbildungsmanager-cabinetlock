/*
 * Schrankschloesser - Waveshare RP2040-ETH (CH9120 network bridge) with Modbus relay modules on RS485.
 *
 * Uses the AMLink library for the AusbildungsManager hardware interface (protocol v2). The device only opens the
 * cabinet the server tells it to open and reports whether each cabinet door is closed. Who may open which cabinet is
 * decided in the AusbildungsManager.
 *
 * Entities (per cabinet N):  cabinet.N (lock, action unlock)  and, if the cabinet has a contact,
 *                            cabinet.N.door (binary_sensor, on = door open).
 *
 * Needs: arduino-pico core. The library AMLink is part of this folder (src/AMLink, see DEPENDENCIES.md).
 * Setup: see README.md (the cabinet table below, address of the server in CH9120.cpp, pairing, module addresses).
 */
#include "src/AMLink/AMLink.h"

#include "CH9120.h"
#include "Modbus.h"

// ---------------------------------------------------------------------------------------------------------------------
// 1. THE CABINETS - one line each, in the order of the cabinet numbers (cabinet 1 first). Add or remove lines freely.
//
//      module  = Modbus address of the relay module this cabinet is wired to (1..255, see README)
//      relay   = number of the relay on that module that opens the lock (1..8)
//      contact = number of the input on that module that reports "door closed" (1..8), or 0 if there is no contact
// ---------------------------------------------------------------------------------------------------------------------
struct Cabinet {
  uint8_t module;
  uint8_t relay;
  uint8_t contact;
};

const Cabinet Cabinets[] = {
  // module  relay  contact
  {   2,      1,     1 },   // cabinet 1
  {   2,      2,     2 },   // cabinet 2
  {   2,      3,     3 },   // cabinet 3
  {   1,      2,     1 },   // cabinet 4
  {   1,      1,     2 },   // cabinet 5
  {   2,      4,     4 },   // cabinet 6
};
#define Channel (sizeof(Cabinets) / sizeof(Cabinets[0]))

// ---------------------------------------------------------------------------------------------------------------------
// 2. THE BOARD - pins and serial ports. Another board: change these (or pass them when building, e.g. -DModbus_DE=5).
//    The network part (CH9120 on Serial2) is board specific too; see README for other network chips.
// ---------------------------------------------------------------------------------------------------------------------
#ifndef TCP_Disconnected
#define TCP_Disconnected D17      // CH9120 pin: HIGH while the chip has no connection to the server
#endif
#ifndef Modbus_DE
#define Modbus_DE D2              // RS485 direction pin (HIGH = sending); -1 if the transceiver switches by itself
#endif
#ifndef Modbus_Serial
#define Modbus_Serial Serial1     // serial port of the RS485 transceiver
#endif
#ifndef Modbus_Baud
#define Modbus_Baud 9600          // delivered as 9600 by the module
#endif
#ifndef TCP_Serial
#define TCP_Serial Serial2        // serial port of the CH9120
#endif
#ifndef Debug_Serial
#define Debug_Serial Serial
#endif

// ---------------------------------------------------------------------------------------------------------------------
// 3. SOFTWARE
// ---------------------------------------------------------------------------------------------------------------------
#define Input_Interval 250        // ms between two input requests (modules are asked in turn)
#define Default_Pulse_Ms 1000     // relay time if the server does not ask for a specific duration
#define Max_Pulse_Ms 10000        // longest relay time (the module counts in steps of 100 ms, at most 25.5 s)

// Setting the address of a relay module (see README): connect ONLY that module, build with the new address in
// -DProgram_Module_Address=<1..255>. The board then does nothing else: it sends the address and shows what the module
// answers. 0 = normal operation.
#ifndef Program_Module_Address
#define Program_Module_Address 0
#endif

// The CH9120 holds the TCP connection; AMLink talks to it over the serial port. The device secret lives in the
// flash-backed EEPROM emulation of the RP2040.
amlink::SerialBridgeTransport transport(TCP_Serial, TCP_Disconnected);
amlink::EepromStorage storage;
AMLink amLink(transport, storage);
ModbusRelays modbus(Modbus_Serial, Modbus_DE);

// Names of the entities are not copied by AMLink: they live here.
char lockKey[Channel][14];     // "cabinet.12"
char doorKey[Channel][20];     // "cabinet.12.door"
char lockLabel[Channel][16];   // "Schrank 12"
char doorLabel[Channel][20];   // "Schranktür 12"

bool lockOpen[Channel];        // relay is on (time runs)
uint32_t lockUntil[Channel];
int8_t doorReported[Channel];  // -1 unknown, 0 closed, 1 open

uint8_t Modules[Channel];      // each module address once
unsigned ModuleCount = 0;
unsigned CurrentModule = 0;
uint32_t Input_LastInterval = 0;

// The server asks to open a cabinet: the relay module switches the relay for the time by itself, we only remember
// when it is over to report "locked" again. The context is the cabinet index.
int onLock(void* context, amlink::Call& call) {
  unsigned ch = unsigned(reinterpret_cast<intptr_t>(context));
  if (!call.is("unlock")) return amlink::AM_UNSUPPORTED;
  long ms = call.intArg("pulse_ms", Default_Pulse_Ms);
  if (ms < 100) ms = 100;
  if (ms > Max_Pulse_Ms) ms = Max_Pulse_Ms;
  uint8_t tenths = uint8_t((ms + 50) / 100);
  modbus.pulse(Cabinets[ch].module, Cabinets[ch].relay, tenths);
  lockUntil[ch] = millis() + uint32_t(tenths) * 100;
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
  Debug_Serial.begin(9600);
  Modbus_Serial.begin(Modbus_Baud);
  modbus.begin();

  if (Program_Module_Address) {
    Debug_Serial.println("Programmiermodus: nur EIN Relaismodul am Bus!");
    modbus.setAddress(Program_Module_Address);
    Debug_Serial.printf("Neue Adresse %d gesendet\n", Program_Module_Address);
    return;
  }

  CH9120_init();  // sets up the bridge; it connects to the server by itself

  // Which modules exist (each address once)
  for (unsigned i = 0; i < Channel; i++) {
    bool alreadyAdded = false;
    for (unsigned j = 0; j < ModuleCount; j++) {
      if (Modules[j] == Cabinets[i].module) alreadyAdded = true;
    }
    if (!alreadyAdded) Modules[ModuleCount++] = Cabinets[i].module;
  }

  if (!amLink.begin(amlink::Identity("rp2040-eth-cabinet", "2.0.0"))) {
    Debug_Serial.println("AMLink konnte nicht gestartet werden (Geheimnis nicht speicherbar?)");
  }
  amLink.setPairingButton(pairingPressed);
  amLink.onStatus(onStatus);

  for (unsigned i = 0; i < Channel; i++) {
    doorReported[i] = -1;
    snprintf(lockKey[i], sizeof(lockKey[i]), "cabinet.%u", i + 1);
    snprintf(doorKey[i], sizeof(doorKey[i]), "cabinet.%u.door", i + 1);
    snprintf(lockLabel[i], sizeof(lockLabel[i]), "Schrank %u", i + 1);
    snprintf(doorLabel[i], sizeof(doorLabel[i]), "Schranktür %u", i + 1);
    amLink.entity(lockKey[i], "lock")->label(lockLabel[i]).feature("unlock").attr("max_pulse_ms", (long)Max_Pulse_Ms)
        .onAction(onLock, reinterpret_cast<void*>(intptr_t(i)));
    if (Cabinets[i].contact) amLink.entity(doorKey[i], "binary_sensor")->label(doorLabel[i]).cls("door");
    amLink.setState(lockKey[i], "locked");
  }

  Debug_Serial.println("Client gestartet");
}

// Programming mode: show what the module answers to "what is your address?"
void programmingLoop() {
  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last = millis();
    modbus.requestAddress();
  }
  ModbusRelays::Reply reply;
  if (modbus.poll(reply) && reply.kind == ModbusRelays::Reply::Address) {
    Debug_Serial.printf("Modul-Adresse: %d\n", reply.value);
  }
}

void loop() {
  if (Program_Module_Address) {
    programmingLoop();
    return;
  }

  amLink.loop();  // connection, login, signatures, ping - never blocks
  uint32_t now = millis();

  // End of the relay times
  for (unsigned i = 0; i < Channel; i++) {
    if (lockOpen[i] && int32_t(now - lockUntil[i]) >= 0) {
      lockOpen[i] = false;
      amLink.setState(lockKey[i], "locked");
    }
  }

  // Ask the modules for their inputs in turn (only while the server is connected: nobody else listens)
  if (amLink.online() && now - Input_LastInterval >= Input_Interval) {
    Input_LastInterval = now;
    modbus.requestInputs(Modules[CurrentModule]);
    CurrentModule = (CurrentModule + 1) % ModuleCount;
  }

  // Answer: input = 1 means door closed. Reported on change only.
  ModbusRelays::Reply reply;
  if (modbus.poll(reply) && reply.kind == ModbusRelays::Reply::Inputs) {
    for (unsigned i = 0; i < Channel; i++) {
      if (Cabinets[i].module != reply.module || !Cabinets[i].contact) continue;
      int open = ((reply.value >> (Cabinets[i].contact - 1)) & 0x01) ? 0 : 1;
      if (open != doorReported[i]) {
        doorReported[i] = open;
        amLink.setState(doorKey[i], open ? "on" : "off");
        Debug_Serial.printf("Schrank %u (Modul %d): %s\n", i + 1, reply.module, open ? "offen" : "geschlossen");
      }
    }
  }
}
