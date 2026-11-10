#pragma once
/*
 * Modbus RTU to relay modules on an RS485 bus (tested with the Waveshare "Modbus RTU Relay", see modbus_rtu__ip_en.pdf).
 *
 * Nothing here blocks: a request is sent, the answer is collected byte by byte in poll() and checked with its CRC.
 * Every module has its own address (1..255, delivered as 255, see README for how to change it) and has relays 1..8 and
 * inputs 1..8. This file does not know the board: the serial port and the direction pin are handed in.
 */
#include <Arduino.h>
#include <Stream.h>
#include <stdint.h>

// Register of relay n (1..8) for "switch on for a time": Modbus_Relay_Register_Base + Modbus_Relay_Register_Step * (n - 1).
// These are the values of the Waveshare module (0x0003, 0x0008, 0x000D, ...). Another module type: set them when building.
#ifndef Modbus_Relay_Register_Base
#define Modbus_Relay_Register_Base 3
#endif
#ifndef Modbus_Relay_Register_Step
#define Modbus_Relay_Register_Step 5
#endif
// How long a missing answer is waited for
#ifndef Modbus_Reply_Timeout
#define Modbus_Reply_Timeout 200
#endif

class ModbusRelays {
public:
  struct Reply {
    enum Kind { None, Inputs, Address } kind;
    uint8_t module;   // address the answer came from
    uint8_t value;    // Inputs: bit 0 = input 1 ... bit 7 = input 8.  Address: the address the module reports
  };

  // port: the serial port of the RS485 transceiver (already begin()ed). dePin: direction pin (HIGH while sending), or
  // -1 for transceivers that switch by themselves.
  ModbusRelays(Stream& port, int dePin) : port_(port), dePin_(dePin), used_(0), lastByte_(0) {}

  void begin();

  // Relay 1..8 of a module on for `tenths` x 100 ms (1..255); the module switches it off by itself.
  void pulse(uint8_t module, uint8_t relay, uint8_t tenths);
  // Ask a module for its inputs.
  void requestInputs(uint8_t module);
  // Give ONE module a new address. Goes to address 0 (everybody listens): only one module may be on the bus!
  void setAddress(uint8_t newAddress);
  // Ask for the address (address 0: whatever module is alone on the bus answers).
  void requestAddress();

  // Takes in received bytes. True when a complete, correct answer to an input or address request is there.
  // Answers to relay commands are consumed and dropped, damaged bytes are skipped.
  bool poll(Reply& reply);

  static uint16_t crc(const uint8_t* data, int length);

private:
  void send(const uint8_t* message, int length);

  Stream& port_;
  int dePin_;
  uint8_t buffer_[16];
  int used_;
  uint32_t lastByte_;
};
