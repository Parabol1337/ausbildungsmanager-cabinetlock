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

// What differs between module types: where the registers of the relays are and how the inputs are numbered.
// Describe another module here (or in the sketch) and give it to the cabinets that use it.
struct ModbusBoardType {
  const char* name;
  uint16_t relayRegisterBase;   // register of relay 1 for "switch on for a time" ...
  uint16_t relayRegisterStep;   // ... relay n has base + step * (n - 1)
  uint16_t pulseUnitMs;         // the time of that command is counted in steps of this many ms
  bool inputsMsbFirst;          // false: input 1 is bit 0 of the answer, true: input 1 is bit 7
};

// Waveshare "Modbus RTU Relay" (the 1, 2, 4 and 8 relay versions speak the same commands; relay numbers 1..8, inputs 1..8): registers 0x0003, 0x0008, 0x000D, ..., steps of 0.1 s.
const ModbusBoardType ModbusWaveshare = {"Waveshare Modbus RTU Relay", 3, 5, 100, false};

// How long a missing answer is waited for
#ifndef Modbus_Reply_Timeout
#define Modbus_Reply_Timeout 200
#endif

class ModbusRelays {
public:
  struct Reply {
    enum Kind { None, Inputs, Address } kind;
    uint8_t module;   // address the answer came from
    uint8_t value;    // Inputs: the 8 input bits as the module sent them (see inputOn()).  Address: the address the module reports
  };

  // port: the serial port of the RS485 transceiver (already begin()ed). dePin: direction pin (HIGH while sending), or
  // -1 for transceivers that switch by themselves.
  ModbusRelays(Stream& port, int dePin) : port_(port), dePin_(dePin), used_(0), lastByte_(0) {}

  void begin();

  // Relay 1..8 of a module on for `steps` (1..255) x the time step of its type; the module switches it off by itself.
  void pulse(const ModbusBoardType& type, uint8_t module, uint8_t relay, uint8_t steps);
  // Ask a module for its 8 inputs.
  void requestInputs(uint8_t module);
  // Give ONE module a new address. Goes to address 0 (everybody listens): only one module may be on the bus!
  void setAddress(uint8_t newAddress);
  // Ask for the address (address 0: whatever module is alone on the bus answers).
  void requestAddress();

  // Takes in received bytes. True when a complete, correct answer to an input or address request is there.
  // Answers to relay commands are consumed and dropped, damaged bytes are skipped.
  bool poll(Reply& reply);

  // Is input 1..8 on in the answer `bits` of a module of this type?
  static bool inputOn(const ModbusBoardType& type, uint8_t bits, uint8_t input) {
    return (bits >> (type.inputsMsbFirst ? 8 - input : input - 1)) & 0x01;
  }

  static uint16_t crc(const uint8_t* data, int length);

private:
  void send(const uint8_t* message, int length);

  Stream& port_;
  int dePin_;
  uint8_t buffer_[16];
  int used_;
  uint32_t lastByte_;
};
