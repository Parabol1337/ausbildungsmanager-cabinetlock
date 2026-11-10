#include "Modbus.h"

void ModbusRelays::begin() {
  if (dePin_ >= 0) {
    pinMode(dePin_, OUTPUT);
    digitalWrite(dePin_, LOW);
  }
}

uint16_t ModbusRelays::crc(const uint8_t* data, int length) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < length; pos++) {
    crc ^= data[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// Sends the message and appends the CRC (low byte first).
void ModbusRelays::send(const uint8_t* message, int length) {
  uint16_t check = crc(message, length);
  if (dePin_ >= 0) digitalWrite(dePin_, HIGH);
  port_.write(message, length);
  port_.write(uint8_t(check & 0xFF));
  port_.write(uint8_t(check >> 8));
  port_.flush();
  if (dePin_ >= 0) digitalWrite(dePin_, LOW);
}

void ModbusRelays::pulse(const ModbusBoardType& type, uint8_t module, uint8_t relay, uint8_t steps) {
  uint16_t reg = uint16_t(type.relayRegisterBase + type.relayRegisterStep * (relay - 1));
  const uint8_t message[] = {module, 0x10, uint8_t(reg >> 8), uint8_t(reg & 0xFF), 0x00, 0x02, 0x04, 0x00, 0x04, 0x00, steps};
  send(message, sizeof(message));
}

void ModbusRelays::requestInputs(uint8_t module) {
  const uint8_t message[] = {module, 0x02, 0x00, 0x00, 0x00, 0x08};
  send(message, sizeof(message));
}

void ModbusRelays::setAddress(uint8_t newAddress) {
  const uint8_t message[] = {0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, newAddress};
  send(message, sizeof(message));
}

void ModbusRelays::requestAddress() {
  const uint8_t message[] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x01};
  send(message, sizeof(message));
}

bool ModbusRelays::poll(Reply& reply) {
  reply.kind = Reply::None;
  while (port_.available() && used_ < (int)sizeof(buffer_)) {
    buffer_[used_++] = uint8_t(port_.read());
    lastByte_ = millis();
  }
  if (used_ && millis() - lastByte_ > Modbus_Reply_Timeout) used_ = 0;  // incomplete frame: forget it

  while (used_ >= 5) {
    int length = 0;
    if (buffer_[1] == 0x02 && buffer_[2] == 1) length = 6;        // module, function, byte count, data, crc
    else if (buffer_[1] == 0x03 && buffer_[2] == 2) length = 7;   // address: ..., 2 data bytes, crc
    else if (buffer_[1] == 0x10) length = 8;                      // answer to a write command
    else if (buffer_[1] & 0x80) length = 5;                       // error answer
    if (length == 0) {                                            // not a frame start: resynchronise
      memmove(buffer_, buffer_ + 1, --used_);
      continue;
    }
    if (used_ < length) return false;                             // wait for the rest

    uint16_t check = crc(buffer_, length - 2);
    bool good = buffer_[length - 2] == uint8_t(check & 0xFF) && buffer_[length - 1] == uint8_t(check >> 8);
    bool found = false;
    if (good && buffer_[1] == 0x02) {
      reply.kind = Reply::Inputs;
      reply.module = buffer_[0];
      reply.value = buffer_[3];
      found = true;
    } else if (good && buffer_[1] == 0x03) {
      reply.kind = Reply::Address;
      reply.module = buffer_[0];
      reply.value = buffer_[4];
      found = true;
    }
    int drop = good ? length : 1;
    used_ -= drop;
    memmove(buffer_, buffer_ + drop, used_);
    if (found) return true;
  }
  return false;
}
