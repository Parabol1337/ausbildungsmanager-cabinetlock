// Modbus RTU to the relay modules (RS485 with direction pin Modbus_DE). Nothing here blocks: a request is sent, the
// answer is collected byte by byte in loop() (modbusPoll) and checked with its CRC.

#define Modbus_Reply_Timeout 200   // ms after which a missing answer is given up

uint16_t modbusCrc(const byte* data, int length) {
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

// Sends the message and appends the CRC.
void Send2Modbus(byte msg[], byte bytes) {
  uint16_t crc = modbusCrc(msg, bytes);
  digitalWrite(Modbus_DE, HIGH);
  Modbus_Serial.write(msg, bytes);
  Modbus_Serial.write(lowByte(crc));
  Modbus_Serial.write(highByte(crc));
  Modbus_Serial.flush();
  digitalWrite(Modbus_DE, LOW);
}

// Relay on for a time: "flash" in units of 100 ms (the relay module switches it off by itself).
void SetModbusRelais(byte slave, byte output, byte flash) {
  byte message[] = {slave, 0x10, 0x00, output, 0x00, 0x02, 0x04, 0x00, 0x04, 0x00, flash};
  Send2Modbus(message, sizeof(message));
}

// Request the 8 inputs of a module.
void RequestModbusInput(byte slave) {
  byte message[] = {slave, 0x02, 0x00, 0x00, 0x00, 0x08};
  Send2Modbus(message, sizeof(message));
}

// Collects answers. Returns true when a complete, correct answer to an input request is there: the bits are in
// `inputs`, the module in `slave`. Answers to relay commands are consumed and ignored. Garbage is dropped byte by byte.
bool modbusPoll(byte& slave, byte& inputs) {
  static byte buffer[16];
  static int used = 0;
  static uint32_t lastByte = 0;

  while (Modbus_Serial.available() && used < (int)sizeof(buffer)) {
    buffer[used++] = (byte)Modbus_Serial.read();
    lastByte = millis();
  }
  if (used && millis() - lastByte > Modbus_Reply_Timeout) used = 0;  // incomplete frame: forget it

  while (used >= 5) {
    int length = 0;
    if (buffer[1] == 0x02 && buffer[2] == 1) length = 6;        // slave, function, byte count, data, crc
    else if (buffer[1] == 0x10) length = 8;                     // answer to a relay command
    else if (buffer[1] & 0x80) length = 5;                      // error answer
    if (length == 0) {                                          // not a frame start: resynchronise
      memmove(buffer, buffer + 1, --used);
      continue;
    }
    if (used < length) return false;                            // wait for the rest

    uint16_t crc = modbusCrc(buffer, length - 2);
    bool good = buffer[length - 2] == lowByte(crc) && buffer[length - 1] == highByte(crc);
    bool input = good && buffer[1] == 0x02;
    if (input) {
      slave = buffer[0];
      inputs = buffer[3];
    }
    if (good) {
      used -= length;
      memmove(buffer, buffer + length, used);
    } else {
      memmove(buffer, buffer + 1, --used);
    }
    if (input) return true;
  }
  return false;
}
