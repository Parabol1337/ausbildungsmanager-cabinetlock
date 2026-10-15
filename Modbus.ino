// MODBUS Relais setzen
void SetModbusRelais(byte slave, byte output, byte flash) {
  byte message[] = {slave, 0x10, 0x00, output, 0x00, 0x02, 0x04, 0x00, 0x04, 0x00, flash};
  Send2Modbus(message, sizeof(message));
}

// MODBUS Eingang anfordern
void RequestModbusInput(byte slave) {
  byte message[] = {slave, 0x02, 0x00, 0x00, 0x00, 0x08};
  Send2Modbus(message, sizeof(message));
}

// MODBUS Senden mit CRC
void Send2Modbus(byte msg[], byte bytes) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < bytes; pos++) {
    crc ^= msg[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  digitalWrite(Modbus_DE, HIGH);
  Modbus_Serial.write(msg, bytes);
  Modbus_Serial.write(lowByte(crc));
  Modbus_Serial.write(highByte(crc));
  Modbus_Serial.flush();
  digitalWrite(Modbus_DE, LOW);
}
