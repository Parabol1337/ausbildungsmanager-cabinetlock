# Schrankschlösser (RP2040-ETH)

Firmware für die Schrankschlösser der Ausbildungswerkstatt: Waveshare **RP2040-ETH** mit **CH9120**-Netzwerkbaustein und
**Modbus-Relaismodulen** am RS485-Bus. Pro Schrank öffnet ein Relais das Schloss und ein Eingang meldet, ob die Tür
geschlossen ist. Die Firmware spricht mit dem AusbildungsManager über die Hardware-Schnittstelle
([Protokoll v2](https://github.com/Parabol1337/ausbildungsmanager-core/blob/main/docs/hardware/README.md)) mit der
Bibliothek **AMLink**.

Das Gerät entscheidet nichts: Es öffnet den Schrank, den der Server nennt, und meldet, ob die Tür offen oder zu ist.
Wer welchen Schrank öffnen darf, steht im AusbildungsManager.

## Aufbau

Pro Schrank `N` bis zu zwei Entitäten:

| Entität | Domäne | Bedeutung |
|---|---|---|
| `cabinet.N` | lock | Schloss, Aktion `unlock` (`pulse_ms`, Standard 1000 ms, höchstens 10 s, in Schritten von 100 ms) |
| `cabinet.N.door` | binary_sensor | Tür, `on` = offen, `off` = geschlossen (nur wenn der Schrank einen Kontakt hat) |

## Die Schränke einrichten

Oben in `RP2040-ETH_Cabinet.ino` steht **eine Tabelle, eine Zeile pro Schrank** (erste Zeile = Schrank 1):

```cpp
const Cabinet Cabinets[] = {
  // module  relay  contact
  {   2,      1,     1 },   // cabinet 1
  {   2,      2,     2 },   // cabinet 2
  ...
};
```

| Spalte | Bedeutung |
|---|---|
| `module` | Modbus-Adresse des Relaismoduls, an dem der Schrank hängt (1 bis 255) |
| `relay` | Nummer des Relais auf diesem Modul, das das Schloss öffnet (1 bis 8) |
| `contact` | Nummer des Eingangs auf diesem Modul, der „Tür zu“ meldet (1 bis 8), oder `0` wenn der Schrank keinen Kontakt hat |

Zeilen lassen sich beliebig hinzufügen oder entfernen; die Anzahl der Schränke und Entitäten richtet sich danach. Die
Tabelle entspricht der bisherigen Verdrahtung (die alte Firmware hatte stattdessen Registeradressen `3, 8, 13, 18` für die
Relais 1 bis 4 und Bitnummern ab 0 für die Eingänge).

## Adresse eines Relaismoduls einstellen

Die Module sind ab Werk auf Adresse **255** eingestellt (Bereich 1 bis 255). Jedes Modul am Bus braucht eine eigene
Adresse, passend zur Spalte `module`. Einstellen mit diesem Board (siehe auch `modbus_rtu__ip_en.pdf`, Punkt 7):

1. Nur das **eine** Modul an den Bus hängen (der Befehl geht an Adresse 0, dann hören alle Module am Bus zu).
2. Beim Bauen `-DProgram_Module_Address=5` setzen (oder in der Arduino IDE den Wert in der `.ino` von `0` auf die neue
   Adresse ändern) und hochladen. Das Board macht dann nichts anderes: Es schickt die Adresse und fragt sie danach jede
   Sekunde ab. Auf der seriellen Konsole (9600 Baud) steht `Modul-Adresse: 5`.
3. Zurück auf `0` stellen, Modul beschriften, nächstes Modul einzeln anschließen.

Die Baudrate der Module ist ab Werk 9600; sie lässt sich mit `Modbus_Baud` anpassen.

## Anderes Board oder andere Module

- **Pins und Schnittstellen** stehen im Abschnitt „THE BOARD“ oben in der `.ino` (`Modbus_Serial`, `Modbus_DE`,
  `TCP_Disconnected`, …) und lassen sich auch beim Bauen mit `-D…` überschreiben. `Modbus_DE = -1` für RS485-Wandler,
  die die Richtung selbst umschalten.
- **Andere Relaismodule:** Die Modbus-Teile stehen getrennt in `Modbus.h` / `Modbus.cpp` und wissen nichts vom Board.
  Die Registeradresse eines Relais ist `Modbus_Relay_Register_Base + Modbus_Relay_Register_Step * (Relais - 1)`
  (Waveshare: 3 und 5); bei einem anderen Modul beim Bauen setzen.
- **Anderes Netzwerk (WLAN, W5500 …):** Das CH9120 gehört zur Netzwerkseite. Mit anderem Netzwerk wird in der `.ino`
  statt `SerialBridgeTransport` der passende Transport von AMLink benutzt (`ClientTransport` für WiFi/Ethernet-Clients);
  Entitäten und Modbus bleiben gleich. Das ist nicht getestet.

Gegenüber der bisherigen Firmware:

- Die Modbus-Teile liegen jetzt in `Modbus.h` / `Modbus.cpp`, die Einstellungen in einer Tabelle pro Schrank.
- Die Zeilen-/`cmd`-Logik und ArduinoJson entfallen, ebenso das blockierende Warten auf die Verbindung.
- Die Modbus-Antworten werden mit ihrer Prüfsumme (CRC) geprüft und nicht mehr blockierend gelesen. Antworten auf
  Relaisbefehle werden erkannt und verworfen.
- Das Schloss meldet nach der Öffnungszeit wieder `locked`.
- Die Eingänge werden alle 250 ms abwechselnd pro Modul gefragt (vorher 1 s), solange der Server verbunden ist.

## Einrichtung

1. **Arduino IDE** mit dem Kern *Raspberry Pi Pico/RP2040* (arduino-pico) für das Board Waveshare RP2040-ETH.
2. Es muss **nichts weiter installiert werden**: AMLink liegt im Ordner (`src/AMLink`, siehe
   [DEPENDENCIES.md](DEPENDENCIES.md); dort steht auch die Version des Board-Pakets). ArduinoJson wird nicht mehr gebraucht.
3. In `CH9120.cpp` Server und Port eintragen: `CH9120_TARGET_IP` (Adresse des AusbildungsManagers) und
   `CH9120_TARGET_PORT = 8010` (Hardware-Gateway; bisher stand dort 8000). Die Zeile `CH9120_SetTargetIP(...)` war
   auskommentiert und ist jetzt aktiv: Der Chip bekommt die Adresse aus der Datei und nicht aus einer früheren Einstellung.
4. Sketch hochladen. Auf der seriellen Konsole (9600 Baud) erscheint der Zustand und beim ersten Start der **Kopplungscode**.
5. **Freigabe** im AusbildungsManager unter *Administration → Hardware*: Das Gerät erscheint als „Wartet auf Freigabe“.
   Den angezeigten Code vergleichen und „Code stimmt – freigeben“, oder „Freigabe am Gerät starten“ und innerhalb von zwei
   Minuten **BOOTSEL** drücken (das Gerät hat keine andere Taste). Danach Standort und Raum zuordnen.

Das Geheimnis des Geräts wird beim ersten Start erzeugt und im Flash gespeichert. Ein Neuaufspielen der Firmware behält
es. Soll das Gerät als neues Gerät angemeldet werden, im AusbildungsManager löschen und das Geheimnis zurücksetzen
(`amLink.factoryReset()`).

## Tests auf dem Rechner

Der **unveränderte Sketch** (mit `Modbus.cpp`) lässt sich als Programm für den Rechner bauen. Der CH9120 und der
RS485-Bus mit zwei Relaismodulen (Adresse 1 und 2) werden darum herum simuliert; die Verbindung geht über einen echten
Socket zum echten Gateway. Der Test dafür liegt im Server-Repository (`tests/Feature/Hardware/CabinetLockSketchTest.php`):

```bash
# im Server-Repository (ausbildungsmanager-core), beide Repositories als Nachbarordner
php artisan test --filter=CabinetLockSketchTest
```

Er prüft die Einrichtung (12 Entitäten), dass genau der verlangte Schrank am richtigen Modul geöffnet wird (mit Dauer und
gültiger CRC), das Melden von Tür offen/zu und dass ein stummer Bus die Verbindung zum Server nicht stört, und das Einstellen einer Modul-Adresse. Der Pfad lässt
sich über `CABINETLOCK_PATH` setzen. Gebaut wird mit `make -C test/host` (nur ein C++-Compiler nötig).

**Nicht geprüft** ist der Bau und Lauf auf dem echten Board mit dem arduino-pico-Kern und an den echten Relaismodulen:
Der Test ersetzt Arduino-Klassen, Treiber, CH9120 und die Module durch Nachbauten. Zum Beispiel muss `BOOTSEL` im
verwendeten Kern verfügbar sein, und das Zeitverhalten am RS485-Bus (Antwortzeit der Module) ist nur angenommen.
