#include "Q_BEE_V6_bluetooth.h"
#include <SoftwareSerial.h>

// Define RX and TX pins for Bluetooth module (change as needed)
#define BT_RX_PIN A0
#define BT_TX_PIN A1
SoftwareSerial bluetoothSerial(BT_RX_PIN, BT_TX_PIN); // RX, TX

static String last_command = "";
static bool command_available = false;

void bluetooth_init(long baudrate) {
    bluetoothSerial.begin(baudrate); // Use SoftwareSerial for Bluetooth
}

void bluetooth_process() {
    while (bluetoothSerial.available()) {
        char c = bluetoothSerial.read();
        if (c == '\n' || c == '\r') {
            if (last_command.length() > 0) {
                command_available = true;
            }
        } else {
            last_command += c;
        }
    }
}

bool bluetooth_command_available() {
    return command_available;
}

String bluetooth_get_command() {
    return last_command;
}

void bluetooth_clear_command() {
    last_command = "";
    command_available = false;
}
