#ifndef Q_BEE_V6_BLUETOOTH_H
#define Q_BEE_V6_BLUETOOTH_H

#include <Arduino.h>
#include <SoftwareSerial.h>

// Expose the Bluetooth serial object for use in other files if needed
extern SoftwareSerial bluetoothSerial;

// Initializes the JDY-31 Bluetooth module (SSP-C)
void bluetooth_init(long baudrate = 9600);

// Checks for and processes incoming Bluetooth commands
void bluetooth_process();

// Returns true if a new command is available
bool bluetooth_command_available();

// Gets the last received command (as a String)
String bluetooth_get_command();

// Clears the last received command
void bluetooth_clear_command();

#endif // Q_BEE_V6_BLUETOOTH_H
