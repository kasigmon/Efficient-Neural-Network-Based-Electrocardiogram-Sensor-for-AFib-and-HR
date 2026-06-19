// ==========================================================================================
// BLE OUTPUT
// Minimal BLE system for ENGG499
// Kirk A. Sigmon - kirk.a.sigmon.th@dartmouth.edu
// ==========================================================================================

#ifndef BLEOUTPUT_H
#define BLEOUTPUT_H

// IMPORTS -----------------------------------------------------------------------------
#include "Arduino.h"

// BUILD SWITCH ------------------------------------------------------------------------
// BLEOutput now lives inside ENGG499 instead of its own Arduino library. That means every
// Lab 9 build that uses ENGG499 can see this translation unit. Default to real BLE output
// for the live-electrode build, but let AD3/simplified builds compile tiny no-op stubs.
#ifndef ENGG499_ENABLE_BLE_OUTPUT
#if defined(LAB9_ENABLE_BLE)
#define ENGG499_ENABLE_BLE_OUTPUT LAB9_ENABLE_BLE
#elif defined(LAB9_USE_ELECTRODES) && !(LAB9_USE_ELECTRODES)
#define ENGG499_ENABLE_BLE_OUTPUT 0
#else
#define ENGG499_ENABLE_BLE_OUTPUT 1
#endif
#endif

// BLE OUTPUT CONSTANTS ----------------------------------------------------------------
// Keep text messages short enough for the BLE string characteristic
#define BLEOUTPUT_MAX_TEXT_LENGTH 64

// BLE OUTPUT CALLBACKS ----------------------------------------------------------------
// These let sketches react to connect/disconnect edges without duplicating ArduinoBLE code
typedef void (*BLEOutputConnectionHandler)();

// API ---------------------------------------------------------------------------------
// Start the final-project BLE peripheral.
bool ble_output_begin(const char* device_name = nullptr);

// Register connect/disconnect callbacks for the BLE peripheral.
void ble_output_set_connection_handlers(
    BLEOutputConnectionHandler connected_handler,
    BLEOutputConnectionHandler disconnected_handler
);

// Give ArduinoBLE time to process events and notifications.
void ble_output_poll();

// Report whether a central is currently connected.
bool ble_output_connected();

// Write plain status text to the log characteristic.
bool ble_output_write_text(const char* text);

// Update the AFIB prediction characteristic.
bool ble_output_write_afib_prediction(bool afib_detected, bool valid = true);

// Update the heart-rate characteristic in BPM.
bool ble_output_write_heart_rate(uint16_t heart_rate_bpm);

// Format and write a compact BLE status message.
bool ble_output_write_textf(const char* format, ...);

// Delay briefly, then poll BLE once.
void ble_output_delay(uint32_t delay_ms);

#endif
