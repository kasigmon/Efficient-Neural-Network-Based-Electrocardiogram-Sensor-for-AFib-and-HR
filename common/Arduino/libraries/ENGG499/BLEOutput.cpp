// ==========================================================================================
// BLE OUTPUT
// Minimal BLE system for ENGG499
// Kirk A. Sigmon - kirk.a.sigmon.th@dartmouth.edu
// ==========================================================================================

#include "BLEOutput.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if ENGG499_ENABLE_BLE_OUTPUT

#include <ArduinoBLE.h>

namespace {

// BLE CONSTANTS -----------------------------------------------------------------------
// This UUID is the stable BLE identity for the final project board
constexpr char kStableDeviceName[] = "ENGG499_BOARD";
constexpr char kBoardServiceUuid[] = "454E4747-3439-0000-8000-0F4F0C499009";

// Lab 9 UUIDs intentionally supersede the earlier HR-monitor UUIDs so stale Lab 8
// peripherals do not look like valid final-project boards.
constexpr char kLogServiceUuid[] = "4C4F4700-8F16-4A52-A3A2-0F4F0C499009";
constexpr char kLogCharacteristicUuid[] = "4C4F4701-8F16-4A52-A3A2-0F4F0C499009";
constexpr char kAfibServiceUuid[] = "41464942-8F16-4A52-A3A2-0F4F0C499009";
constexpr char kAfibCharacteristicUuid[] = "41464942-8F16-4A52-A3A2-0F4F0C499109";
constexpr char kHeartRateServiceUuid[] = "48520000-8F16-4A52-A3A2-0F4F0C499009";
constexpr char kHeartRateCharacteristicUuid[] = "48520001-8F16-4A52-A3A2-0F4F0C499009";

// Keep the AFIB label small enough for the string characteristic.
constexpr size_t kAfibTextLength = 8;

// BLE STATE ---------------------------------------------------------------------------
// Everything starts off until ble_output_begin() successfully advertises.
bool g_ble_initialized = false;
BLEOutputConnectionHandler g_connected_handler = nullptr;
BLEOutputConnectionHandler g_disconnected_handler = nullptr;

// BOARD IDENTITY - stable advertised/discoverable service UUID only
BLEService g_board_service(kBoardServiceUuid);

// LOG - human-readable status text
BLEService g_log_service(kLogServiceUuid);
BLEStringCharacteristic g_log_characteristic(
    kLogCharacteristicUuid,
    BLERead | BLENotify,
    BLEOUTPUT_MAX_TEXT_LENGTH
);
BLEDescriptor g_log_description("2901", "Log Output");

// AFIB PREDICTION - the current classifier result
BLEService g_afib_service(kAfibServiceUuid);
BLEStringCharacteristic g_afib_characteristic(
    kAfibCharacteristicUuid,
    BLERead | BLENotify,
    kAfibTextLength
);
BLEDescriptor g_afib_description("2901", "AFIB Prediction");

// HEART RATE - the latest estimated BPM value
BLEService g_heart_rate_service(kHeartRateServiceUuid);
BLEUnsignedShortCharacteristic g_heart_rate_characteristic(
    kHeartRateCharacteristicUuid,
    BLERead | BLENotify
);
BLEDescriptor g_heart_rate_description("2901", "Heart Rate BPM");

// HELPERS -----------------------------------------------------------------------------
// Copy a C string into a fixed-size BLE buffer without risking a missing terminator.
size_t clamp_copy_c_string(char* destination, size_t destination_length, const char* source) {

  // BLEStringCharacteristic has a fixed max length. This helper truncates safely.
  if (destination == nullptr || destination_length == 0) {
    return 0;
  }

  if (source == nullptr) {
    destination[0] = '\0';
    return 0;
  }

  size_t copied_length = 0;
  while (copied_length + 1 < destination_length && source[copied_length] != '\0') {
    destination[copied_length] = source[copied_length];
    copied_length++;
  }

  destination[copied_length] = '\0';
  return copied_length;
}

// Notify the sketch when a BLE central connects.
void ble_output_connected_handler(BLEDevice central) {
  (void)central;

  if (g_connected_handler != nullptr) {
    g_connected_handler();
  }
}

// After a disconnect, notify the sketch and resume advertising immediately.
void ble_output_disconnected_handler(BLEDevice central) {
  (void)central;

  if (g_disconnected_handler != nullptr) {
    g_disconnected_handler();
  }

  BLE.advertise();
}

}

// API ---------------------------------------------------------------------------------

// Start the BLE services and begin advertising the board.
bool ble_output_begin(const char* device_name) {

  // Don't re-initialize if a sketch calls begin twice
  if (g_ble_initialized) {
    return true;
  }
  if (!BLE.begin()) {
    return false;
  }

  // Keep BLE identity stable
  (void)device_name;
  BLE.setLocalName(kStableDeviceName);
  BLE.setDeviceName(kStableDeviceName);
  BLE.setAdvertisedServiceUuid(kBoardServiceUuid);

  // Add short human-readable descriptions for generic BLE browser apps
  g_log_characteristic.addDescriptor(g_log_description);
  g_afib_characteristic.addDescriptor(g_afib_description);
  g_heart_rate_characteristic.addDescriptor(g_heart_rate_description);
  g_log_service.addCharacteristic(g_log_characteristic);
  g_afib_service.addCharacteristic(g_afib_characteristic);
  g_heart_rate_service.addCharacteristic(g_heart_rate_characteristic);

  // The board service is just identity. The other services are actual outputs: log text,
  // AFIB/NORMAL/UNKNOWN, and HR in BPM.
  BLE.addService(g_board_service);
  BLE.addService(g_log_service);
  BLE.addService(g_afib_service);
  BLE.addService(g_heart_rate_service);

  // Keep connect/disconnect handling inside the wrapper and expose simple callbacks
  BLE.setEventHandler(BLEConnected, ble_output_connected_handler);
  BLE.setEventHandler(BLEDisconnected, ble_output_disconnected_handler);

  // Seed characteristics with readable defaults
  g_log_characteristic.writeValue("BOOT");
  g_afib_characteristic.writeValue("UNKNOWN");
  g_heart_rate_characteristic.writeValue(static_cast<unsigned short>(0));

  if (!BLE.advertise()) {
    BLE.end();
    return false;
  }

  g_ble_initialized = true;
  return true;
}

// Register sketch-level callbacks for BLE connection state changes.
void ble_output_set_connection_handlers(
    BLEOutputConnectionHandler connected_handler,
    BLEOutputConnectionHandler disconnected_handler
) {
  g_connected_handler = connected_handler;
  g_disconnected_handler = disconnected_handler;
}

// Give ArduinoBLE time to process connect, disconnect, and notification work.
void ble_output_poll() {
  if (!g_ble_initialized) {
    return;
  }

  BLE.poll();
}

// Confirm whether we're initialized and a central is currently connected.
bool ble_output_connected() {
  return g_ble_initialized && BLE.connected();
}

// Write status text, truncating it to the characteristic length.
bool ble_output_write_text(const char* text) {
  if (text == nullptr) {
    return false;
  }

  // Truncate before checking initialization so formatting behavior is deterministic
  char truncated_text[BLEOUTPUT_MAX_TEXT_LENGTH + 1];
  clamp_copy_c_string(truncated_text, sizeof(truncated_text), text);

  if (!g_ble_initialized) {
    return false;
  }

  const bool text_ok = g_log_characteristic.writeValue(truncated_text) > 0;
  BLE.poll();
  return text_ok;
}

// Write the AFIB output as a small readable label.
bool ble_output_write_afib_prediction(bool afib_detected, bool valid) {
  if (!g_ble_initialized) {
    return false;
  }

  const char* label = "UNKNOWN";
  if (valid) {
    label = afib_detected ? "AFIB" : "NORMAL";
  }

  const bool write_ok = g_afib_characteristic.writeValue(label) > 0;
  BLE.poll();
  return write_ok;
}

// Write the heart-rate output in BPM
bool ble_output_write_heart_rate(uint16_t heart_rate_bpm) {
  if (!g_ble_initialized) {
    return false;
  }

  const bool write_ok =
      g_heart_rate_characteristic.writeValue(static_cast<unsigned short>(heart_rate_bpm)) > 0;
  BLE.poll();
  return write_ok;
}

// Format a compact status string, then write it to the log characteristic
bool ble_output_write_textf(const char* format, ...) {
  if (format == nullptr) {
    return false;
  }

  char buffer[BLEOUTPUT_MAX_TEXT_LENGTH + 1];
  va_list arguments;
  va_start(arguments, format);
  vsnprintf(buffer, sizeof(buffer), format, arguments);
  va_end(arguments);

  return ble_output_write_text(buffer);
}

// Delay briefly while still giving ArduinoBLE one poll at the end
void ble_output_delay(uint32_t delay_ms) {
  delay(delay_ms);
  ble_output_poll();
}

#else

// NO-BLE STUBS ------------------------------------------------------------------------
// When the AD3/simplified build disables electrode mode, the sketch still sees the same
// API, but the ENGG499 library does not drag the ArduinoBLE stack into flash/SRAM.
bool ble_output_begin(const char* device_name) {
  (void)device_name;
  return false;
}

void ble_output_set_connection_handlers(
    BLEOutputConnectionHandler connected_handler,
    BLEOutputConnectionHandler disconnected_handler
) {
  (void)connected_handler;
  (void)disconnected_handler;
}

void ble_output_poll() {
}

bool ble_output_connected() {
  return false;
}

bool ble_output_write_text(const char* text) {
  (void)text;
  return false;
}

bool ble_output_write_afib_prediction(bool afib_detected, bool valid) {
  (void)afib_detected;
  (void)valid;
  return false;
}

bool ble_output_write_heart_rate(uint16_t heart_rate_bpm) {
  (void)heart_rate_bpm;
  return false;
}

bool ble_output_write_textf(const char* format, ...) {
  (void)format;
  return false;
}

void ble_output_delay(uint32_t delay_ms) {
  delay(delay_ms);
}

#endif
