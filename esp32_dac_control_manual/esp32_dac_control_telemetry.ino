#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

Adafruit_MCP4725 mcp4725;
Adafruit_ADS1115 ads;

// ----------------------------------------------------
// I2C pins
// ----------------------------------------------------
#define SDA_PIN 21
#define SCL_PIN 22

// ----------------------------------------------------
// ESP32 DAC1 - GPIO25, DAC2 - GPIO26
// ----------------------------------------------------
#define DAC1_PIN 25
#define DAC2_PIN 26
#define ESP32_VREF 3.3

// ----------------------------------------------------
// MCP4725
// ----------------------------------------------------
#define MCP_VREF 3.3
#define MCP_MAX_VOLTAGE 2.0   // hardware ceiling noted from original sweep test

// ----------------------------------------------------
// ADS1115 channel mapping for readback
// ASSUMPTION: A2 reads DAC2. Change if wired differently.
// ----------------------------------------------------
#define ADC_CH_MCP  0   // A0 - MCP4725 output
#define ADC_CH_DAC1 1   // A1 - DAC1 output
#define ADC_CH_DAC2 2   // A2 - DAC2 output

// How often to send a telemetry point, in milliseconds
#define TELEMETRY_INTERVAL_MS 500

// ----------------------------------------------------
// BLE UUIDs
// ----------------------------------------------------
#define SERVICE_UUID                  "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CONTROL_CHARACTERISTIC_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
// New characteristic, used only for streaming readings out (NOTIFY)
#define TELEMETRY_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

BLEServer *pServer = nullptr;
BLECharacteristic *pControlCharacteristic = nullptr;
BLECharacteristic *pTelemetryCharacteristic = nullptr;
bool deviceConnected = false;

// Current target voltages, updated live from BLE writes
volatile float dac1Voltage = 0.0;
volatile float dac2Voltage = 0.0;
volatile float mcpVoltage  = 0.0;

unsigned long lastTelemetryMs = 0;


// ====================================================
// Output setters
// ====================================================
void setDAC1Voltage(float voltage)
{
  int dacValue = round((voltage / ESP32_VREF) * 255.0);
  dacValue = constrain(dacValue, 0, 255);
  dacWrite(DAC1_PIN, dacValue);
}

void setDAC2Voltage(float voltage)
{
  int dacValue = round((voltage / ESP32_VREF) * 255.0);
  dacValue = constrain(dacValue, 0, 255);
  dacWrite(DAC2_PIN, dacValue);
}

void setMCPVoltage(float voltage)
{
  if (voltage > MCP_MAX_VOLTAGE)
  {
    voltage = MCP_MAX_VOLTAGE;
  }
  uint16_t dacValue = round((voltage / MCP_VREF) * 4095.0);
  if (dacValue > 4095) dacValue = 4095;
  mcp4725.setVoltage(dacValue, false);
}


// ====================================================
// BLE: server connect/disconnect
// ====================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    deviceConnected = true;
    Serial.println("Client connected");
  }

  void onDisconnect(BLEServer *server) override {
    deviceConnected = false;
    Serial.println("Client disconnected, restarting advertising");
    BLEDevice::startAdvertising();
  }
};


// ====================================================
// BLE: incoming control write handler
// Expected formats: "D1:1.20"  "D2:2.50"  "MCP:0.75"
// ====================================================
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String rxValue = characteristic->getValue();
    if (rxValue.length() == 0) return;

    Serial.print("Received: ");
    Serial.println(rxValue);

    int separatorIndex = rxValue.indexOf(':');
    if (separatorIndex == -1)
    {
      Serial.println("Ignored: no ':' separator found");
      return;
    }

    String channel = rxValue.substring(0, separatorIndex);
    float voltage = rxValue.substring(separatorIndex + 1).toFloat();
    channel.trim();
    channel.toUpperCase();

    if (channel == "D1")
    {
      dac1Voltage = voltage;
      setDAC1Voltage(dac1Voltage);
    }
    else if (channel == "D2")
    {
      dac2Voltage = voltage;
      setDAC2Voltage(dac2Voltage);
    }
    else if (channel == "MCP")
    {
      mcpVoltage = voltage;
      setMCPVoltage(mcpVoltage);
    }
    else
    {
      Serial.print("Unknown channel: ");
      Serial.println(channel);
    }
  }
};


// ====================================================
// Read one ADS1115 channel and convert to volts
// ====================================================
float readChannelVolts(uint8_t channel)
{
  int16_t raw = ads.readADC_SingleEnded(channel);
  return ads.computeVolts(raw);
}


// ====================================================
// Build and send one telemetry point over BLE NOTIFY
// Format: "set_d1,meas_d1,set_d2,meas_d2,set_mcp,meas_mcp"
// ====================================================
void sendTelemetry()
{
  float measD1  = readChannelVolts(ADC_CH_DAC1);
  float measD2  = readChannelVolts(ADC_CH_DAC2);
  float measMcp = readChannelVolts(ADC_CH_MCP);

  char payload[80];
  snprintf(payload, sizeof(payload), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
           dac1Voltage, measD1,
           dac2Voltage, measD2,
           mcpVoltage, measMcp);

  pTelemetryCharacteristic->setValue((uint8_t*)payload, strlen(payload));
  pTelemetryCharacteristic->notify();

  Serial.print("Telemetry: ");
  Serial.println(payload);
}


// ====================================================
// SETUP
// ====================================================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("ESP32 + MCP4725 + ADS1115 - BLE manual control + telemetry");
  Serial.println("==========================================");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mcp4725.begin(0x60, &Wire))
  {
    Serial.println("ERROR: MCP4725 not found!");
    while (1) { delay(1000); }
  }
  Serial.println("MCP4725 detected.");

  if (!ads.begin(0x48, &Wire))
  {
    Serial.println("ERROR: ADS1115 not found!");
    while (1) { delay(1000); }
  }
  Serial.println("ADS1115 detected.");
  ads.setGain(GAIN_TWOTHIRDS); // +/- 6.144 V

  setDAC1Voltage(dac1Voltage);
  setDAC2Voltage(dac2Voltage);
  setMCPVoltage(mcpVoltage);

  // --------------------------------------------------
  // BLE setup
  // --------------------------------------------------
  BLEDevice::init("ESP32-DAC-Controller");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Control characteristic: website writes to this
  pControlCharacteristic = pService->createCharacteristic(
      CONTROL_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR
  );
  pControlCharacteristic->setCallbacks(new MyCallbacks());
  pControlCharacteristic->addDescriptor(new BLE2902());

  // Telemetry characteristic: ESP32 notifies the website
  pTelemetryCharacteristic = pService->createCharacteristic(
      TELEMETRY_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pTelemetryCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE server started, waiting for a connection...");
  Serial.println("Send values as D1:<v>  D2:<v>  MCP:<v>");
}


// ====================================================
// LOOP
// Outputs only change on a BLE write (handled in
// MyCallbacks::onWrite). Telemetry streams out on a timer.
// ====================================================
void loop()
{
  if (deviceConnected)
  {
    unsigned long now = millis();
    if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)
    {
      lastTelemetryMs = now;
      sendTelemetry();
    }
  }
  delay(10);
}
