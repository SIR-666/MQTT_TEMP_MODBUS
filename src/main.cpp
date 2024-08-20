#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include "WiFi.h"
#include "ESP.h"

// Define pins for SoftwareSerial
#define TX_PIN 18
#define RX_PIN 19
#define DE_RE_PIN 2 // Control pin for DE/RE of RS485

// Instantiate SoftwareSerial object
SoftwareSerial ModbusSerial(RX_PIN, TX_PIN);

// Function to control RS485 direction
void setRS485Transmit(bool transmit)
{
  digitalWrite(DE_RE_PIN, transmit ? HIGH : LOW);
}

// Function to calculate Modbus CRC16
uint16_t ModbusCRC16(uint8_t *buffer, uint8_t length)
{
  uint16_t crc = 0xFFFF;

  for (uint8_t pos = 0; pos < length; pos++)
  {
    crc ^= buffer[pos]; // XOR byte into least significant byte of crc

    for (uint8_t i = 8; i != 0; i--)
    { // Loop over each bit
      if ((crc & 0x0001) != 0)
      {                // If the LSB is set
        crc >>= 1;     // Shift right
        crc ^= 0xA001; // XOR with polynomial
      }
      else
      {
        crc >>= 1; // Just shift right
      }
    }
  }

  return crc;
}

// Function to send a Modbus request and read response
uint16_t readModbusRegister(uint8_t slaveAddress, uint16_t registerAddress)
{
  uint8_t request[8];
  request[0] = slaveAddress;                  // Modbus address of PTA9B01
  request[1] = 0x03;                          // Function code (0x04 = Read Input Registers)
  request[2] = (registerAddress >> 8) & 0xFF; // High byte of register address
  request[3] = registerAddress & 0xFF;        // Low byte of register address
  request[4] = 0x00;                          // Number of registers to read (High byte)
  request[5] = 0x01;                          // Number of registers to read (Low byte)

  // Calculate CRC
  uint16_t crc = ModbusCRC16(request, 6);
  request[6] = crc & 0xFF;        // CRC low byte
  request[7] = (crc >> 8) & 0xFF; // CRC high byte

  // Send Modbus request
  setRS485Transmit(true);
  ModbusSerial.write(request, 8);
  ModbusSerial.flush();
  setRS485Transmit(false);

  // Wait for the response (8 bytes)
  uint8_t response[7];
  ModbusSerial.readBytes(response, 7);

  // Check CRC of response
  crc = ModbusCRC16(response, 5);
  if (crc != ((response[6] << 8) | response[5]))
  {
    Serial.println("CRC Error!");
    return 0xFFFF; // Return an error code
  }

  // Extract the data
  uint16_t data = (response[3] << 8) | response[4];
  return data;
}

//----------------------------------------------------------------MQTT Message-------------------------------
// float temperature = 23;
const char *SSID = "IR-GA NETWORK";
const char *PWD = "indoprima";

IPAddress mqttServer(192, 168, 0, 10);

int countmqtt = 0;
void callback(char *topic, byte *payload, unsigned int length)
{
  // Serial.println("Callback");
  // Serial.println((char) payload[0]);
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    //   Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }

  if (String(topic) == "IPGP5/TEMP/WCLOUT")
  {

    Serial.println(messageTemp);
    // connectestablished(messageTemp);
  }
}
WiFiClient wifiClient = WiFiClient();
// mqttClient.subscribe("esp32/output");
PubSubClient mqttClient(mqttServer, 1883, callback, wifiClient);

void connectToWiFi()
{
  Serial.print("Connecting to ");
  Serial.println(SSID);
  WiFi.begin(SSID, PWD);
  int countconnect = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");

    delay(500);

    countconnect++;
    if (countconnect > 10)
    {
      ESP.restart();
      countconnect = 0;
    }
    // we can even make the ESP32 to sleep
  }

  if (WiFi.status() == WL_CONNECTED)
    Serial.print("Connected - ");
  // Serial.println(WiFi.localIP);
}

void reconnect()
{
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected())
  {
    Serial.println("Reconnecting to MQTT Broker..");
    String clientId = "ESP32-REQWCL_5T";

    if (mqttClient.connect(clientId.c_str()))
    {
      Serial.println("Connected.");
      mqttClient.subscribe("PGP5/TEMP/WCLOUT");
      // subscribe to topic
    }
    else
      countmqtt++;
    if (countmqtt > 50)
      ESP.restart();
  }
}

float temperature = 0;
float data_temp[4] = {0, 0, 0, 0};
String send_temp;
void setup()
{
  // Set up DE/RE pin
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW); // Start in receive mode

  // Initialize SoftwareSerial for Modbus
  ModbusSerial.begin(9600); // Modbus baud rate
  Serial.begin(115200);     // Serial monitor
  mqttClient.setCallback(callback);
  connectToWiFi();

  Serial.println("Modbus communication started...");
}

void loop()
{
  // Read register 0x0000 from PTA9B01 (adjust register address as needed)
  for (uint8_t i = 1; i < 5; i++)
  {
    uint16_t data = readModbusRegister(i, 0x0000);

    if (data != 0xFFFF)
    {
      temperature = data * 0.1; // Example conversion, adjust as needed
    }
    else
    {
      // Serial.println("Failed to read Modbus register");
      temperature = 0;
    }

    data_temp[i - 1] = temperature;

    delay(1000); // Wait 1 second before next read
  }
  for (uint8_t i = 0; i < 4; i++)
  {
    send_temp = send_temp + String(data_temp[i]) + "#";
  }

  if (WiFi.status() != WL_CONNECTED)
    connectToWiFi();

  if (!mqttClient.connected())
    reconnect();
  else
  {
    Serial.print("Temperature: ");
    Serial.println(send_temp);
    
    String sendmqtt = String("WCL5T") + "#" + send_temp;
    mqttClient.publish("/WCL5TEMP/FURNACE", sendmqtt.c_str());
    send_temp.clear();
  }

  mqttClient.loop();
}
