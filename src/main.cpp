#include <Arduino.h>
#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
#include <ETHClass2.h> //Is to use the modified ETHClass
#define ETH ETH2
#else
#include <ETH.h>
#endif
#include <SPI.h>
#include <SD.h>
#include "utilities.h" //Board PinMap
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include "example_generated.h"

#define RGBLED_PIN 12
#define NUM_LEDS 8

// The configuration here must be an unused IP address, otherwise it will cause device network conflicts.
// Change to IP and DNS corresponding to your network, gateway
IPAddress staticIP(192, 168, 36, 112);
IPAddress gateway(192, 168, 36, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 36, 1);

// The udp library class
WiFiUDP udp;

const int udpPort = 3333;

static bool eth_connected = false;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, RGBLED_PIN, NEO_GRB + NEO_KHZ800);

void setup();
void loop();
void colorWipe(uint32_t color, int wait);

void WiFiEvent(arduino_event_id_t event)
{
  switch (event)
  {
  case ARDUINO_EVENT_ETH_START:
    Serial.println("ETH Started");
    // set eth hostname here
    ETH.setHostname("esp32-ethernet");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    Serial.println("ETH Connected");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    Serial.print("ETH MAC: ");
    Serial.print(ETH.macAddress());
    Serial.print(", IPv4: ");
    Serial.print(ETH.localIP());
    if (ETH.fullDuplex())
    {
      Serial.print(", FULL_DUPLEX");
    }

    Serial.print(", ");
    Serial.print(ETH.linkSpeed());
    Serial.println("Mbps");
    eth_connected = true;

    // initializes the UDP state
    // This initializes the transfer buffer
    Serial.print("Start UDP receiver to ");
    Serial.print(udpPort);
    Serial.println(" Port");
    udp.begin(WiFi.localIP(), udpPort);

    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    Serial.println("ETH Disconnected");
    eth_connected = false;
    break;
  case ARDUINO_EVENT_ETH_STOP:
    Serial.println("ETH Stopped");
    eth_connected = false;
    break;
  default:
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  colorWipe(strip.Color(255, 0, 0), 50); // Red
  colorWipe(strip.Color(0, 255, 0), 50); // Green
  colorWipe(strip.Color(0, 0, 255), 50); // Blue
  WiFi.onEvent(WiFiEvent);

#ifdef ETH_POWER_PIN
  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, HIGH);
#endif

#ifdef ETH_POWER_PIN
  pinMode(ETH_POWER_PIN, OUTPUT);
  digitalWrite(ETH_POWER_PIN, HIGH);
#endif

#if CONFIG_IDF_TARGET_ESP32
  if (!ETH.begin(ETH_TYPE, ETH_ADDR, ETH_MDC_PIN,
                 ETH_MDIO_PIN, ETH_RESET_PIN, ETH_CLK_MODE))
  {
    Serial.println("ETH start Failed!");
  }
#else
  if (!ETH.begin(ETH_PHY_W5500, 1, ETH_CS_PIN, ETH_INT_PIN, ETH_RST_PIN,
                 SPI3_HOST,
                 ETH_SCLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN))
  {
    Serial.println("ETH start Failed!");
  }
#endif

  // No intermediate routing is used and IP addresses need to be configured manually
  if (ETH.config(staticIP, gateway, subnet, dns, dns) == false)
  {
    Serial.println("Configuration failed.");
  }
}

void loop()
{
  if (eth_connected)
  {
    if (udp.parsePacket() > 0)
    {
      // Assuming the entire FlatBuffer is received in one UDP packet.
      size_t len = udp.available();
      uint8_t *buf = new uint8_t[len];
      udp.read(buf, len);

      // Get access to the root table (Monster)
      auto monster = MyGame::Sample::GetMonster(buf);

      // Access fields from the Monster table
      auto pos = monster->pos(); // Accessing a struct
      Serial.print("Monster Position X: ");
      Serial.println(pos->x());
      Serial.print("Monster Position Y: ");
      Serial.println(pos->y());
      Serial.print("Monster Position Z: ");
      Serial.println(pos->z());

      Serial.print("Monster HP: ");
      Serial.println(monster->hp());
      Serial.print("Monster Mana: ");
      Serial.println(monster->mana());
      Serial.print("Monster Name: ");
      Serial.println(monster->name()->c_str());

      // Clean up the buffer
      delete[] buf;
    }
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t color, int wait)
{
  for (int i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, color);
    strip.show();
    delay(wait);
  }
}
