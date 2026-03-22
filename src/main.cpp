#include <Arduino.h>
#include <VL53L1X_ULD.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP23X08.h>
#include <Wire.h>
#include "config.h"
#include "sensors.h"

Adafruit_NeoPixel pixels(7, leds, NEO_GRB + NEO_KHZ800);
Adafruit_MCP23X08 mcp;
VL53L1X_ULD sensor[sc];

const int threshold = 500;

void scanI2C()
{
  byte error, address;
  int count = 0;

  Serial.println("\nScanning I2C bus...");

  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("Device found at 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      count++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (count == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("Scan complete");

  Serial.println();

  delay(3000);
}

void drive(int left, int right)
{
  left = constrain(left, -100, 100);
  right = constrain(right, -100, 100);

  int pwmLeft = map(abs(left), 0, 100, 0, 255);
  int pwmRight = map(abs(right), 0, 100, 0, 255);

  if (left > 0)
  {
    digitalWrite(m1a, HIGH);
    digitalWrite(m1b, LOW);
  }
  else if (left < 0)
  {
    digitalWrite(m1a, LOW);
    digitalWrite(m1b, HIGH);
  }
  else
  {
    digitalWrite(m1a, LOW);
    digitalWrite(m1b, LOW);
  }

  if (right > 0)
  {
    digitalWrite(m2a, HIGH);
    digitalWrite(m2b, LOW);
  }
  else if (right < 0)
  {
    digitalWrite(m2a, LOW);
    digitalWrite(m2b, HIGH);
  }
  else
  {
    digitalWrite(m2a, LOW);
    digitalWrite(m2b, LOW);
  }

  analogWrite(pwm1, pwmLeft);
  analogWrite(pwm2, pwmRight);
}

void setup()
{
  Serial.begin(115200);
  pixels.begin();

  Wire.begin(sda, scl, 100000);

  for (int i = 0; i < 10; i++)
  {
    Wire.beginTransmission(0x20);
    Wire.endTransmission();
    delay(10);
  }

  if (!mcp.begin_I2C(0x20, &Wire))
  {
    Serial.println("Error: MCP23008 not found!");
    pixels.fill(pixels.Color(80, 0, 0));
    pixels.show();
    while (1)
      scanI2C();
  }

  for (int i = 0; i < 8; i++)
  {
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i, LOW);
  }

  for (int i = 0; i < sc; i++)
  {
    if (!init_sensor(sensor[i], addresses[i], xshut_pins[i]))
    {
      Serial.printf("Error initializing sensor %d at address 0x%02X\n", i, addresses[i]);
      pixels.setPixelColor(i, pixels.Color(80, 0, 0));
      pixels.show();
      while (1)
        ;
    }

    set_sensor_settings(sensor[i], Short, 11, 5, 58, 15, 15, threshold);
    pixels.setPixelColor(i, pixels.Color(0, 80, 0));
    pixels.show();
    Serial.printf("Sensor %d initialized and moved to 0x%02X\n", i, sensor[i].GetI2CAddress());
  }

  pixels.fill(pixels.Color(0, 0, 0));
  pixels.show();

  delay(250);

  for (int i = 0; i < sc; i++)
  {
    pixels.setPixelColor(i, pixels.Color(0, 80, 0));
    pixels.show();
    delay(40);
  }

  delay(250);

  pixels.fill(pixels.Color(0, 0, 0));
  pixels.show();

  Serial.println("All sensors initialized successfully!");

  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);
  pinMode(m2a, OUTPUT);
  pinMode(m2b, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(stby, OUTPUT);

  digitalWrite(stby, HIGH);
}

void loop()
{
  VL53L1X_Result_t results;
  for (int i = 0; i < sc; i++)
  {
    sensor[i].GetResult(&results);
    uint16_t distance = (results.Status == 0) ? results.Distance : threshold;

    if (distance < threshold)
    {
      pixels.setPixelColor(i, pixels.Color(0, 0, 80));
    }
    else
    {
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    }

    pixels.show();

    Serial.printf("%d ", distance);

    sensor[i].ClearInterrupt();
  }

  Serial.println();

  // drive(100, 100);
  // delay(1000);

  // // backward
  // drive(-100, -100);
  // delay(1000);

  // // spin right
  // drive(100, -100);
  // delay(1000);

  // // spin left
  // drive(-100, 100);
  // delay(1000);

  // // gentle forward
  // drive(50, 50);
  // delay(1000);

  // // curve right
  // drive(100, 50);
  // delay(1000);

  // // curve left
  // drive(50, 100);
  // delay(1000);

  // // stop
  // drive(0, 0);
  // delay(1000);
}