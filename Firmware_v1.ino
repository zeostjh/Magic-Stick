#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OSCMessage.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP Udp;
char ssid[] = "#########";
char password[] = "##########";
IPAddress localIP(10, 0, 0, 10);
IPAddress destIP(10, 0, 0, 3);
const int port = 8000;

MPU6050 mpu6050(Wire);
float shakeThreshold = 5.0;
float shake = 0;

const int hallPin = 2;
volatile bool hallTriggered = false;

const int rPin = 5;
const int gPin = 6;
const int bPin = 7;

Adafruit_SSD1306 display(0x3D);

// Function declarations
void tap();
void tapFlash();
void spell();
void spellFlash();
void alternateColors();
void fadeTo(int red, int green, int blue, int time);
void lightning();
////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Udp.begin(port);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(hallPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(hallPin), []() { hallTriggered = true; }, RISING);
  pinMode(rPin, OUTPUT);
  pinMode(gPin, OUTPUT);
  pinMode(bPin, OUTPUT);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) 
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.display();
  delay(2000);
  display.clearDisplay();
}
////////////////////////////////////////////////////////////////////////////////////
void loop() {
  mpu6050.update();
  float accZ = mpu6050.getAccZ();
  float gyroY = mpu6050.getGyroY();
  float gyroX = mpu6050.getGyroX();
  if (accZ >= 0.1) {
    tap();
  } else if (accZ <= -0.1) {
    spell();
  }
}
////////////////////////////////////////////////////////////////////////////////////
void tap() 
{
  float accZ = mpu6050.getAccZ();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(hallPin);
  display.println(accZ);
  display.display();
  analogWrite(rPin,  138);
  analogWrite(gPin, 43);
  analogWrite(bPin, 226);
  
  if (digitalRead(hallPin) == HIGH) 
  {
    tapFlash();
    delay(2000);
    fadeTo(138, 43, 226, 2000);
  }
}

void tapFlash() 
{
  OSCMessage msg("/wand/flash/go");
  Udp.beginPacket(destIP, port);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  lightning();
}
////////////////////////////////////////////////////////////////////////////////////
void spell() 
{
  float gyroX = mpu6050.getGyroX();
  float gyroY = mpu6050.getGyroY(); 
  shake = (gyroX + gyroY / 2);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(shake);
  display.println(shakeThreshold);
  display.display();
  if (abs(shake) > shakeThreshold) 
  {
    spellFlash();
  } else if (abs(shake) < shakeThreshold) 
  {
    alternateColors();
  }
}

void spellFlash() 
{
  OSCMessage msg("/magic/flash/go");
  Udp.beginPacket(destIP, port);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  lightning();
}
////////////////////////////////////////////////////////////////////////////////////
void lightning() 
{
  unsigned long startTime = millis();
  int duration = 2000;
  int fadeDuration = 500;
  while (millis() - startTime < duration) 
  {
    int intensity = random(128, 256);
    analogWrite(rPin, intensity);
    analogWrite(gPin, intensity);
    analogWrite(bPin, intensity);
    delay(fadeDuration);
    for (int i = intensity; i >= 0; i -= 10) 
    {
      analogWrite(rPin, i);
      analogWrite(gPin, i);
      analogWrite(bPin, i);
      delay(10);
    }
    delay(50);
  }
}
////////////////////////////////////////////////////////////////////////////////////
void alternateColors() 
{
  fadeTo(128, 0, 128, 2000); //purpletr
  fadeTo(  0, 0, 255, 2000); //blue
}
////////////////////////////////////////////////////////////////////////////////////
void fadeTo(int red, int green, int blue, int time) 
{
  int steps = 256;
  int delayTime = time / steps;
  for (int i = 0; i <= steps; ++i) 
  {
    analogWrite(rPin,   map(i, 0, steps, analogRead(rPin), red));
    analogWrite(gPin, map(i, 0, steps, analogRead(gPin), green));
    analogWrite(bPin,  map(i, 0, steps, analogRead(bPin), blue));
    delay(delayTime);
  }
}
