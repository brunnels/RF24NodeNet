#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24NodeNet.h>

static uint16_t nodeAddress = 00; // RF24Network address of this node.
static uint16_t remoteAddr = 01;
static uint8_t remoteLedPin = 5;

uint8_t ledState = LOW; // The current state of the remote LED
uint32_t previousMillis = 0; // The last time the remote LED was updated
static uint16_t interval = 1000; // How often to update the LED

RF24 radio(7, 8);
RF24Network network(radio);
RF24NodeNet nodenet(network);
DigitalPin remotePin;

// Callback when a digital pin status packet is received.
void getDigitalPinStatus(uint16_t fromAddr, DigitalPin pin) {
  if (fromAddr == remoteAddr) {
    Serial.print("Remote pin ");
    Serial.print(pin.pin);
    Serial.print(" is now ");
    Serial.println(pin.state == HIGH ? "HIGH" : "LOW");
    ledState = pin.state;
  }
}

void setup() {
  // Initialise the SPI bus.
  SPI.begin();
  // Initialise the nrf24L01 radio.
  radio.begin();
  // Initialise the RF24Network.
  network.begin( /*channel*/ 90, /*node address*/ nodeAddress);
  // Initialise the RF24NodeNet
  nodenet.begin();

  // Attach the digital pin receive handler callback to listen for responses.
  nodenet.addDigitalPinRcvHandler(getDigitalPinStatus);

  // Initialise Serial debugging
  Serial.begin(115200);
}

void loop() {
  // Pump the RF24NodeNet
  nodenet.update();

  uint32_t currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    if (ledState == LOW) {
      remotePin = { remoteLedPin, OUTPUT, HIGH };
      nodenet.writeDigitalPin(remoteAddr, remotePin);
      delay(5);
      remotePin = { 13, OUTPUT, HIGH };
      nodenet.writeDigitalPin(remoteAddr, remotePin);
    }
    else {
      remotePin = { remoteLedPin, OUTPUT, LOW };
      nodenet.writeDigitalPin(remoteAddr, remotePin);
      delay(5);
      remotePin = { 13, OUTPUT, LOW };
      nodenet.writeDigitalPin(remoteAddr, remotePin);
    }

  }
}