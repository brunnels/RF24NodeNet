#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24NodeNet.h>

static uint16_t nodeAddress = 01; // RF24Network address of this node.

RF24 radio(7, 8);
RF24Network network(radio);
RF24NodeNet nodenet(network);

// Callback when a digital pin update request is received.
void writeDigitalPin(uint16_t fromAddr, DigitalPin pin)
{
  Serial.print("Digital pin write state ");
  Serial.print(pin.state);
  Serial.print(" for pin ");
  Serial.print(pin.pin);
  Serial.print(" from node ");
  Serial.println(fromAddr);
  pinMode(pin.pin, pin.mode);
  digitalWrite(pin.pin, pin.state);
  nodenet.sendDigitalPin(fromAddr, pin);
}

void setup() {
  Serial.begin(115200);

  // Initialise the SPI bus.
  SPI.begin();
  // Initialise the nRF24L01 radio.
  radio.begin();
  // Initialise the RF24Network.
  network.begin( /*channel*/ 90, /*node address*/ nodeAddress);
  // Initialise the RF24NodeNet
  nodenet.begin();

  //nodenet.addDigitalPinWriteHandler(writeDigitalPin);
}

void loop() {
  // Update the RF24NodeNet network
  nodenet.update();
}