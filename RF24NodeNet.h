#ifndef __RF24_NODE_NET_H__
#define __RF24_NODE_NET_H__

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

#include "RF24NodeNetTypes.h"
#include <Timer.h>

class RF24Network;

class RF24NodeNet
{
  public:
  RF24NodeNet(RF24Network& network);  
  Timer timer;

  void addDigitalPinReadHandler(digitalPinReadHandler handler);
  void addDigitalPinWriteHandler(digitalPinWriteHandler handler);
  void addDigitalPinRcvHandler(digitalPinRcvHandler handler);
  void addDigitalPinCallbackHandler(digitalFallbackCallback callback);

  uint8_t readDigitalPin(uint16_t toAddr, uint8_t pin);
  uint8_t writeDigitalPin(uint16_t toAddr, DigitalPin pin);
  uint8_t sendDigitalPin(uint16_t toAddr, DigitalPin pin);

  void addPwmPinReadHandler(pwmPinReadHandler handler);
  void addPwmPinWriteHandler(pwmPinWriteHandler handler);
  void addPwmPinRcvHandler(pwmPinRcvHandler handler);
  void addPwmPinCallbackHandler(pwmFallbackCallback callback);

  uint8_t readPwmPin(uint16_t toAddr, uint8_t pin);
  uint8_t writePwmPin(uint16_t toAddr, PwmPin pin);
  uint8_t sendPwmPin(uint16_t toAddr, PwmPin pin);

  void addAnalogPinReadHandler(analogPinReadHandler handler);
  void addAnalogPinWriteHandler(analogPinWriteHandler handler);
  void addAnalogPinRcvHandler(analogPinRcvHandler handler);
  void addAnalogPinCallbackHandler(analogFallbackCallback callback);

  uint8_t readAnalogPin(uint16_t toAddr, uint8_t pin);
  uint8_t writeAnalogPin(uint16_t toAddr, AnalogPin pin);
  uint8_t sendAnalogPin(uint16_t toAddr, AnalogPin pin);  

  uint8_t readHeartbeat(uint16_t toAddr);
  uint8_t sendHeartbeat(uint16_t toAddr);
  uint8_t writeDigitalBitArray(uint16_t toAddr, DigitalBitArray bitArray);
  //  uint8_t writePacket(uint16_t toAddr, uint8_t type, void* packet);
//  uint8_t sendPacket(uint16_t toAddr, uint8_t type, void* packet);

  void addDigitalBitArrayReadHandler(digitalBitArrayReadHandler handler);
  void addDigitalBitArrayWriteHandler(digitalBitArrayWriteHandler handler);
  void addDigitalBitArrayRcvHandler(digitalBitArrayRcvHandler handler);
  void addDigitalBitArrayCallbackHandler(digitalBitArrayFallbackCallback callback);

  uint8_t readDigitalBitArray(uint16_t toAddr, uint8_t latch_id);
  uint8_t sendDigitalBitArray(uint16_t toAddr, DigitalBitArray digital_bit_array);

  DigitalPin GetNewDigitalPin(uint8_t pin, uint8_t state, uint8_t fallback = HIGH, uint8_t mode = OUTPUT);
  PwmPin GetNewPwmPin(uint8_t pin, uint8_t value, uint8_t fallback = 0);
  AnalogPin GetNewAnalogPin(uint8_t pin, uint8_t value, uint8_t fallback = 0, uint8_t mode = INPUT);
  DigitalBitArray GetNewDigitalBitArray(uint8_t array_id, uint8_t bits, uint8_t modes = B11111111, uint8_t fallbacks = B11111111);
  void begin();
  void update(void);

  private:
  RF24Network& _network;
  int8_t _callbackTimer;
  int8_t _fallbackTimer;

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  BitArray<uint64_t> _digitalFallbackPins;
#else
  BitArray<uint16_t> _digitalFallbackPins;
#endif
  int8_t _pwmFallbackPins[NUM_PWM_PINS];
  int8_t _analogFallbackPins[NUM_ANALOG_INPUTS];
  BitArray<uint8_t> _digitalBitArrayFallbackPins;

  digitalFallbackCallback _digitalFallbackCallback;
  pwmFallbackCallback _pwmFallbackCallback;
  analogFallbackCallback _analogFallbackCallback;
  digitalBitArrayFallbackCallback _digitalBitArrayFallbackCallback;

  static void _fbCallback(void* context);  
  static void _heartbeatCallback(void* context);

  digitalPinReadHandler _digitalPinReadHandler;
  digitalPinWriteHandler _digitalPinWriteHandler;
  digitalPinRcvHandler _digitalPinRcvHandler;
  void _digitalPinHandler(RF24NetworkHeader header);

  pwmPinReadHandler _pwmPinReadHandler;
  pwmPinWriteHandler _pwmPinWriteHandler;
  pwmPinRcvHandler _pwmPinRcvHandler;
  void _pwmPinHandler(RF24NetworkHeader header);

  analogPinReadHandler _analogPinReadHandler;
  analogPinWriteHandler _analogPinWriteHandler;
  analogPinRcvHandler _analogPinRcvHandler;
  void _analogPinHandler(RF24NetworkHeader header);

  void _heartbeatHandler(RF24NetworkHeader header);

  digitalBitArrayReadHandler _digitalBitArrayReadHandler;
  digitalBitArrayWriteHandler _digitalBitArrayWriteHandler;
  digitalBitArrayRcvHandler _digitalBitArrayRcvHandler;
  void _digitalBitArrayHandler(RF24NetworkHeader header);

  uint8_t _write(uint16_t toAddr, uint8_t type, const void* message);

  DigitalPin& _readPin(DigitalPin& pin);
  PwmPin& _readPin(PwmPin& pin);
  AnalogPin& _readPin(AnalogPin& pin);

  void _writePin(DigitalPin& pin);
  void _writePin(PwmPin& pin);
  void _writePin(AnalogPin& pin);

  void _printDigitalFallbackDebug(uint8_t pin, uint8_t value);
  void _printPwmFallbackDebug(uint8_t pin, uint8_t value);
  void _printAnalogFallbackDebug(uint8_t pin, uint8_t value);
  void _printDigitalBitArrayFallbackDebug(uint8_t pin, uint8_t value);
  void _printPwmPinSetDebug(PwmPin pwm);
  void _printPwmPinGetDebug(PwmPin pwm);
  void _printDigitalPinSetDebug(DigitalPin digital);
  void _printDigitalPinGetDebug(DigitalPin digital);
};
#endif
