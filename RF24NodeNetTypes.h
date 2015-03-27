#ifndef __RF24_NODE_NET_TYPES_H__
#define __RF24_NODE_NET_TYPES_H__

#include <stdint.h>

#define HEARTBEAT_INTERVAL 5000
#define FALLBACK_TIMEOUT 30000
#define BASE_NODE_ADDRESS 00
#define PKT_DIGITALPIN 1
#define PKT_PWMPIN 2
#define PKT_ANALOGPIN 3
#define PKT_HEARTBEAT 4
#define PKT_BITARRAY 5

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define NUM_PWM_PINS 15
#else
#define NUM_PWM_PINS 6
#endif

template<typename T>
class BitArray
{
public:
  explicit BitArray(T initialValues)
  {
    _bits = initialValues;
  }

  bool Set(uint8_t bitIndex, bool value)
  {
    if (bitIndex > getMaxBits()) return false;

    T mask = 1 << bitIndex;
    // clear bit
    _bits &= ~mask;

    if (value)
    {
      // set bit
      _bits |= mask;
    }

    return true;
  }

  bool IsTrue(uint8_t bitIndex)
  {
    if (bitIndex > getMaxBits()) return false;

    T mask = 1 << bitIndex;
    return ((_bits & mask) > 0);
  }

  bool IsFalse(uint8_t bitIndex)
  {
    if (bitIndex > getMaxBits()) return false;

    T mask = 1 << bitIndex;
    return ((_bits & mask) == 0);
  }

  uint8_t getMaxBits()
  {
    return (sizeof(T) * 8);
  }

  bool operator[] (uint8_t bitIndex)
  {
    return IsTrue(bitIndex);
  }

  operator T()
  {
    return _bits;
  }

private:
  T _bits;
};

struct DigitalBitArray
{
  DigitalBitArray() : bits(0), modes(0), fallbacks(0) {}

  uint8_t array_id;
  BitArray<uint8_t> bits;
  BitArray<uint8_t> modes;
  BitArray<uint8_t> fallbacks;
};

struct DigitalPin
{
  DigitalPin() : mode(OUTPUT), state(HIGH), fallback(-1) {}

  uint8_t pin;
  uint8_t mode;
  uint8_t state;
  int8_t fallback;
};

struct PwmPin
{
  PwmPin() : value(0), fallback(-1) {}

  uint8_t pin;
  uint8_t value;
  int8_t fallback;
};

struct AnalogPin
{
  AnalogPin() : mode(INPUT), value(0), fallback(-1) {}

  uint8_t pin;
  uint8_t mode;
  uint16_t value;
  int8_t fallback;
};

struct Heartbeat
{
  Heartbeat() : response(255) {}
  uint8_t response;
};

typedef void (*digitalPinReadHandler)(uint16_t fromAddr, uint8_t pin);
typedef void (*digitalPinWriteHandler)(uint16_t fromAddr, DigitalPin pin);
typedef void (*digitalPinRcvHandler)(uint16_t fromAddr, DigitalPin pin);

typedef void (*pwmPinReadHandler)(uint16_t fromAddr, uint8_t pin);
typedef void (*pwmPinWriteHandler)(uint16_t fromAddr, PwmPin pin);
typedef void (*pwmPinRcvHandler)(uint16_t fromAddr, PwmPin pin);

typedef void (*analogPinReadHandler)(uint16_t fromAddr, uint8_t pin);
typedef void (*analogPinWriteHandler)(uint16_t fromAddr, AnalogPin pin);
typedef void (*analogPinRcvHandler)(uint16_t fromAddr, AnalogPin pin);

typedef void(*digitalBitArrayReadHandler)(uint16_t fromAddr, uint8_t array_id);
typedef void(*digitalBitArrayWriteHandler)(uint16_t fromAddr, DigitalBitArray bitarray);
typedef void(*digitalBitArrayRcvHandler)(uint16_t fromAddr, DigitalBitArray bitarray);

#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
typedef void(*digitalFallbackCallback)(int8_t fallbackTimer, BitArray<uint64_t> fallbackBits);
#else
typedef void(*digitalFallbackCallback)(int8_t fallbackTimer, BitArray<uint16_t> fallbackBits);
#endif
typedef void(*pwmFallbackCallback)(int8_t fallbackTimer, int8_t pwmFallbackPins[NUM_PWM_PINS]);
typedef void(*analogFallbackCallback)(int8_t fallbackTimer, int8_t analogFallbackPins[NUM_ANALOG_INPUTS]);
typedef void(*digitalBitArrayFallbackCallback)(int8_t fallbackTimer, BitArray<uint8_t> fallbackBits);

#endif
