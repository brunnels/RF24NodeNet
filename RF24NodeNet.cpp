#include "RF24Network.h"
#include "RF24NodeNet.h"

RF24NodeNet::RF24NodeNet(RF24Network& network) :
  _network(network),
  _callbackTimer(-1),
  _fallbackTimer(-1),
  _digitalFallbackPins(0),
  _digitalFallbackCallback(0),
  _pwmFallbackCallback(NULL),
  _analogFallbackCallback(NULL),
  _digitalBitArrayFallbackCallback(NULL),
  _digitalPinReadHandler(NULL),
  _digitalPinWriteHandler(NULL),
  _digitalPinRcvHandler(NULL),
  _pwmPinReadHandler(NULL),
  _pwmPinWriteHandler(NULL),
  _pwmPinRcvHandler(NULL),
  _analogPinReadHandler(NULL),
  _analogPinWriteHandler(NULL),
  _analogPinRcvHandler(NULL),
  _digitalBitArrayReadHandler(NULL),
  _digitalBitArrayWriteHandler(NULL),
//  _pwmFallbackPins({ 0 }),
//  _analogFallbackPins({0}),
  _digitalBitArrayRcvHandler(NULL),
  _digitalBitArrayFallbackPins(0){

  
}

void RF24NodeNet::_digitalPinHandler(RF24NetworkHeader header)
{
  DigitalPin payload;
  _network.read(header, &payload, sizeof(payload));

  if (header.type > 64)
  {
    if (_digitalPinWriteHandler != NULL)
    {
      _digitalPinWriteHandler(header.from_node, payload);
    }
    else
    {
      _writePin(payload);
//      sendPacket(header.from_node, PKT_DIGITALPIN, (void*)&payload);
      sendDigitalPin(header.from_node, payload);
    }
  }
  else if (header.type > 32)
  {
    if (_digitalPinReadHandler != NULL)
    {
      _digitalPinReadHandler(header.from_node, payload.pin);
    }
    else
    {
//      sendPacket(header.from_node, PKT_DIGITALPIN, (void*)&_readPin(payload));
      sendDigitalPin(header.from_node, payload);
    }
  }
  else
  {
    if (_digitalPinRcvHandler != NULL)
    {
      _digitalPinRcvHandler(header.from_node, payload);
    }
  }
}

void RF24NodeNet::_pwmPinHandler(RF24NetworkHeader header)
{
  PwmPin payload;
  _network.read(header, &payload, sizeof(payload));

//  Serial.print("PWM packet for pin ");
//  Serial.print(payload.pin);
//  Serial.println("Received");
//  Serial.println(header.type);

  if (header.type > 64)
  {
    if (_pwmPinWriteHandler != NULL)
    {
      _pwmPinWriteHandler(header.from_node, payload);
    }
    else
    {
      
      _writePin(payload);
      sendPwmPin(header.from_node, payload);
    }
  }
  else if (header.type > 32)
  {
    if (_pwmPinReadHandler != NULL)
    {
      _pwmPinReadHandler(header.from_node, payload.pin);
    }
    else
    {
      sendPwmPin(header.from_node, payload);
    }
  }
  else
  {
    if (_pwmPinRcvHandler != NULL)
    {
      _pwmPinRcvHandler(header.from_node, payload);
    }
  }
}

void RF24NodeNet::_analogPinHandler(RF24NetworkHeader header)
{
  AnalogPin payload;
  _network.read(header, &payload, sizeof(payload));

  if (header.type > 64)
  {
    if (_analogPinWriteHandler != NULL)
    {
      _analogPinWriteHandler(header.from_node, payload);
    }
    else
    {
      _writePin(payload);
      sendAnalogPin(header.from_node, payload);
    }
  }
  else if (header.type > 32)
  {
    if (_analogPinReadHandler != NULL)
    {
      _analogPinReadHandler(header.from_node, payload.pin);
    }
    else
    {
      sendAnalogPin(header.from_node, payload);
    }
  }
  else
  {
    if (_analogPinRcvHandler != NULL)
    {
      _analogPinRcvHandler(header.from_node, payload);
    }
  }
}

void RF24NodeNet::_heartbeatHandler(RF24NetworkHeader header)
{
//  Serial.println("Heartbeat packet");
  Heartbeat payload;
  _network.read(header, &payload, sizeof(payload));

  if (header.type > 32)
  {
//    Serial.println("Heartbeat reply");
    sendHeartbeat(header.from_node);
  }
  else
  {
//    Serial.println("Heartbeat received");
    if (_fallbackTimer != NULL)
    {
      timer.stop(_fallbackTimer);
    }
    _fallbackTimer = timer.after(FALLBACK_TIMEOUT, _fbCallback, static_cast<void*>(this));
  }
}

void RF24NodeNet::_digitalBitArrayHandler(RF24NetworkHeader header)
{
  DigitalBitArray payload;
  _network.read(header, &payload, sizeof(payload));

  if (header.type > 64)
  {
    if (_digitalBitArrayWriteHandler != NULL)
    {
      _digitalBitArrayWriteHandler(header.from_node, payload);
    }
  }
  else if (header.type > 32)
  {
    if (_digitalBitArrayReadHandler != NULL)
    {
      _digitalBitArrayReadHandler(header.from_node, payload.array_id);
    }
  }
  else
  {
    if (_digitalBitArrayRcvHandler != NULL)
    {
      _digitalBitArrayRcvHandler(header.from_node, payload);
    }
  }
}

//uint8_t RF24NodeNet::writePacket(uint16_t toAddr, uint8_t type, void* packet)
//{
//  switch (type % 32)
//  {
//  case PKT_DIGITALPIN:
//    DigitalPin *obj = (DigitalPin*) packet;
//    break;
//  case PKT_PWMPIN:
//    PwmPin *obj = (PwmPin*)packet;
//    break;
//  case PKT_ANALOGPIN:
//    AnalogPin *obj = (AnalogPin*)packet;
//    break;
//  case PKT_HEARTBEAT:
//    Heartbeat *obj = (Heartbeat*)packet;
//    break;
//  case PKT_BITARRAY:
//    DigitalBitArray *obj = (DigitalBitArray*)packet;
//    break;
//  default:
//    // No such type :-(
//    return false;
//  }
//  return _write(toAddr, type + 64, packet);
//}

//uint8_t RF24NodeNet::sendPacket(uint16_t toAddr, uint8_t type, void* packet)
//{
//  return _write(toAddr, type, &packet);
//}

uint8_t RF24NodeNet::writeDigitalPin(uint16_t toAddr, DigitalPin pin)
{
  return _write(toAddr, PKT_DIGITALPIN + 64, &pin);
}

uint8_t RF24NodeNet::readDigitalPin(uint16_t toAddr, uint8_t pin)
{
  DigitalPin payload;
  payload.pin = pin;
  return _write(toAddr, PKT_DIGITALPIN + 32, &payload);
}

uint8_t RF24NodeNet::sendDigitalPin(uint16_t toAddr, DigitalPin pin)
{
  return _write(toAddr, PKT_DIGITALPIN, &pin);
}

uint8_t RF24NodeNet::writePwmPin(uint16_t toAddr, PwmPin pin)
{
  return _write(toAddr, PKT_PWMPIN + 64, &pin);
}

uint8_t RF24NodeNet::readPwmPin(uint16_t toAddr, uint8_t pin)
{
  PwmPin payload;
  payload.pin = pin;
  return _write(toAddr, PKT_PWMPIN + 32, &payload);
}

uint8_t RF24NodeNet::sendPwmPin(uint16_t toAddr, PwmPin pin)
{
  return _write(toAddr, PKT_PWMPIN, &pin);
}

uint8_t RF24NodeNet::writeAnalogPin(uint16_t toAddr, AnalogPin pin)
{
  return _write(toAddr, PKT_ANALOGPIN + 64, &pin);
}

uint8_t RF24NodeNet::readAnalogPin(uint16_t toAddr, uint8_t pin)
{
  AnalogPin payload;
  payload.pin = pin;
  return _write(toAddr, PKT_ANALOGPIN + 32, &payload);
}

uint8_t RF24NodeNet::sendAnalogPin(uint16_t toAddr, AnalogPin pin)
{
  return _write(toAddr, PKT_ANALOGPIN, &pin);
}

uint8_t RF24NodeNet::readHeartbeat(uint16_t toAddr)
{
  Heartbeat payload;
  return _write(toAddr, PKT_HEARTBEAT + 32, &payload);
}

uint8_t RF24NodeNet::sendHeartbeat(uint16_t toAddr)
{
  Heartbeat payload;
  return _write(toAddr, PKT_HEARTBEAT, &payload);
}

uint8_t RF24NodeNet::writeDigitalBitArray(uint16_t toAddr, DigitalBitArray bitArray)
{
  return _write(toAddr, PKT_BITARRAY + 64, &bitArray);
}

uint8_t RF24NodeNet::readDigitalBitArray(uint16_t toAddr, uint8_t array_id)
{
  DigitalBitArray payload;
  payload.array_id = array_id;
  return _write(toAddr, PKT_BITARRAY + 32, &payload);
}

uint8_t RF24NodeNet::sendDigitalBitArray(uint16_t toAddr, DigitalBitArray bitArray)
{
  return _write(toAddr, PKT_BITARRAY, &bitArray);
}

void RF24NodeNet::_heartbeatCallback(void* context)
{
  RF24NodeNet* nodenet = static_cast<RF24NodeNet*>(context);
  (*nodenet).readHeartbeat(00);
//  Serial.println("Heartbeat Sent");
}

void RF24NodeNet::_printDigitalFallbackDebug(uint8_t pin, uint8_t value)
{
}

void RF24NodeNet::_printPwmFallbackDebug(uint8_t pin, uint8_t value)
{
  Serial.print("PWM Pin fallback to value ");
  Serial.print(value);
  Serial.print(" for pin ");
  Serial.println(pin);
}

void RF24NodeNet::_printAnalogFallbackDebug(uint8_t pin, uint8_t value)
{
  Serial.print("Analog Pin fallback to value ");
  Serial.print(value);
  Serial.print(" for pin ");
  Serial.println(pin);
}

void RF24NodeNet::_printDigitalBitArrayFallbackDebug(uint8_t pin, uint8_t value)
{
}

void RF24NodeNet::_fbCallback(void* context)
{
  RF24NodeNet* nodenet = static_cast<RF24NodeNet*>(context);
  if ((*nodenet)._digitalFallbackCallback != NULL)
  {
    (*nodenet)._digitalFallbackCallback((*nodenet)._fallbackTimer, (*nodenet)._digitalFallbackPins);
  }
  else
  {
    for (uint8_t i = 0; i < (*nodenet)._digitalFallbackPins.getMaxBits(); i++)
    {
      digitalWrite(i, (*nodenet)._pwmFallbackPins[i]);
    }
  }

  if ((*nodenet)._pwmFallbackCallback != NULL)
  {
    (*nodenet)._pwmFallbackCallback((*nodenet)._fallbackTimer, (*nodenet)._pwmFallbackPins);
  }
  else
  {
    for (uint8_t i = 0; i < NUM_PWM_PINS; i++)
    {
      digitalWrite(i, (*nodenet)._pwmFallbackPins[i]);
    }
  }

  if ((*nodenet)._analogFallbackCallback != NULL)
  {
    (*nodenet)._analogFallbackCallback((*nodenet)._fallbackTimer, (*nodenet)._analogFallbackPins);
  }
  else
  {
    for (uint8_t i = 0; i < NUM_ANALOG_INPUTS; i++)
    {
      digitalWrite(i, (*nodenet)._analogFallbackPins[i]);
    }
  }

  if ((*nodenet)._digitalBitArrayFallbackCallback != NULL)
  {
    (*nodenet)._digitalBitArrayFallbackCallback((*nodenet)._fallbackTimer, (*nodenet)._digitalBitArrayFallbackPins);
  }
}

DigitalPin RF24NodeNet::GetNewDigitalPin(uint8_t pin, uint8_t state, uint8_t fallback, uint8_t mode)
{
  DigitalPin digitalPin;
  digitalPin.pin = pin;
  digitalPin.state = state;
  digitalPin.fallback = fallback;
  digitalPin.mode = mode;
  return digitalPin;
}

PwmPin RF24NodeNet::GetNewPwmPin(uint8_t pin, uint8_t value, uint8_t fallback)
{
  PwmPin pwmPin;
  pwmPin.pin = pin;
  pwmPin.value = value;
  pwmPin.fallback = fallback;
  return pwmPin;
}

AnalogPin RF24NodeNet::GetNewAnalogPin(uint8_t pin, uint8_t value, uint8_t fallback, uint8_t mode)
{
  AnalogPin analogPin;
  analogPin.pin = pin;
  analogPin.value = value;
  analogPin.fallback = fallback;
  analogPin.mode = mode;
  return analogPin;
}

DigitalBitArray RF24NodeNet::GetNewDigitalBitArray(uint8_t array_id, uint8_t bits, uint8_t modes, uint8_t fallbacks)
{

}

void RF24NodeNet::_printPwmPinSetDebug(PwmPin pwm)
{
#ifdef RF24_NODENET_DEBUG
    server.print("Setting PWM pin ");
    server.print(pwm.pin);
    server.print(" to ");
    server.println(pwm.value);
#endif
}

void RF24NodeNet::_printPwmPinGetDebug(PwmPin pwm)
{
#ifdef RF24_NODENET_DEBUG
    server.print("PWM pin ");
    server.print(pwm.pin);
    server.print(" is now ");
    server.println(pwm.value);
#endif
}

void RF24NodeNet::_printDigitalPinSetDebug(DigitalPin digital)
{
#ifdef RF24_NODENET_DEBUG
    server.print("Setting Digital pin ");
    server.print(digital.pin);
    server.print(" to ");
    server.println(digital.state ? "Off" : "On");
#endif
}

void RF24NodeNet::_printDigitalPinGetDebug(DigitalPin digital)
{
#ifdef RF24_NODENET_DEBUG
    server.print("Digital pin ");
    server.print(digital.pin);
    server.print(" is now ");
    server.println(digital.state ? "Off" : "On");
#endif
}

void RF24NodeNet::_writePin(DigitalPin& pin)
{
  pinMode(pin.pin, pin.mode);
  digitalWrite(pin.pin, pin.state);
  if (pin.fallback != NULL)
  {
    _digitalFallbackPins.Set(pin.pin, pin.fallback);
  }
  _readPin(pin);
}

void RF24NodeNet::_writePin(PwmPin& pin)
{
  pinMode(pin.pin, OUTPUT);
  analogWrite(pin.pin, pin.value);
  if (pin.fallback != NULL)
  {
    _pwmFallbackPins[pin.pin] = pin.fallback;
  }
  _readPin(pin);
}

void RF24NodeNet::_writePin(AnalogPin& pin)
{
  uint8_t pinNum = analogInputToDigitalPin(pin.pin);
  pinMode(pinNum, pin.mode);
  analogWrite(pinNum, pin.value);
  if (pin.fallback != NULL)
  {
    _analogFallbackPins[pin.pin] = pin.fallback;
  }
  _readPin(pin);
}

DigitalPin& RF24NodeNet::_readPin(DigitalPin& pin)
{
  // I don't trust this, would not work in the past to read an output pin so best to assume the digitalWrite worked
  //pin.state = digitalRead(pin.pin);
  return pin;
}

PwmPin& RF24NodeNet::_readPin(PwmPin& pin)
{
//  pin.value = analogRead(pin.pin);
  return pin;
}

AnalogPin& RF24NodeNet::_readPin(AnalogPin& pin)
{
  pin.value = analogRead(analogInputToDigitalPin(pin.pin));
  return pin;
}

void RF24NodeNet::begin()
{
  _callbackTimer = timer.every(HEARTBEAT_INTERVAL, _heartbeatCallback, static_cast<void*>(this));
  _heartbeatCallback(static_cast<void*>(this));
}

void RF24NodeNet::update(void)
{
  _network.update();
  timer.update();

  while (_network.available())
  {
    
    RF24NetworkHeader header;
    _network.peek(header);
    switch (header.type % 32)
    {
      case PKT_DIGITALPIN:
//        Serial.println("Digital packet received");
        _digitalPinHandler(header);
        break;
      case PKT_PWMPIN:
//        Serial.println("PWM packet received");
        _pwmPinHandler(header);
        break;
      case PKT_ANALOGPIN:
//        Serial.println("Analog packet received");
        _analogPinHandler(header);
        break;
      case PKT_HEARTBEAT:
//        Serial.println("Heartbeat packet received");
        _heartbeatHandler(header);
        break;
      case PKT_BITARRAY:
//        Serial.println("Bitarray packet received");
        _digitalBitArrayHandler(header);
        break;
    }
  }
}

uint8_t RF24NodeNet::_write(uint16_t toaddr, uint8_t type, const void* message)
{
  uint8_t size = 0;

  switch (type % 32)
  {
    case PKT_DIGITALPIN:
      size = sizeof(DigitalPin);
      break;
    case PKT_PWMPIN:
      size = sizeof(PwmPin);
      break;
    case PKT_ANALOGPIN:
      size = sizeof(AnalogPin);
      break;
    case PKT_HEARTBEAT:
      size = sizeof(Heartbeat);
      break;
    case PKT_BITARRAY:
      size = sizeof(DigitalBitArray);
      break;
    default:
      // No such type :-(
      return size;
  }
  RF24NetworkHeader header(toaddr, type);
  return _network.write(header, message, size);
}

void RF24NodeNet::addDigitalPinReadHandler(digitalPinReadHandler handler)
{
  _digitalPinReadHandler = handler;
}

void RF24NodeNet::addDigitalPinWriteHandler(digitalPinWriteHandler handler)
{
  _digitalPinWriteHandler = handler;
}

void RF24NodeNet::addDigitalPinRcvHandler(digitalPinRcvHandler handler)
{
  _digitalPinRcvHandler = handler;
}

void RF24NodeNet::addPwmPinReadHandler(pwmPinReadHandler handler)
{
  _pwmPinReadHandler = handler;
}

void RF24NodeNet::addPwmPinWriteHandler(pwmPinWriteHandler handler)
{
  _pwmPinWriteHandler = handler;
}

void RF24NodeNet::addPwmPinRcvHandler(pwmPinRcvHandler handler)
{
  _pwmPinRcvHandler = handler;
}

void RF24NodeNet::addAnalogPinReadHandler(analogPinReadHandler handler)
{
  _analogPinReadHandler = handler;
}

void RF24NodeNet::addAnalogPinWriteHandler(analogPinWriteHandler handler)
{
  _analogPinWriteHandler = handler;
}

void RF24NodeNet::addAnalogPinRcvHandler(analogPinRcvHandler handler)
{
  _analogPinRcvHandler = handler;
}

void RF24NodeNet::addDigitalBitArrayReadHandler(digitalBitArrayReadHandler handler)
{
  _digitalBitArrayReadHandler = handler;
}

void RF24NodeNet::addDigitalBitArrayWriteHandler(digitalBitArrayWriteHandler handler)
{
  _digitalBitArrayWriteHandler = handler;
}

void RF24NodeNet::addDigitalBitArrayRcvHandler(digitalBitArrayRcvHandler handler)
{
  _digitalBitArrayRcvHandler = handler;
}

void RF24NodeNet::addDigitalPinCallbackHandler(digitalFallbackCallback callback)
{
  _digitalFallbackCallback = callback;
}

void RF24NodeNet::addPwmPinCallbackHandler(pwmFallbackCallback callback)
{
  _pwmFallbackCallback = callback;
}

void RF24NodeNet::addAnalogPinCallbackHandler(analogFallbackCallback callback)
{
  _analogFallbackCallback = callback;
}

void RF24NodeNet::addDigitalBitArrayCallbackHandler(::digitalBitArrayFallbackCallback callback)
{
  _digitalBitArrayFallbackCallback = callback;
}
