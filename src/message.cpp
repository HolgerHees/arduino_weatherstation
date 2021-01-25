#include "message.h"

#include <core/MyMessage.h>

Message::Message(const uint8_t _sensorId, const mysensors_data_t _dataType)
{
  this->sensor = _sensorId;
  this->type = static_cast<uint8_t>(_dataType);
}

uint8_t Message::getSensor(void) const
{
	return this->sensor;
}

uint8_t Message::getType(void) const
{
    return this->type;
}

uint8_t Message::getPayloadType(void) const
{
    return this->payload;
}

float Message::getFloat(void) const
{
    return this->fValue;
}

Message& Message::setFloat( const float value)
{
    this->fValue = value;
    this->payload = static_cast<uint8_t>(MP_FLOAT);
    return *this;
}

int32_t Message::getInt32(void) const
{
    return this->lValue;
}

Message& Message::setInt32( const int32_t value)
{
    this->lValue = value;
    this->payload = static_cast<uint8_t>(MP_INT32);
    return *this;
}

uint32_t Message::getUInt32(void) const
{
	return this->ulValue;
}

Message& Message::setUInt32( const uint32_t value)
{
    this->ulValue = value;
    this->payload = static_cast<uint8_t>(MP_UINT32);
    return *this;
}

int16_t Message::getInt16(void) const
{
    return this->iValue;
}

Message& Message::setInt16( const int16_t value)
{
    this->iValue = value;
    this->payload = static_cast<uint8_t>(MP_INT16);
    return *this;
}

uint16_t Message::getUInt16(void) const
{
	return this->uiValue;
}

Message& Message::setUInt16( const uint16_t value)
{
    this->uiValue = value;
    this->payload = static_cast<uint8_t>(MP_INT32);
    return *this;
}

int8_t Message::getInt8(void) const
{
    return this->bValue;
}

Message& Message::setInt8( const int8_t value)
{
    this->bValue = value;
    this->payload = static_cast<uint8_t>(MP_INT8);
    return *this;
}

uint8_t Message::getUInt8(void) const
{
	return this->ubValue;
}

Message& Message::setUInt8( const uint8_t value)
{
    this->ubValue = value;
    this->payload = static_cast<uint8_t>(MP_UINT8);
    return *this;
}
