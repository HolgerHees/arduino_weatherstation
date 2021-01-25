#ifndef MESSAGE_H
#define MESSAGE_H

#ifdef __cplusplus
#include <Arduino.h>
#include <stdint.h>
#endif

#include <core/MyMessage.h>

typedef enum {
	MP_INT8					= 1,
	MP_UINT8				= 2,
	MP_INT16				= 3,
	MP_UINT16				= 4,
	MP_INT32				= 5,
	MP_UINT32				= 6,
	MP_FLOAT				= 7
} message_payload_t;

class Message
{
    public:
        Message(const uint8_t sensorId, const mysensors_data_t dataType);
        
        uint8_t getSensor(void) const;
        uint8_t getType(void) const;
        uint8_t getPayloadType(void) const;
        
        float getFloat(void) const;
        Message& setFloat( const float value);
        int32_t getInt32(void) const;
        Message& setInt32( const int32_t value);
        uint32_t getUInt32(void) const;
        Message& setUInt32( const uint32_t value);
        int16_t getInt16(void) const;
        Message& setInt16( const int16_t value);
        uint16_t getUInt16(void) const;
        Message& setUInt16( const uint16_t value);
        int8_t getInt8(void) const;
        Message& setInt8( const int8_t value);
        uint8_t getUInt8(void) const;
        Message& setUInt8( const uint8_t value);
    private:
        uint8_t payload;

        uint8_t type;
        uint8_t sensor;

        uint8_t ubValue;
        int8_t bValue;
        uint16_t uiValue;
        int16_t iValue;
        uint32_t ulValue;
        int32_t lValue;
        float fValue;
};
#endif
