#ifndef CAN_CODEC_H
#define CAN_CODEC_H

#include <Arduino.h>

inline float normalizeSignalValue(uint64_t target, float factor, float offset, bool isSigned);

inline uint64_t denormalizeSignalValue(float physical_value, float factor, float offset);

void clearBits(uint8_t* target_byte, uint8_t* bits_to_clear, const uint8_t startbit, const uint8_t length);

void storeSignal(uint8_t* frame, uint64_t value, const uint8_t startbit, const uint8_t length, bool bigEndian, bool isSigned);

uint64_t extractSignal(const uint8_t* frame, const uint8_t startbit, const uint8_t length, bool bigEndian, bool isSigned);

// For Vector CAN DB files https://vector.com/vi_candb_en.html

float decodeSignal(const uint8_t* frame, const uint16_t startbit, const uint16_t length, bool bigEndian, bool isSigned, float factor, float offset);

void encodeSignal(uint8_t* frame, const float value, const uint16_t startbit, const uint16_t length, bool bigEndian, bool isSigned, float factor, float offset);

// Texas instruments IQ notation https://en.wikipedia.org/wiki/Q_(number_format)

double extractIQ(const uint8_t* frame, uint8_t start, uint8_t length, uint8_t float_length, bool bigEndian, bool isSigned);

void storeIQ(uint8_t* frame, double value, uint8_t start, uint8_t length, uint8_t float_length, bool bigEndian, bool isSigned);

#endif