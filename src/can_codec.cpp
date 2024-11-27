
#include "can_codec.h"

#define MASK64(nbits) ((0xffffffffffffffff) >> (64 - nbits))

inline float normalizeSignalValue(uint64_t target, float factor, float offset, bool isSigned) {
    return (isSigned)
           ? ((int64_t)target) * factor + offset
           : target * factor + offset;
}

inline uint64_t denormalizeSignalValue(float physical_value, float factor, float offset) {
    return (int64_t)((physical_value - offset) / factor);
}

void clearBits(uint8_t* target_byte, const uint8_t startbit, const uint8_t length) {
  uint8_t endbit = startbit + length;
  if (endbit > 8) endbit = 8;
  for (uint8_t i = startbit; i < endbit; ++i) {
    *target_byte &= ~(1UL << i);
  }
}

void storeSignal(uint8_t* frame, uint64_t value, uint8_t startbit, uint8_t length, bool bigEndian, bool isSigned) {
  if (length == 0 || length > 64) return; // Validate length

  // Mask the value to the correct length
  value &= MASK64(length);

  if (bigEndian) {
    // Big Endian (Motorola)
    uint16_t bit_pos = startbit;
    while (length > 0) {
      uint8_t byte_index = bit_pos / 8;
      uint8_t bit_in_byte = 7 - (bit_pos % 8);
      uint8_t bits_in_this_byte = (bit_in_byte + 1 < length) ? bit_in_byte + 1 : length;
      uint8_t shift_amount = bit_in_byte - bits_in_this_byte + 1;
      clearBits(&frame[byte_index], shift_amount, bits_in_this_byte);
      frame[byte_index] |= ((value >> (length - bits_in_this_byte)) & ((1 << bits_in_this_byte) - 1)) << shift_amount;
      length -= bits_in_this_byte;
      bit_pos += bits_in_this_byte;
    }
  } else {
    // Little Endian (Intel)
    uint16_t bit_pos = startbit;
    uint8_t current_bit = 0;
    while (length > 0) {
      uint8_t byte_index = bit_pos / 8;
      uint8_t bit_in_byte = bit_pos % 8;
      uint8_t bits_in_this_byte = (8 - bit_in_byte < length) ? 8 - bit_in_byte : length;
      clearBits(&frame[byte_index], bit_in_byte, bits_in_this_byte);
      frame[byte_index] |= ((value >> current_bit) & ((1ULL << bits_in_this_byte) - 1)) << bit_in_byte;
      length -= bits_in_this_byte;
      current_bit += bits_in_this_byte;
      bit_pos += bits_in_this_byte;
    }
  }
}

uint64_t extractSignal(const uint8_t* frame, const uint8_t startbit, const uint8_t length, bool bigEndian, bool isSigned) {
  if (length == 0 || length > 64) return 0; // Validate length

  uint64_t target = 0;

  if (bigEndian) {
    // Big Endian (Motorola)
    uint16_t bit_pos = startbit;
    uint8_t bits_remaining = length;
    while (bits_remaining > 0) {
      uint8_t byte_index = bit_pos / 8;
      uint8_t bit_in_byte = 7 - (bit_pos % 8);
      uint8_t bits_in_this_byte = (bit_in_byte + 1 < bits_remaining) ? bit_in_byte + 1 : bits_remaining;
      uint8_t shift_amount = bits_remaining - bits_in_this_byte;
      uint8_t mask = ((1 << bits_in_this_byte) - 1);
      uint8_t data = (frame[byte_index] >> (bit_in_byte - bits_in_this_byte + 1)) & mask;
      target |= ((uint64_t)data) << shift_amount;
      bits_remaining -= bits_in_this_byte;
      bit_pos += bits_in_this_byte;
    }
  } else {
    // Little Endian (Intel)
    uint16_t bit_pos = startbit;
    uint8_t bits_remaining = length;
    uint8_t current_bit = 0;
    while (bits_remaining > 0) {
      uint8_t byte_index = bit_pos / 8;
      uint8_t bit_in_byte = bit_pos % 8;
      uint8_t bits_in_this_byte = (8 - bit_in_byte < bits_remaining) ? 8 - bit_in_byte : bits_remaining;
      uint8_t mask = ((1 << bits_in_this_byte) - 1);
      uint8_t data = (frame[byte_index] >> bit_in_byte) & mask;
      target |= ((uint64_t)data) << current_bit;
      bits_remaining -= bits_in_this_byte;
      current_bit += bits_in_this_byte;
      bit_pos += bits_in_this_byte;
    }
  }

  // Sign-extend if necessary
  if (isSigned) {
    uint64_t sign_bit = 1ULL << (length - 1);
    if (target & sign_bit) {
      target |= (~0ULL) << length;
    }
  }

  return target;
}

// For Vector CAN DB files https://vector.com/vi_candb_en.html

float decodeSignal(const uint8_t* frame, const uint16_t startbit, const uint16_t length, bool bigEndian, bool isSigned, float factor, float offset) {
    return normalizeSignalValue(extractSignal(frame, startbit, length, bigEndian, isSigned), factor, offset, isSigned);
}

void encodeSignal(uint8_t* frame, const float value, const uint16_t startbit, const uint16_t length, bool bigEndian, bool isSigned, float factor, float offset) {
    storeSignal(frame, denormalizeSignalValue(value, factor, offset), startbit, length, bigEndian, isSigned);
}

// Texas instruments IQ notation https://en.wikipedia.org/wiki/Q_(number_format)

double extractIQ(const uint8_t* frame, uint8_t start, uint8_t length, uint8_t float_length, bool bigEndian, bool isSigned) {
    return (int64_t) extractSignal(frame, start, length, bigEndian, isSigned) / std::pow(2, float_length);
}

void storeIQ(uint8_t* frame, double value, uint8_t start, uint8_t length, uint8_t float_length, bool bigEndian, bool isSigned) {
    storeSignal(frame, value * std::pow(2, float_length), start, length, bigEndian, isSigned);
}
