
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

void clearBits(uint8_t* target_byte, uint8_t* bits_to_clear, const uint8_t startbit, const uint8_t length) {
    for (uint8_t i = startbit; i < length + startbit; ++i) {
        *target_byte &= ~(1UL << i);
        *bits_to_clear -= 1;
    }
}

void storeSignal(uint8_t* frame, uint64_t value, const uint8_t startbit, const uint8_t length, bool bigEndian, bool isSigned) {
    uint8_t start_byte = startbit / 8;
    uint8_t startbit_in_byte = startbit % 8;
    uint8_t end_byte = 0;
    int8_t count = 0;
    uint8_t current_target_length = (8 - startbit_in_byte);
    uint8_t bits_to_clear = length;

    // Mask the value
    value &= MASK64(length);

    // Write bits of startbyte
    clearBits(&frame[start_byte], &bits_to_clear, startbit_in_byte, current_target_length > length ? length : current_target_length);
    frame[start_byte] |= value << startbit_in_byte;

    // Write residual bytes
    if (bigEndian) { // Motorola (big endian
        end_byte = (start_byte * 8 + 8 - startbit_in_byte - length) / 8;
        for (count = start_byte - 1; count >= end_byte; count--) {
            clearBits(&frame[count], &bits_to_clear, 0, bits_to_clear >= 8 ? 8 : bits_to_clear);
            frame[count] |= value >> current_target_length;
            current_target_length += 8;
        }

    } else { // Intel (little endian)
        end_byte = (startbit + length - 1) / 8;
        for (count = start_byte + 1; count <= end_byte; count++) {
            clearBits(&frame[count], &bits_to_clear, 0, bits_to_clear >= 8 ? 8 : bits_to_clear);
            frame[count] |= value >> current_target_length;
            current_target_length += 8;
        }
    }
}

uint64_t extractSignal(const uint8_t* frame, const uint8_t startbit, const uint8_t length, bool bigEndian, bool isSigned) {
    uint8_t start_byte = startbit / 8;
    uint8_t startbit_in_byte = startbit % 8;
    uint8_t current_target_length = (8 - startbit_in_byte);
    uint8_t end_byte = 0;
    int8_t count = 0;

    // Write first bits to target
    uint64_t target = frame[start_byte] >> startbit_in_byte;

    // Write residual bytes
    if (bigEndian) {  // Motorola (big endian)
        end_byte = (start_byte * 8 + 8 - startbit_in_byte - length) / 8;
        for (count = start_byte - 1; count >= end_byte; count--) {
            target |= frame[count] << current_target_length;
            current_target_length += 8;
        }

    } else { // Intel (little endian)
        end_byte = (startbit + length - 1) / 8;
        for (count = start_byte + 1; count <= end_byte; count++) {
            target |= frame[count] << current_target_length;
            current_target_length += 8;
        }
    }

    // Mask value
    target &= MASK64(length);

    // perform sign extension
    if (isSigned) {
        int64_t msb_sign_mask = 1 << (length - 1);
        target = ((int64_t)target ^ msb_sign_mask) - msb_sign_mask;
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
