/**
 * @file esp32_i2c_rw.h
 *
 * @author
 * Gabriel Boni Vicari (133192@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 *
 * @copyright 2012 Jeff Rowberg
 *
 * @brief I2C Read/Write functions for ESP32 ESP-IDF.
 */

#ifndef ESP32_I2C_RW_H
#define ESP32_I2C_RW_H

#include <driver/i2c.h>

/**
 * @brief Select the register in the device where data will be read from.
 *
 * @param device_address I2C slave device address.
 * @param register_address Address of the first register to read from.
 */
void select_register(uint8_t device_address, uint8_t register_address);

/**
 * @brief Read multiple bytes from 8-bit registers.
 *
 * @param device_address I2C slave device address.
 * @param register_address Address of the first register to read from.
 * @param size Number of registers to read.
 * @param data Buffer to store the read data in.
 *
 * @return Status of read operation.
 */
int8_t esp32_i2c_read_bytes
(
    uint8_t device_address,
    uint8_t register_address,
    uint8_t size,
    uint8_t *data
);

/**
 * @brief Read single byte from an 8-bit register.
 *
 * @param device_address I2C slave device address.
 * @param register_address Address of the register to read from.
 * @param data Container to store the byte read from register.
 *
 * @return Status of read operation.
 */
int8_t esp32_i2c_read_byte
(
    uint8_t device_address,
    uint8_t register_address,
    uint8_t *data
);

/**
 * @brief Read multiple bits from an 8-bit register.
 *
 * @param device_address I2C slave device address.
 * @param register_address Address of the register to read from.
 * @param bit_start First bit position to read (0-7).
 * @param size Number of bits to read (Not more than 8).
 * @param data Container to store the right-aligned value.
 *
 * @return Status of read operation.
 */
int8_t esp32_i2c_read_bits
(
    uint8_t device_address,
    uint8_t register_address,
    uint8_t bit_start,
    uint8_t size,
    uint8_t *data
);

/**
 * @brief Read single bit from an 8-bit register.
 *
 * @param device_address I2C slave device address.
 * @param register_address Address of the register to read from.
 * @param bit_number Bit position to read (0-7).
 * @param data Container to store the bit.
 *
 * @return Status of read operation.
 */
int8_t esp32_i2c_read_bit
(
    uint8_t device_address,
    uint8_t register_address,
    uint8_t bit_number,
    uint8_t *data
);

/**
 * @brief Write multiple bytes to 8-bit registers.
 *
 * @param device_address I2C slave device address.
 * @param register_address Address of the first register to write to.
 * @param size Number of bytes to write.
 * @param data Array of bytes to write.
 *
 * @return Status of write operation.
 */
bool esp32_i2c_write_bytes
(
    uint8_t device_address,
    uint8_t register_address,
    uint8_t size,
    uint8_t *data
);

/**
 * @brief Write single byte to an 8-bit register.
 *
 * @param device_address I2C slave device address.
 * @param register_address Address of the register to write to.
 * @param dat: Array of bytes to write.
 *
 * @return Status of write operation.
 */
bool esp32_i2c_write_byte
(
    uint8_t device_address,
    uint8_t register_address,
    uint8_t data
);

/**
 * @brief Write multiple bits to an 8-bit register.
 *
 * @param device_address I2C slave device address.
 * @param register_address Address of the register to write to.
 * @param bit_star: First bit position to write (0-7).
 * @param size Number of bits to write (Not more than 8).
 * @param data Right-aligned value to write.
 *
 * @return Status of write operation.
 */
bool esp32_i2c_write_bits
(
    uint8_t device_address,
    uint8_t register_address,
    uint8_t bit_start,
    uint8_t size,
    uint8_t data
);

/**
 * @brief Write single bit to an 8-bit register.
 *
 * @param device_address I2C slave device address.
 * @param register_address Address of the register to write to.
 * @param bit_number Bit position to write (0-7).
 * @param data Bit value to write.
 *
 * @return Status of write operation.
 */
bool esp32_i2c_write_bit
(
    uint8_t device_address,
    uint8_t register_address,
    uint8_t bit_number,
    uint8_t data
);

/**
 * @brief Write word to an 8-bit register.
 *
 * @param device_address I2C slave device address.
 * @param register_address Address of the register to write to.
 * @param data Word to write.
 *
 * @return Status of write operation.
 */
int8_t esp32_i2c_write_word
(
    uint8_t device_address,
    uint8_t register_address,
    uint8_t data
);
#endif
