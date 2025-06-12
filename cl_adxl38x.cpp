#include "cl_adxl38x.h"

uint8_t CL_ADXL38X::readRegister(uint8_t reg) {
  uint8_t regValue = 0;
  reg = (reg << 1) | ADXL38X_SPI_READ_FLAG;
  _spi->beginTransaction(_settings);
  digitalWrite(_csPin, LOW);
  _spi->transfer(reg);
  regValue = _spi->transfer(0x00);
  digitalWrite(_csPin, HIGH);
  _spi->endTransaction();
  return regValue;
}

void CL_ADXL38X::writeRegister(uint8_t reg, uint8_t value) {
  reg = (reg << 1) | ADXL38X_SPI_WRITE_FLAG;
  _spi->beginTransaction(_settings);
  digitalWrite(_csPin, LOW);
  _spi->transfer(reg);
  _spi->transfer(value);
  digitalWrite(_csPin, HIGH);
  _spi->endTransaction();
}

bool CL_ADXL38X::readMultipleRegisters(uint8_t reg, uint16_t size,
                                       uint8_t* read_data) {
  reg = (reg << 1) | ADXL38X_SPI_READ_FLAG;
  _spi->beginTransaction(_settings);
  digitalWrite(_csPin, LOW);
  _spi->transfer(reg);
  for (size_t i = 0; i < size; i++) {
    read_data[i] = _spi->transfer(0x00);
  }
  digitalWrite(_csPin, HIGH);
  _spi->endTransaction();
  return true;
}

bool CL_ADXL38X::updateRegisterBits(uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t data = 0;
  data = readRegister(reg);
  data &= ~mask;
  data |= _fieldPrepare(mask, value);
  writeRegister(reg, data);
  return true;
}

uint8_t CL_ADXL38X::readRegisterBits(uint8_t reg, uint8_t mask) {
  return (readRegister(reg) & mask) >> _findFirstSetBit(mask);
}

bool CL_ADXL38X::init() {
  _settings = SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0);
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  _spi->begin();
#ifdef USE_CUSTOM_PIN
  _spi->begin(_sckPin, _misoPin, _mosiPin, _csPin);
#endif

  if (readRegister(ADXL38X_DEVID_AD) != ADXL38X_RESET_DEVID_AD) {
    return false;
  }

  if (readRegister(ADXL38X_DEVID_MST) != ADXL38X_RESET_DEVID_MST) {
    return false;
  }

  if (readRegister(ADXL38X_PART_ID) != ADXL38X_RESET_PART_ID) {
    return false;
  }

  return true;
}

bool CL_ADXL38X::softReset() {
  writeRegister(ADXL38X_REG_RESET, ADXL38X_RESET_CODE);
  delay(1);
  if (readRegister(ADXL38X_DEVID_AD) != ADXL38X_RESET_DEVID_AD) {
    return false;
  } else {
    return true;
  }
}

bool CL_ADXL38X::setRange(adxl38x_range range) {
  updateRegisterBits(ADXL38X_OP_MODE, ADXL38X_MASK_RANGE, range);
  return true;
}

bool CL_ADXL38X::setOpMode(adxl38x_op_mode mode) {
  updateRegisterBits(ADXL38X_OP_MODE, ADXL38X_MASK_OP_MODE, mode);
  delay(2);
  return true;
}

bool CL_ADXL38X::setFilter(uint8_t filter) {
  writeRegister(ADXL38X_FILTER, filter);
  return true;
}

bool CL_ADXL38X::setChannel(adxl38x_ch_select ch) {
  updateRegisterBits(ADXL38X_DIG_EN, ADXL38X_MASK_CHEN_DIG_EN, ch);
  return true;
}

bool CL_ADXL38X::setFIFOEnable(bool enable) {
  updateRegisterBits(ADXL38X_DIG_EN, ADXL38X_BIT_FIFO_ENABLE, enable);
  return true;
}

bool CL_ADXL38X::setFIFO(uint16_t num_samples, bool external_trigger,
                         adxl38x_fifo_mode fifo_mode, bool ch_ID_enable,
                         bool read_reset) {
  uint8_t fifo_cfg0_data;
  uint8_t set_channels;
  uint8_t num_samples_low;
  uint8_t num_samples_high;
  set_channels = readRegisterBits(ADXL38X_DIG_EN, ADXL38X_MASK_CHEN_DIG_EN);

  if (num_samples > 320) {
    return false;
  } else if ((num_samples > 318) &&
             ((!set_channels) || (set_channels == ADXL38X_CH_EN_XYZ) ||
              (set_channels == ADXL38X_CH_EN_YZT))) {
    return false;
  }

  num_samples_low = (uint8_t)(num_samples & 0x00FF);
  writeRegister(ADXL38X_FIFO_CFG1, num_samples_low);

  num_samples_high = (uint8_t)((num_samples >> 8) & 0x01);
  fifo_cfg0_data |= num_samples_high;
  fifo_cfg0_data |= _fieldPrepare(ADXL38X_FIFOCFG_FIFOMODE_MSK, fifo_mode);

  if (read_reset) {
    fifo_cfg0_data |= 1 << 7;
  }

  if (ch_ID_enable) {
    fifo_cfg0_data |= 1 << 6;
  }

  if (external_trigger && fifo_mode == ADXL38X_FIFO_TRIGGER) {
    fifo_cfg0_data |= 1 << 3;
  }

  writeRegister(ADXL38X_FIFO_CFG0, fifo_cfg0_data);

  return true;
}

bool CL_ADXL38X::getFIFOWaterMark() {
  return readRegisterBits(ADXL38X_STATUS0, ADXL38X_BIT_FIFO_WARTERMARK);
}

bool CL_ADXL38X::getFIFOFull() {
  return readRegisterBits(ADXL38X_STATUS0, ADXL38X_BIT_FIFO_FULL);
}

uint16_t CL_ADXL38X::getFIFOEntries() {
  uint8_t buf[2];
  uint16_t fifo_entries;
  readMultipleRegisters(ADXL38X_FIFO_STATUS0, 2, buf);
//   Serial.println(buf[0], HEX);
//   Serial.println(buf[1], HEX);
  fifo_entries = buf[0] | (buf[1] << 8);
  fifo_entries &= 0x01FF;
  return fifo_entries;
}

bool CL_ADXL38X::getFIFOData(uint8_t* fifo_data, uint16_t size) {
  readMultipleRegisters(ADXL38X_FIFO_DATA, size, fifo_data);
  return true;
}

uint32_t CL_ADXL38X::_findFirstSetBit(uint32_t word) {
  uint32_t first_set_bit = 0;
  while (word) {
    if (word & 0x1) return first_set_bit;
    word >>= 1;
    first_set_bit++;
  }
  return 32;
}

uint32_t CL_ADXL38X::_fieldPrepare(uint32_t mask, uint32_t value) {
  return (value << _findFirstSetBit(mask)) & mask;
}
