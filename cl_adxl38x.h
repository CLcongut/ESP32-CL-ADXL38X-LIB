#pragma once
#include <Arduino.h>
#include <SPI.h>

enum adxl38x_range {
  ADXL380_RANGE_4G = 0,
  ADXL382_RANGE_15G = 0,
  ADXL380_RANGE_8G = 1,
  ADXL382_RANGE_30G = 1,
  ADXL380_RANGE_16G = 2,
  ADXL382_RANGE_60G = 2
};

enum adxl38x_op_mode {
  ADXL38X_MODE_STDBY = 0,
  ADXL38X_MODE_HRT_SND = 1,
  ADXL38X_MODE_ULP = 2,
  ADXL38X_MODE_VLP = 3,
  ADXL38X_MODE_LP = 4,
  ADXL38X_MODE_LP_SERIAL_ULP_OP = 6,
  ADXL38X_MODE_LP_SERIAL_VLP_OP = 7,
  ADXL38X_MODE_RBW = 8,
  ADXL38X_MODE_RBW_SERIAL_ULP_OP = 10,
  ADXL38X_MODE_RBW_SERIAL_VLP_OP = 11,
  ADXL38X_MODE_HP = 12,
  ADXL38X_MODE_HP_SERIAL_ULP_OP = 14,
  ADXL38X_MODE_HP_SERIAL_VLP_OP = 15,
};

enum adxl38x_ch_select {
  ADXL38X_CH_DSB_ALL = 0,
  ADXL38X_CH_EN_X = 1,
  ADXL38X_CH_EN_Y = 2,
  ADXL38X_CH_EN_XY = 3,
  ADXL38X_CH_EN_Z = 4,
  ADXL38X_CH_EN_YZ = 6,
  ADXL38X_CH_EN_XYZ = 7,
  ADXL38X_CH_EN_T = 8,
  ADXL38X_CH_EN_ZT = 12,
  ADXL38X_CH_EN_YZT = 14,
  ADXL38X_CH_EN_XYZT = 15
};

enum adxl38x_fifo_mode {
  ADXL38X_FIFO_DISABLE = 0,
  ADXL38X_FIFO_NORMAL = 1,
  ADXL38X_FIFO_STREAM = 2,
  ADXL38X_FIFO_TRIGGER = 3
};

struct xyztRaw_Struct {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t t;
};

class CL_ADXL38X {
 public:
  static constexpr uint8_t ADXL38X_SPI_READ_FLAG = 0x01;
  static constexpr uint8_t ADXL38X_SPI_WRITE_FLAG = 0x00;
  static constexpr uint8_t ADXL38X_RESET_CODE = 'R';

  static constexpr uint8_t ADXL38X_DEVID_AD = 0x00;
  static constexpr uint8_t ADXL38X_DEVID_MST = 0x01;
  static constexpr uint8_t ADXL38X_PART_ID = 0x02;
  static constexpr uint8_t ADXL38X_STATUS0 = 0x11;
  static constexpr uint8_t ADXL38X_STATUS1 = 0x12;
  static constexpr uint8_t ADXL38X_STATUS2 = 0x13;
  static constexpr uint8_t ADXL38X_STATUS3 = 0x14;
  static constexpr uint8_t ADXL38X_FIFO_DATA = 0x1D;
  static constexpr uint8_t ADXL38X_FIFO_STATUS0 = 0x1E;
  static constexpr uint8_t ADXL38X_OP_MODE = 0x26;
  static constexpr uint8_t ADXL38X_DIG_EN = 0x27;
  static constexpr uint8_t ADXL38X_REG_RESET = 0x2A;
  static constexpr uint8_t ADXL38X_INT0_MAP0 = 0x2B;
  static constexpr uint8_t ADXL38X_INT0_MAP1 = 0x2C;
  static constexpr uint8_t ADXL38X_INT1_MAP0 = 0x2D;
  static constexpr uint8_t ADXL38X_INT1_MAP1 = 0x2E;
  static constexpr uint8_t ADXL38X_FIFO_CFG0 = 0x30;
  static constexpr uint8_t ADXL38X_FIFO_CFG1 = 0x31;
  static constexpr uint8_t ADXL38X_FILTER = 0x50;

  static constexpr uint8_t ADXL38X_RESET_DEVID_AD = 0xAD;
  static constexpr uint8_t ADXL38X_RESET_DEVID_MST = 0x1D;
  static constexpr uint8_t ADXL38X_RESET_PART_ID = 0x17;

  static constexpr uint8_t ADXL38X_MASK_RANGE = 0xC0;
  static constexpr uint8_t ADXL38X_MASK_OP_MODE = 0x0F;
  static constexpr uint8_t ADXL38X_MASK_CHEN_DIG_EN = 0xF0;
  static constexpr uint8_t ADXL38X_FIFOCFG_FIFOMODE_MSK = 0x30;

  static constexpr uint8_t ADXL38X_BIT_FIFO_WARTERMARK = 0x08;
  static constexpr uint8_t ADXL38X_BIT_FIFO_FULL = 0x02;
  static constexpr uint8_t ADXL38X_BIT_FIFO_ENABLE = 0x08;
  static constexpr uint8_t ADXL38X_BIT_FIFO_WTMK_INT0 = 0x08;

#ifndef W_DETERMINE
  static constexpr uint8_t W_FILTER_SETTING =
      0x70;  //(Bypass EQ, LPF_MODE 0b11)
#endif

  CL_ADXL38X(SPIClass *spi, uint8_t csPin, uint32_t speed = 8000000)
      : _spi{spi}, _csPin{csPin}, _spiSpeed{speed} {}

  CL_ADXL38X(uint8_t csPin, uint32_t speed = 8000000)
      : _spi{&SPI}, _csPin{csPin}, _spiSpeed{speed} {}
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);
  bool readMultipleRegisters(uint8_t reg, uint16_t size, uint8_t *read_data);
  void updateRegisterBits(uint8_t reg, uint8_t mask, uint8_t value);
  uint8_t readRegisterBits(uint8_t reg, uint8_t mask);
  bool init();
  bool softReset();
  void setRange(adxl38x_range range);
  void setOpMode(adxl38x_op_mode mode);
  void setFilter(uint8_t filter);
  void setChannel(adxl38x_ch_select ch);
  void setFIFOEnable(bool enable);
  bool setFIFO(uint16_t num_samples, bool external_trigger,
               adxl38x_fifo_mode fifo_mode, bool ch_ID_enable, bool read_reset);
  void setFIFOWaterMarkINT0();
  void clearFIFOWaterMark();
  // bool getDeviceID();
  bool getFIFOWaterMark();
  bool getFIFOFull();
  uint16_t getFIFOEntries();
  bool getFIFOData(uint8_t *fifo_data, uint16_t size);

 private:
  uint32_t _findFirstSetBit(uint32_t word);
  uint32_t _fieldPrepare(uint32_t mask, uint32_t value);

  SPIClass *_spi;
  uint8_t _csPin;
  uint32_t _spiSpeed;
  SPISettings _settings;
};
