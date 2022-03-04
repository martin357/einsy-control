#pragma once

#define AD7150_I2C_ADDRESS                  0x48

#define AD7150_REG_STATUS                   0x00
#define AD7150_REG_CH1_DATA_HIGH            0x01
#define AD7150_REG_CH1_DATA_LOW             0x02
#define AD7150_REG_CH2_DATA_HIGH            0x03
#define AD7150_REG_CH2_DATA_LOW             0x04
#define AD7150_REG_CH1_AVERAGE_HIGH         0x05
#define AD7150_REG_CH1_AVERAGE_LOW          0x06
#define AD7150_REG_CH2_AVERAGE_HIGH         0x07
#define AD7150_REG_CH2_AVERAGE_LOW          0x08
#define AD7150_REG_CH1_SENSITIVITY          0x09
#define AD7150_REG_CH1_TIMEOUT              0x0A
#define AD7150_REG_CH1_SETUP                0x0B
#define AD7150_REG_CH2_SENSITIVITY          0x0C
#define AD7150_REG_CH2_TIMEOUT              0x0D
#define AD7150_REG_CH2_SETUP                0x0E
#define AD7150_REG_CONFIGURATION            0x0F
#define AD7150_REG_POWER_DOWN_TIMER         0x10
#define AD7150_REG_CH1_CAPDAC               0x11
#define AD7150_REG_CH2_CAPDAC               0x12
#define AD7150_REG_SERIAL_NUMBER_3          0x13
#define AD7150_REG_SERIAL_NUMBER_2          0x14
#define AD7150_REG_SERIAL_NUMBER_1          0x15
#define AD7150_REG_SERIAL_NUMBER_0          0x16
#define AD7150_REG_CHIP_ID                  0x17

//Values for the Capdac Register (0x11)
#define AD7150_OFFSET_DAC_DISABLED          0x00
#define AD7150_OFFSET_DAC_EN_AUTO_FULL      0xFF // 0x80
#define AD7150_OFFSET_DAC_EN_AUTO_HALF      0xDF // 0x80
#define AD7150_OFFSET_DAC_EN_MANU_0         0x81  //1
#define AD7150_OFFSET_DAC_EN_MANU_1         0x88  //8
#define AD7150_OFFSET_DAC_EN_MANU_2         0x8F  //15
#define AD7150_OFFSET_DAC_EN_MANU_3         0x97  //23
#define AD7150_OFFSET_DAC_EN_MANU_4         0x9E  //30
#define AD7150_OFFSET_DAC_EN_MANU_5         0xA7  //39
#define AD7150_OFFSET_DAC_EN_MANU_6         0xAE  //46
#define AD7150_OFFSET_DAC_EN_MANU_7         0xB6  //54
#define AD7150_OFFSET_DAC_EN_MANU_8         0xBF  //63

//Values for the Setup Register (0x0B)
#define AD7150_CAPACTIVE_INPUT_RANGE_2      0x0B
#define AD7150_CAPACTIVE_INPUT_RANGE_0_5    0x4B
#define AD7150_CAPACTIVE_INPUT_RANGE_1      0x8B
#define AD7150_CAPACTIVE_INPUT_RANGE_4      0xCB
