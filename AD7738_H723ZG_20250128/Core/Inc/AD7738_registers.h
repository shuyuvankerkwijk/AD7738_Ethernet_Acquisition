/*
 * AD7738_registers.h
 *
 *  Created on: Jan 7, 2025
 *      Author: shuyu
 */

#ifndef INC_AD7738_REGISTERS_H_
#define INC_AD7738_REGISTERS_H_

// Register Definitions
#define AD7734_REG_COMM           0x00 // Communications Register (W, 8-bit)
#define AD7738_REG_IO             0x01 // I/P Port Register (RW, 8-bit)
#define AD7738_REG_REV            0x02 // Revision Register (R, 8-bit)
#define AD7738_REG_TEST           0x03 // Test Register (RW, 24-bit)
#define AD7738_REG_STATUS         0x04 // ADC Status Register (8-bit)
#define AD7738_REG_CKS            0x05 // Checksum Register (RW, 16-bit)
#define ADD738_REG_ZSCAL          0x06 // ADC Zero-Scale Calibration Register (24-bit)
#define AD7738_REG_FS             0x07 // ADC Full Scale Calibration Register (25-bit)
#define AD7738_REG_CH1_DATA       0x08 // CH1 Data Register (RW, 24-bit)
#define AD7738_REG_CH2_DATA       0x09 // CH2 Data Register (RW, 24-bit)
#define AD7738_REG_CH3_DATA       0x0A // CH3 Data Register (RW, 24-bit)
#define AD7738_REG_CH4_DATA       0x0B // CH4 Data Register (RW, 24-bit)
#define AD7738_REG_CH5_DATA       0x0C // CH5 Data Register (RW, 24-bit)
#define AD7738_REG_CH6_DATA       0x0D // CH6 Data Register (RW, 24-bit)
#define AD7738_REG_CH7_DATA       0x0E // CH7 Data Register (RW, 24-bit)
#define AD7738_REG_CH8_DATA       0x0F // CH8 Data Register (RW, 24-bit)
#define AD7738_REG_CH1_ZSCAL      0x10 // CH1 Zero-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH2_ZSCAL      0x11 // CH2 Zero-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH3_ZSCAL      0x12 // CH3 Zero-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH4_ZSCAL      0x13 // CH4 Zero-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH5_ZSCAL      0x14 // CH5 Zero-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH6_ZSCAL      0x15 // CH6 Zero-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH7_ZSCAL      0x16 // CH7 Zero-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH8_ZSCAL      0x17 // CH8 Zero-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH1_FSCAL      0x18 // CH1 Full-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH2_FSCAL      0x19 // CH2 Full-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH3_FSCAL      0x1A // CH3 Full-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH4_FSCAL      0x1B // CH4 Full-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH5_FSCAL      0x1C // CH5 Full-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH6_FSCAL      0x1D // CH6 Full-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH7_FSCAL      0x1E // CH7 Full-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH8_FSCAL      0x1F // CH8 Full-Scale Calibration Register (RW, 24-bit)
#define AD7738_REG_CH1_STAT       0x20 // CH1 Channel Status (R, 8-bit)
#define AD7738_REG_CH2_STAT       0x21 // CH2 Channel Status (R, 8-bit)
#define AD7738_REG_CH3_STAT       0x22 // CH3 Channel Status (R, 8-bit)
#define AD7738_REG_CH4_STAT       0x23 // CH4 Channel Status (R, 8-bit)
#define AD7738_REG_CH5_STAT       0x24 // CH5 Channel Status (R, 8-bit)
#define AD7738_REG_CH6_STAT       0x25 // CH6 Channel Status (R, 8-bit)
#define AD7738_REG_CH7_STAT       0x26 // CH7 Channel Status (R, 8-bit)
#define AD7738_REG_CH8_STAT       0x27 // CH8 Channel Status (R, 8-bit)
#define AD7738_REG_CH1_SETUP      0x28 // CH1 Channel Setup (RW, 8-bit)
#define AD7738_REG_CH2_SETUP      0x29 // CH2 Channel Setup (RW, 8-bit)
#define AD7738_REG_CH3_SETUP      0x2A // CH3 Channel Setup (RW, 8-bit)
#define AD7738_REG_CH4_SETUP      0x2B // CH4 Channel Setup (RW, 8-bit)
#define AD7738_REG_CH5_SETUP      0x2C // CH5 Channel Setup (RW, 8-bit)
#define AD7738_REG_CH6_SETUP      0x2D // CH6 Channel Setup (RW, 8-bit)
#define AD7738_REG_CH7_SETUP      0x2E // CH7 Channel Setup (RW, 8-bit)
#define AD7738_REG_CH8_SETUP      0x2F // CH8 Channel Setup (RW, 8-bit)
#define AD7738_REG_CH1_CONVTIME   0x30 // CH1 Channel Conversion Time (RW, 8-bit)
#define AD7738_REG_CH2_CONVTIME   0x31 // CH2 Channel Conversion Time (RW, 8-bit)
#define AD7738_REG_CH3_CONVTIME   0x32 // CH3 Channel Conversion Time (RW, 8-bit)
#define AD7738_REG_CH4_CONVTIME   0x33 // CH4 Channel Conversion Time (RW, 8-bit)
#define AD7738_REG_CH5_CONVTIME   0x34 // CH5 Channel Conversion Time (RW, 8-bit)
#define AD7738_REG_CH6_CONVTIME   0x35 // CH6 Channel Conversion Time (RW, 8-bit)
#define AD7738_REG_CH7_CONVTIME   0x36 // CH7 Channel Conversion Time (RW, 8-bit)
#define AD7738_REG_CH8_CONVTIME   0x37 // CH8 Channel Conversion Time (RW, 8-bit)
#define AD7738_REG_MODE_READ      0x38 // Mode Register Read (RW, 8-bit)
#define AD7738_REG_CH1_MODE       0x38 // CH1 Mode Register Write(RW, 8-bit)
#define AD7738_REG_CH2_MODE       0x39 // CH2 Mode Register Write(RW, 8-bit)
#define AD7738_REG_CH3_MODE       0x3A // CH3 Mode Register Write(RW, 8-bit)
#define AD7738_REG_CH4_MODE       0x3B // CH4 Mode Register Write(RW, 8-bit)
#define AD7738_REG_CH5_MODE       0x3C // CH5 Mode Register Write(RW, 8-bit)
#define AD7738_REG_CH6_MODE       0x3D // CH6 Mode Register Write(RW, 8-bit)
#define AD7738_REG_CH7_MODE       0x3E // CH7 Mode Register Write(RW, 8-bit)
#define AD7738_REG_CH8_MODE       0x3F // CH8 Mode Register Write(RW, 8-bit)

// Operation Modes
#define AD7738_COMM_WRITE         0b00000000 // Write Operation
#define AD7738_COMM_READ          0b01000000 // Read Operation

// Specific Modes
#define AD7738_CH_IO_MODE1        0b11111000 // P0 pin, P1 pin, 1, 1, RDY FN (1 = RDY goes low after all conversions), 0, 0, SYNC (0 = default)
#define AD7738_CH_IO_MODE2        0b11110000 // P0 pin, P1 pin, 1, 1, RDY FN (1 = RDY goes low after each conversion), 0, 0, SYNC (0 = default)

#define AD7738_CH_SETUP_MODE1     0b00001100 // BUF OFF (0 = default), COM1, COM0 (00 = AINx - AINCOM), Stat. Opt (0 = default), ENABLE (1 = enable CH in cont. conversion mode), RNG2, RNG1, RNG0 (100 = +- 2.5V)
#define AD7738_CH_SETUP_MODE2     0b00001000 // BUF OFF (0 = default), COM1, COM0 (00 = AINx - AINCOM), Stat. Opt (0 = default), ENABLE (1 = enable CH in cont. conversion mode), RNG2, RNG1, RNG0 (000 = +- 1.25V)

#define AD7738_CH_CONVTIME_MODE1  0b10010001 // CHOP (1 = default), FW 7-bit (0010001 = 0x11 = standard conversion time of 395us = 2532 Hz)
#define AD7738_CH_CONVTIME_MODE2  0b10000100 // CHOP (1 = default), FW 7-bit (10000100 = 0x84 = standard conversion time of 124us = 8074 Hz)
#define AD7738_CH_CONVTIME_MODE3  0b00001000 // CHOP = 0, 0001000 = 0x8

#define AD7738_CH_MODE_MODE1      0b00101100 // MD2, MD1, MD0 (001 = cont. conversion mode), CLKDIS (0 = default), DUMP (1 = always dump status data), CONT RD (1 = continuous read mode), 24/16 bit (0 = 16 bit), CLAMP (0 = default, don't clamp voltage data)
#define AD7738_CH_MODE_MODE2 	  0b00101001 // MD2, MD1, MD0 (001 = cont. conversion mode), CLKDIS (0 = default), DUMP (1 = always dump status data), CONT RD (0 = no continuous read mode), 24/16 bit (0 = 16 bit), CLAMP (1 = clamp voltage data)
#define AD7738_CH_MODE_MODE3      0b00101101 // MD2, MD1, MD0 (001 = cont. conversion mode), CLKDIS (0 = default), DUMP (1 = always dump status data), CONT RD (1 = continuous read mode), 24/16 bit (0 = 16 bit), CLAMP (1 = clamp voltage data)
#define AD7738_CH_MODE_MODE4	  0b00101011 // MD2, MD1, MD0 (001 = cont. conversion mode), CLKDIS (0 = default), DUMP (1 = always dump status data), CONT RD (0 = no continuous read mode), 24/16 bit (1 = 24 bit), CLAMP (1 = clamp voltage data)


#endif /* INC_AD7738_REGISTERS_H_ */
