# AD7738_Ethernet_Acquisition

Overview: Firmware for STM32H723ZG microcontroller to read data from AD7738 sigma-delta ADC board and transmit over Ethernet to computer

Channels: CH1-CH6

Data size: 24 bit

Range: Currently +-1.25V. Change 'AD7738_CH_SETUP_MODE2' to 'AD7738_CH_SETUP_MODE1' to set it to +-2.5V

CHOP: Currently disabled. Change 'AD7738_CH_CONVTIME_MODE3' to 'AD7738_CH_CONVTIME_MODE2' to enable CHOP
