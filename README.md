# STM32G03x_DMA

STM32G03x BQ25672 I2C DMA Driver
Driver for the BQ25672 battery charger IC on STM32G030 using I2C with DMA. Features non-blocking I2C communication, error handling for bus errors, arbitration loss, acknowledge failures, overrun/underrun, and DMA errors, with global status flags (i2c_tx_complete, i2c_rx_complete, i2c_error).