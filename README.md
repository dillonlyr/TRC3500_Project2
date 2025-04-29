# STM project and python script for TRC3500 project 2:
Completed by: *Loo Yi Ren Dillon*

## Basic operations
- stm reads 20k ADC values, and performs simple AC analysis on the 20k samples of signal obtained, and send over to uart2 `(baud rate: 115200)`
- stm passes all raw adc values over uart2 `(baud rate: 115200)`, and python script performs advanced FFT analysis of signal.
- Classify material, Distance, and Height of drop detected.
