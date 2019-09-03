STM32G431 Demo generated by STM32CubeMX

In this example, the CORDIC peripheral is configured in sine function, q1.31
format for both input and output data, and with 6 cycles of precision.
The input data provided to CORDIC peripheral are angles in radians
divided by PI, in q1.31 format. The output data are sines in q1.31 format.
For 6 cycles of precision, the maximal expected residual error of the
calculated sines is 2^-19.

DMA is used to transfer input data from memory to the CORDIC peripheral and
output data from CORDIC peripheral to memory, so that CPU is offloaded.

The calculated sines are stored in aCalculatedSin[] array.
The residual error of calculation results is verified, by comparing to
reference values in aRefSin[] obtained from double precision floating point
calculation.

STM32 board LED is used to monitor the example status:
  - LED2 is ON when correct CORDIC sine results are calculated.
  - LED2 is blinking (1 second period) when exceeding residual error
    on CORDIC sine results is detected or when there is an initialization error.
	