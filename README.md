# Vega
## ESP32 clone of the uSDX project

As a part of radio amateur transceiver whith quadrature geterodin, mixer with 0 Hz IF, AF amplifiers for I and Q channels, this part contains two 12 bit ADC for Audio_I and Audio_Q, one 12 bit DAC output using two 8 bit DACs, one for low byte and second DAC for high 4 bits of 12 bit ADC samples.

This 4 bit shifted right in 4 bits and outputs of DACs summed with 1:16 scala. This 12 bit DAC output used for AF output for loudspeaker at RX mode or for modulator at TX mode. PTT button used to change RX to TX mode, two LEDs: Green for RX mode and Red fot TX mode. At TX mode ADC inputs connects to microphone and SWR meter by change ULP core codes. ADC and DAC controlled by ULP core at 40000 samples per secund. After 128 samples ULP generate log 1 at INT signal (GPIO4), Core0 takes this interrupt signal at GPIO5 and makes DSP work at this samples array. After 256 samples ULP generate log 0 at INT signal (GPIO4), Core0 takes this interrupt signal at GPIO5 and makes DSP work at the next samples array. All time DSP fill DAC output array for continous 40000 samples per secund sound at loudspeaker. Interrupt frequence = 154 HZ.
