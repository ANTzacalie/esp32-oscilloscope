<div align="center">

# ESP32 Oscilloscope

**Generate a signal. Capture it. Watch it take shape.**

![Platform: ESP32](https://img.shields.io/badge/PLATFORM-ESP32-1769AA?style=flat-square) ![Language: C++](https://img.shields.io/badge/LANGUAGE-C%2B%2B-1769AA?style=flat-square) ![Framework: ESP-IDF](https://img.shields.io/badge/FRAMEWORK-ESP--IDF-329344?style=flat-square) ![Software: MATLAB](https://img.shields.io/badge/SOFTWARE-MATLAB-1769AA?style=flat-square) ![Interface: UART](https://img.shields.io/badge/INTERFACE-UART-329344?style=flat-square) ![Topic: IoT](https://img.shields.io/badge/TOPIC-IoT-1769AA?style=flat-square)

A two-ESP32 university project made to **see** sine and square waves in MATLAB.

</div>

## ▶ Demos

### 01 · Square wave

https://github.com/user-attachments/assets/9007af29-f050-4348-b50f-5cad1f0ace6f

<!-- SS2_Demo_1.mp4: paste its GitHub-hosted attachment URL on a line by itself here. -->

### 02 · Sine wave

https://github.com/user-attachments/assets/41accc2e-61ae-45bc-a20c-1565d1ee1906

<!-- SS2_Demo_2.mp4: paste its GitHub-hosted attachment URL on a line by itself here. -->

---

## The idea

| Stage | What happens |
| :--- | :--- |
| **01 · Generate** | The first ESP32 produces a sine wave with its DAC or a square wave with PWM. Buttons select the wave and set its frequency; a potentiometer selects the sine-wave attenuation. |
| **02 · Capture** | The second ESP32 samples the signal with its ADC and streams frames of 256 values over USB serial. |
| **03 · Visualize** | MATLAB draws the waveform and FFT spectrum, estimates the dominant frequency, and displays Vpp and Vrms. |

> Built for signal visualization and learning, rather than calibrated measurements.

## Try it

1. Flash `esp32-dac` to the **generator** and `esp32-osciloscope` to the **analyzer** as separate ESP-IDF applications using PlatformIO;
2. Connect the generator's **GPIO25 (sine)** or **GPIO12 (square)** to the analyzer's **GPIO35**. Connect the boards' grounds, and keep the ADC input within 0–3.3 V.
3. In `gui_matlab`, set `COM_PORT` to the analyzer's serial port, then run the script in MATLAB. The serial baud rate is **115200**.

**Generator controls:** With GPIO13 released, GPIO32 selects sine and GPIO27 selects square. With GPIO13 pressed, GPIO27 increases frequency and GPIO32 decreases it in 1 kHz steps. The potentiometer connects to the generator's GPIO35.

---

<div align="center">

**Mihalcea Catalin Antonio · Mitrache Cristian Mario**

</div>
