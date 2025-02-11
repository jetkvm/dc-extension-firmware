<div align="center">
    <img alt="JetKVM DC logo" src="https://jetkvm.com/dc-logo.png" height="48">

### DC Power Extension Firware for JetKVM

[Discord](https://discord.gg/pWpGMMEBCK) | [Website](https://jetkvm.com) | [Issues](https://github.com/jetkvm/kvm/issues) | [Docs](https://jetkvm.com/docs)

[![Twitter](https://img.shields.io/twitter/url/https/twitter.com/jetkvm.svg?style=social&label=Follow%20%40JetKVM)](https://twitter.com/jetkvm)

</div>

This is a power monitoring and control module for the JetKVM platform, built on the Raspberry Pi RP2040, the same chip as the Raspberry Pi Pico.

## Features
- Voltage, current, and power measurements via INA219 sensor
- Power state control through UART interface

If you've found an issue and want to report it, please check our [Issues](https://github.com/jetkvm/dc-extension-firmware/issues) page. Make sure the description contains information about the firmware version you're using, your hardware setup, and a clear explanation of the steps to reproduce the issue.

# Development

The firmware is written in C using the Raspberry Pi Pico SDK. Knowledge of C programming and embedded systems is recommended. To get started, see [Getting Started with the Raspberry Pi Pico-Series](https://rptl.io/pico-get-started) for information on how to install the SDK and build the project.
