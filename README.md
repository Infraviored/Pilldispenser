# Smart Pill Dispenser Project

The Smart Pill Dispenser is an automated system designed to reliably dispense supplements or medication at scheduled intervals. This project combines hardware ingenuity with software integration to provide a seamless and efficient solution for managing daily health routines. It's perfect for integration into smart home systems like Home Assistant, offering both convenience and reliability.

## Overview

This system utilizes a servo-controlled rotating disc for precise dispensing, ensuring each pill is delivered from its individual reservoir to the collection cup. It features a robust web interface for scheduling and control, utilizing MQTT commands for easy integration into existing smart home setups. This README outlines the hardware components, software setup, and usage instructions to get your Smart Pill Dispenser up and running.

## Hardware

### Components

- **Pill Reservoirs:** Individual containers for each type of pill.
- **Servo Motor:** Controls the rotating disc for dispensing pills.
- **Rotating Disc:** Features a hole to grab and release pills into the collection cup.
- **Collection Cup:** Located at the bottom of the machine where dispensed pills are collected.
- **Feedback Lights:** Provides visual feedback for successful dispensing and error notifications.

### Assembly

1. **3D Printed Parts:** Utilize the provided CAD models (`F3D` and `STEP` files) to print the physical components of the dispenser.
   - `render_front.png`: ![Front View of Pill Dispenser](path/to/render_front.png)
   - `render_disks.png`: ![Rotating Discs](path/to/render_disks.png)
2. **Servo Motor Installation:** Fit the servo motor to the rotating disc ensuring it aligns correctly with the pill reservoirs.
3. **Final Assembly:** Follow the detailed assembly guide in the `hardware` folder to put together all parts of the pill dispenser.

## Software

### Dependencies

- **Microcontroller:** A Wemos D1 mini programmed with PIO (PlatformIO) code provided in the repository.

### Setup

1. **Microcontroller Firmware:** Flash your Wemos D1 mini with the PIO code located in the `firmware` directory.
2. **Web Interface:** Configure the web interface for your dispenser. Detailed instructions are available in the `software` folder.
3. **MQTT Integration:** Set up MQTT commands for integration with your smart home system, allowing for automated scheduling.

## Features and Functionality

- **Automated Pill Dispensing:** Schedule daily dispensing at fixed intervals without manual intervention.
- **Smart Home Integration:** Easily integrates with existing smart home systems like Home Assistant via MQTT.
- **Error Handling:** Automated alerts for unsuccessful dispensing attempts, likely due to empty tanks.
- **Visual Feedback:** Light indicators for successful dispensing and error notifications ensure you're always informed.

## Getting Started

Detailed instructions on assembling the hardware, setting up the software, and integrating the pill dispenser into your smart home system can be found in the respective folders within this repository.

## Support

For support, questions, or feedback, please open an issue in the GitHub repository or contact the maintainers directly.
