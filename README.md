# Smart Pill Dispenser Project

The Smart Pill Dispenser is an automated system designed to reliably dispense supplements or medication at scheduled intervals. This project combines hardware ingenuity with software integration to provide a seamless and efficient solution for managing daily health routines. It's perfect for integration into smart home systems like Home Assistant, offering both convenience and reliability.

## Overview

The Smart Pill Dispenser is an innovative solution designed to automate the process of dispensing pills, making it an essential tool for individuals managing supplements or medications. Utilizing a servo-controlled rotating disc, this system precisely dispenses pills from individual reservoirs into a collection cup. It is equipped with a robust web interface for easy scheduling and control, using MQTT commands for seamless integration into existing smart home setups like Home Assistant. This README provides a detailed guide on the hardware components, software setup, and usage instructions to get your Smart Pill Dispenser operational.

<div style="display:flex; justify-content:space-between; align-items:center;">
  <img src="https://github.com/Infraviored/Pilldispenser/blob/main/CAD/renders/render_front.png?raw=true" alt="Front View of Pill Dispenser" width="49%"/>
  <img src="https://github.com/Infraviored/Pilldispenser/blob/main/CAD/renders/render_rear.png?raw=true" alt="Rotating Discs" width="49%"/>
</div>



## Hardware

### Electrical Components
- **SG 90 Servo Motor:** Controls the rotating disc for dispensing pills.
- **Feedback Lights:** Provides visual feedback for successful dispensing and error notifications.
- **D1 Mini Microcontroller:** A Wemos D1 mini programmed with PIO (PlatformIO) code provided in the repository.
- **Piezo:** A cheap circular Piezo with less than 40mm diameter is required as impact drop sensor. Any will do.
- **Feedback Lights (Optional):** Provide visual feedback for successful dispensing and error notifications.
  
### Plastic Components

- **Pill Reservoirs:** Individual containers for each type of pill.
- **Rotating Disc:** Features a hole to grab and release pills into the collection cup.
- **Collection Cup:** Located at the bottom of the machine where dispensed pills are collected.

### Assembly

1. **3D Printed Parts:** Utilize the provided CAD models (`F3D` and `STEP` files) to print the physical components of the dispenser.
2. **Servo Motor Installation:** Fit the servo motor to the rotating disc ensuring it aligns correctly with the pill reservoirs.
3. **Final Assembly:** Follow the detailed assembly guide in the `hardware` folder to put together all parts of the pill dispenser.


## Features and Functionality

- **Automated Pill Dispensing:** Schedule daily dispensing at fixed intervals without manual intervention.
- **Smart Home Integration:** Easily integrates with existing smart home systems like Home Assistant via MQTT.
- **Error Handling:** Automated alerts for unsuccessful dispensing attempts, likely due to empty tanks.
- **Visual Feedback:** Light indicators for successful dispensing and error notifications ensure you're always informed.

## Support

For support, questions, or feedback, please open an issue in the GitHub repository or contact the maintainers directly.
