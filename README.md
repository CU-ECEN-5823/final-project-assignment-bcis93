# BLE Mesh Node - UV Light Monitoring
This repository contains the code for a BLE Mesh node a connected UV Light sensor. It is part of a larger, greenhouse monitoring system. See the links below for more information.

### Links
[Shared Folder](https://drive.google.com/drive/u/1/folders/1F-_n8RPkxCzO7q5ui7oLMy1UNxIx9kCD)

[Individual Folder](https://drive.google.com/drive/u/1/folders/1JDDYCrJjrd6WlADjWn1jjjsxZNxw6Rsj)

### Status
The UV Light sensor has been fully integrated into the project. Currently, it takes a measurement once every second, and logs the results. It also publishes the latest data to the friend node every 2 seconds. I have designed a state machine (similar to the one we designed for the temperature sensor at the beginning of the semester) for this sensor which handles the process of taking a measurement. It is also designed to allow as much sleep in between steps as possible. I tested the sensor by taking it outside and verifying that the readings were close to expected values (based on the weather app on my phone).

I have integrated support for the LCD screen, which displays UV readings, but only when requested by the user (by pressing PB0). This allows the display to be off for most of the time, increasing energy savings.

Finally, I have also implemented the low power node capabilities, including establishing a friendship with the friend node. This allows the node to sleep as much as possible. I have also optimized each part of the system to use as little energy as possible, including sleeping as much as possible and using LPM on the UV light sensor.
