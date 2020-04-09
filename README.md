# EIM-328 Wearables for Opthamology Research 
>(Jan 2020 - Present)

> Building a research tool for researchers to predict myopia

> A device for consumers to understand their daily eye habits

> #IOTWearable

## Table of Contents 

- [BOM](#BOM)
- [ESP32_Setup](#ESP32_Setup)
- [Universally Unique Identifier](#UUID)
- [Team](#team)
- [FAQ](#faq)
- [Support](#support)
- [License](#license)

## BOM

| Item                    | Purpose                                               | Qty | Remarks |
|-------------------------|-------------------------------------------------------|-----|---------|
| Heltec ESP32 Wifi Kit   | Microcontroller Unit                                  | 1   |         |
| VL53L1X Sparkfun Sensor | Measurement of distance between user's eye and object | 3   |         |
| IMU 6 DOF Sensor        | Measurement of acceleration and gyrometer values      | 1   |         |
| Photodiode              | Measurement of ambient lighting conditions            | 1   |         |
| DS3231 RTC Module       | Timestamp for each individual sensor reading          | 1   |         |
| SD Card Module          | Logging of sensor readings                            | 1   |         |

## ESP32_Setup

> Arduino Setup
1. Go to File -> Preferences in Arduino IDE.
      * Enter https://dl.espressif.com/dl/package_esp32_index.json into the “Additional Board Manager URLs” field. 
      * Note: if you already have the ESP8266 boards URL, you can separate the URLs with a comma.
2. Go to Tools > Board > Boards Manager…
      * Search for ESP32 and press install button for the “ESP32 by Espressif Systems“:
      * Select your Board in Tools > Board menu 
3. Test out example codes to check if your board is working, otherwise install usb drivers for your board.

## Universally Unique Identifier (UUID)

| Name                  | UUID                                 |
|-----------------------|--------------------------------------|
| TOF_SERVICE_UUID      | efbf52a5-d22b-4808-bccd-b45c5b1d1928 |
| TOF_UUID              | 3018bff0-ca31-430b-a6ef-dc5fefd7ee17 |
| LDR_UUID              | e9ff40d9-21da-44dd-b125-ad2d8ef6b026 |
|                       |                                      |
| MOVEMENT_SERVICE_UUID | 739157ab-dfe6-4dc1-84df-6cd801769d7d |
| ROTATION_UUID         | 2403ca8c-0500-4404-8141-6b0210045365 |
| ACCEL_UUID            | d0b5f187-ac23-459f-b44b-e20d50bcf656 |
|                       |                                      |
| TIME_SERVICE_UUID     | 57675859-a6f4-4445-9492-051aa8514552 |
| TIME_UUID             | 10ccece5-e44b-4502-8b69-09646d4072e1 |
|                       |                                      |
| DATA_SERVICE_UUID     | b8ec9f13-81e2-489f-b736-f4e440c86e03 |
| DATA_CALL_UUID        | 5022e570-0f19-4357-848a-fc74234b1348 |
| DATA_SEND_UUID        | 38ca7184-8eeb-481f-9197-2c106f076031 |


## Features


## FAQ
---
## Team

> Or Contributors/People

| <a href="http://fvcproductions.com" target="_blank">**FVCproductions**</a> | <a href="http://fvcproductions.com" target="_blank">**FVCproductions**</a> | <a href="http://fvcproductions.com" target="_blank">**FVCproductions**</a> |
| :---: |:---:| :---:|
| [![FVCproductions](https://avatars1.githubusercontent.com/u/4284691?v=3&s=200)](http://fvcproductions.com)    | [![FVCproductions](https://avatars1.githubusercontent.com/u/4284691?v=3&s=200)](http://fvcproductions.com) | [![FVCproductions](https://avatars1.githubusercontent.com/u/4284691?v=3&s=200)](http://fvcproductions.com)  |
| <a href="http://github.com/fvcproductions" target="_blank">`github.com/fvcproductions`</a> | <a href="http://github.com/fvcproductions" target="_blank">`github.com/fvcproductions`</a> | <a href="http://github.com/fvcproductions" target="_blank">`github.com/fvcproductions`</a> |

- You can just grab their GitHub profile image URL
- You should probably resize their picture using `?s=200` at the end of the image URL.

## Support

Reach out to me at the following places:

- Instagram at <a href="http://instagram.com/bryanlmk" target="_blank">`@bryanlmk`</a>
- Email at <a href=mailto:e0310111@u.nus.edu>NUSmail</a>

---

## License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org)

- **[MIT license](http://opensource.org/licenses/mit-license.php)**
- Copyright 2015 © <a href="http://fvcproductions.com" target="_blank">FVCproductions</a>.

---

**Badges will go here**

- build status
- issues (waffle.io maybe)

> GIF Tools

- Use <a href="http://recordit.co/" target="_blank">**Recordit**</a> to create quicks screencasts of your desktop and export them as `GIF`s.
- For terminal sessions, there's <a href="https://github.com/chjj/ttystudio" target="_blank">**ttystudio**</a> which also supports exporting `GIF`s.

**Recordit**

![Recordit GIF](http://g.recordit.co/iLN6A0vSD8.gif)

**ttystudio**

![ttystudio GIF](https://raw.githubusercontent.com/chjj/ttystudio/master/img/example.gif)

---


