# ESP32 IoT CallKit Demo

## Overview

Bring an ESP32 device into an Agora IoT calling flow so mobile clients can call the device in real time.

Repository: [AgoraIO-Community/ag-iot-callkit-esp32-demo](https://github.com/AgoraIO-Community/ag-iot-callkit-esp32-demo)

## Prerequisites and Setup

1. Use the ESP32-S3-Korvo-2 board, a compatible camera and external speaker. Prepare ESP-IDF release/v4.4 and ESP-ADF, applying the IDF patch required by the README.
2. Provision Agora Link service access, a device license and the companion app. In `main/app_config.h`, configure the project/customer/product values and set `CONFIG_USER_ID` to the intended app user's ID.
3. Activate the installed ESP-IDF environment and run `idf.py set-target esp32s3`, `idf.py menuconfig` and `idf.py build` from this repository. Configure Wi-Fi and board settings in menuconfig.
4. Flash with `idf.py -p <serial-port> flash monitor`, replacing the placeholder with your board's serial port. Wait for the REC-key prompt.

## What To Check

- Press REC and confirm the companion app receives the incoming call.
- Answer to verify two-way audio and device video, then hang up.
- Call the device from the companion app and confirm the reverse call flow.

## Integration Notes

This requires compatible hardware, licensed IoT SDK binaries and Agora Link provisioning. `AGORA_APP_ID` alone and a second generic RTC client cannot replace those prerequisites.

Keep App Certificates, customer secrets and privileged token generation on a trusted backend for production. Use only your own test credentials while configuring historical demos; do not commit them. Preserve the sample's existing dependency versions and lifecycle until a separate migration is tested.

## Source References

- [README.md](../../README.md)
- [main/app_config.h](../../main/app_config.h)

## Validation Status

This recipe was checked against the repository's source and setup documents. The platform build and live service/device flow have not been run as part of this documentation change. Complete the checks above in an environment with the required SDKs, hardware and service access before treating it as a validated deployment.

## Implementation Prompt

```text
You are working in AgoraIO-Community/ag-iot-callkit-esp32-demo. Use docs/ai/RECIPE.md and its linked source files to adapt the ESP32 IoT CallKit Demo reference to this project.

Start by inspecting the actual sample directory and configuration described above. Resolve its listed missing prerequisites before claiming it is runnable. Preserve the existing architecture and compatible dependency versions. Implement the recipe's feature-specific flow, permissions and resource cleanup. Use a trusted backend for privileged credentials and tokens in production.

Run the repository-specific build or launch flow, then perform each What To Check item. Report the commands run, the observed results and anything blocked by service access, hardware or incomplete sample code. Do not claim live validation from a documentation or catalog build alone.
```
