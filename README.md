# esphome-mlx-90640
Performant and feature complete implementation for the Melexis MLX 90640 in ESPHome

Start by compiling the example config and take it from there.
`esphome compile thermal_cam.yaml`

Verified working using a XIAO ESP32-S3.

The heatmap page (`/thermal`, `/thermal.json`, `/thermal.jpg`) is served when the config has a web server
(`web_server`, `captive_portal`, ...). Without one the sensors and the camera entity work on their own.
<img width="730" height="886" alt="image" src="https://github.com/user-attachments/assets/ab878865-ad5b-49d4-9270-a2f2322adcd6" />

<img width="1503" height="898" alt="image" src="https://github.com/user-attachments/assets/bebe8951-c62e-45fc-ab6d-93fbee14540b" />
