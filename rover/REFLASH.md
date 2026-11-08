# Reflash the RoverC-Pro (M5StickC PLUS) with the HQ-compatible firmware

Firmware: `rover/M5_RPi/M5_RPi.ino` (Arduino needs the sketch inside a folder of the same name).
It creates Wi-Fi **RoverControl** (password `Rover2026Test!`), rover at **192.168.4.1**, takes one-letter commands
F/B/L/R (move), Q/E (rotate), S (stop) on **UDP port 4210**, only from **192.168.4.2**, and stops by itself
0.4 s after the last command. This is what `rover/rover_control.py`, `hq/follow.py` and `hq/run.py` speak.

Check the label first: this sketch is for **M5StickC PLUS**. A **PLUS2** needs the M5StickCPlus2 library instead.

## Laptop (Arduino IDE 2)
1. File -> Preferences -> Additional boards manager URLs:
   `https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json`
2. Tools -> Board -> Boards Manager: install **M5Stack**.
3. Tools -> Manage Libraries: install **M5StickCPlus** and **M5_RoverC** (by M5Stack), accept dependencies.
4. File -> Open -> `rover/M5_RPi/M5_RPi.ino`.
5. Plug the M5StickC in with a USB-C data cable (it can stay on the rover).
6. Tools -> Board -> M5Stack -> **M5StickCPlus**; Tools -> Port -> the new COM port
   (none shows up: install the CH9102 USB driver from M5Stack, replug).
7. Upload. If it fails: Tools -> Upload Speed -> 115200, retry.
8. Screen shows: `Rover Ready`, `192.168.4.1`, `UDP Port: 4210`.

## HQ Pi: join the rover network on wlan1 (SSH stays on wlan0)
```
sudo nmcli dev wifi connect RoverControl password 'Rover2026Test!' ifname wlan1
sudo nmcli con mod RoverControl ipv4.method manual ipv4.addresses 192.168.4.2/24 ipv4.never-default yes
sudo nmcli con up RoverControl
cd ~/forge-dhrubo && python3 hq/follow.py --test F --secs 0.5
```
If the screen is right but the wheels never turn: check the RoverC base's own power switch and battery.
