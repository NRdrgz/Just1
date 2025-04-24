# Connect to the Nintendo Lic Pro Controller
To connect to the Lic Pro Controller from the Raspberry Pi <br>
First make sure Pi has bluetooth enabled <br>
```bash
sudo apt install bluetooth bluez bluez-tools rfkill -y
sudo systemctl enable bluetooth
sudo systemctl start bluetooth
```

Put controller in pairing mode  <br>
Then <br>
```bash
bluetoothctl
power on
agent on
default-agent
scan on
```

You'll see something like  <br>
[NEW] Device XX:XX:XX:XX:XX:XX Lic Pro Controller <br>

```bash
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
```

Then <br>
```bash
exit
```

You can verify if it worked by using joystick <br>
```bash
sudo apt install joystick
jstest /dev/input/js0
```