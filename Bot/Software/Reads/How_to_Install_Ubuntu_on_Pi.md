# Install Ubuntu on Raspberry Pi 
## Step 1: Download Ubuntu Image

1. Go to the [Ubuntu Raspberry Pi page](https://ubuntu.com/raspberry-pi).
2. Download your preferred version (e.g., Ubuntu Server 24.04 LTS). Make sure to install Ubuntu Server instead of Ubuntu Desktop if you want to avoid plugging keyboard and mouse on the Pi.

---

## Step 2: Flash the Image to microSD

1. Download and install the [Raspberry Pi Imager](https://www.raspberrypi.com/software/).
2. Launch the Imager and:
   - **Choose OS**: Select “Other general-purpose OS > Ubuntu”
   - **Choose Storage**: Select your microSD card
   - **Customize before writing**:
     - You should have a popup offering you to input custom settings
     - Enable **Set hostname** (e.g., `raspberrypi.local`). This will allow to connect to it without IP address
     - Enable and set **SSH** (e.g., username `your-username`, password `your-password`)
     - Enable and configure **Wi-Fi**:
       - SSID: `your-network-name`
       - Password: `your-password`
       - Wi-Fi country: `US` (or your 2-letter country code)

3. Click **Save**, then **Write** to flash the image.

---

## Step 3: First Boot

1. Insert the microSD into your Raspberry Pi.
2. Power it on.

The Pi will:
- Automatically connect to the Wi-Fi
- Enable SSH access
- Be ready for headless setup

Wait for 5-10 min for the installation to complete

---

## Step 4: Connect via SSH

Once the Pi is online, open a terminal on your computer and connect using:

```bash
ssh your-username@raspberrypi
```