# Raspberry Pi Camera Module 3 on Ubuntu 24.04

**Note: You use the already developed [RPI Camera v3 ROS Node](https://github.com/dnovischi/rpicamv3-ros) instead of these instructions.**

Ubuntu 24.04 does not support the Raspberry Pi Camera Module 3 out of the box. The Ubuntu `libcamera` packages are the upstream version (no Raspberry Pi camera pipeline support), so you need to build and install Raspberry Pi's `libcamera` fork and `rpicam-apps` from source.

## Summary

- **Goal:** Get Camera Module 3 working on Ubuntu 24.04.
- **Approach:** Build and install Raspberry Pi’s `libcamera` fork + `rpicam-apps`.

## Prerequisites

- Raspberry Pi 4 or 5 running Ubuntu 24.04
- Camera Module 3 properly connected
- Internet connection
- At least 2GB free disk space

## Step 1: Remove conflicting packages

```bash
# Remove existing rpicam-apps
sudo apt remove --purge rpicam-apps

# Remove standard libcamera (if installed)
sudo apt remove --purge libcamera-dev libcamera0
```

## Step 2: Install build dependencies

```bash
# Essential build tools
sudo apt install -y git python3-pip python3-jinja2 meson cmake ninja-build

# libcamera dependencies
sudo apt install -y libboost-dev libgnutls28-dev openssl libtiff5-dev pybind11-dev
sudo apt install -y python3-yaml python3-ply libglib2.0-dev libgstreamer-plugins-base1.0-dev

# rpicam-apps dependencies
sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
sudo apt install -y libepoxy-dev libjpeg-dev libtiff5-dev libpng-dev

# For desktop Ubuntu (with GUI preview)
sudo apt install -y qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5

# Hardware encoder support
sudo apt install -y v4l-utils
```

## Step 3: Build libcamera (Raspberry Pi fork)

```bash
# Clone the Raspberry Pi fork
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera

# Configure the build
meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp \
  -Dipas=rpi/vc4,rpi/pisp \
  -Dv4l2=true \
  -Dgstreamer=enabled \
  -Dtest=false \
  -Dlc-compliance=disabled \
  -Dcam=disabled \
  -Dqcam=disabled \
  -Ddocumentation=disabled \
  -Dpycamera=enabled

# Build (this takes ~10–15 minutes)
ninja -C build

# Install
sudo ninja -C build install

# Return to home directory
cd ~
```

> [!NOTE]
> For low-memory devices (Pi Zero, Pi 3), add `-j 1` to limit compilation to a single core:
>
> ```bash
> ninja -C build -j 1
> ```

## Step 4: Build rpicam-apps

```bash
# Clone rpicam-apps
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps

# Configure for desktop Ubuntu
meson setup build \
  -Denable_libav=disabled \
  -Denable_drm=enabled \
  -Denable_egl=enabled \
  -Denable_qt=enabled \
  -Denable_opencv=disabled \
  -Denable_tflite=disabled \
  -Denable_hailo=disabled

# For Ubuntu Server (headless), use this instead:
# meson setup build \
#   -Denable_libav=disabled \
#   -Denable_drm=enabled \
#   -Denable_egl=disabled \
#   -Denable_qt=disabled \
#   -Denable_opencv=disabled \
#   -Denable_tflite=disabled \
#   -Denable_hailo=disabled

# Build
meson compile -C build

# Install
sudo meson install -C build

# Update library cache
sudo ldconfig

cd ~
```

## Step 5: Enable the camera hardware overlay

Edit the boot configuration:

```bash
sudo nano /boot/firmware/config.txt
```

Add this at the end (choose the correct overlay for your camera):

```ini
# For Camera Module 3
dtoverlay=imx708

# For other cameras:
# dtoverlay=imx477  # HQ Camera
# dtoverlay=imx296  # GS Camera
# dtoverlay=imx519  # 16MP Camera
```

Save with `Ctrl+X`, then `Y`, then `Enter`.

## Step 6: Reboot

```bash
sudo reboot
```

## Step 7: Test the camera

After reboot:

```bash
# Check version
rpicam-hello --version

# List detected cameras
rpicam-hello --list-cameras

# Preview for 5 seconds
rpicam-hello -t 5000

# Take a photo
rpicam-still -o test.jpg

# Record 10 seconds of video (IMPORTANT: use --codec yuv420)
rpicam-vid -t 10000 -o test.h264 --codec yuv420
```

## Important: Video recording fix

> [!WARNING]
> When recording video, you must specify `--codec yuv420` to use the hardware H.264 encoder. Without it, `rpicam-vid` can fail with an encoder error.

```bash
# Correct way to record video
rpicam-vid -t 10000 -o video.h264 --codec yuv420

# Record at 1080p
rpicam-vid -t 10000 -o video.h264 --codec yuv420 --width 1920 --height 1080

# Record with 10 Mbps bitrate
rpicam-vid -t 10000 -o video.h264 --codec yuv420 -b 10000000

# Continuous recording (Ctrl+C to stop)
rpicam-vid -t 0 -o video.h264 --codec yuv420
```

## Optional: Make `yuv420` default

To avoid typing `--codec yuv420` every time:

```bash
echo "alias rpicam-vid='rpicam-vid --codec yuv420'" >> ~/.bashrc
source ~/.bashrc
```

## Common commands reference

### Photo capture

```bash
# Basic photo
rpicam-still -o photo.jpg

# Full resolution photo
rpicam-still -o photo.jpg -r

# Photo with 5-second preview
rpicam-still -o photo.jpg -t 5000

# Photo at specific resolution
rpicam-still -o photo.jpg --width 1920 --height 1080
```

### Video recording

```bash
# 30-second video
rpicam-vid -t 30000 -o video.h264 --codec yuv420

# 4K video
rpicam-vid -t 10000 -o video.h264 --codec yuv420 --width 3840 --height 2160

# High quality video (20 Mbps)
rpicam-vid -t 10000 -o video.h264 --codec yuv420 -b 20000000
```

### Preview

```bash
# 10-second preview
rpicam-hello -t 10000

# Fullscreen preview
rpicam-hello -t 10000 --fullscreen
```

## Troubleshooting

### Camera not detected

```bash
# Check camera connection
vcgencmd get_camera

# Check kernel messages
dmesg | grep -i camera

# Verify video devices exist
ls -l /dev/video*

# List available video devices
v4l2-ctl --list-devices
```

You should see `rpivid` in the output, which is the hardware encoder.

### Build fails on low-memory devices

If compilation fails with memory errors on devices like Raspberry Pi Zero or Pi 3:

```bash
# Use single-core compilation
ninja -C build -j 1
meson compile -C build -j 1
```

### Video encoding error

If you see `ERROR: *** Unable to find an appropriate H.264 codec ***`, you likely forgot to add `--codec yuv420`:

```bash
# Wrong (will fail)
rpicam-vid -t 10000 -o video.h264

# Correct (will work)
rpicam-vid -t 10000 -o video.h264 --codec yuv420
```

### Permission denied

Add your user to the `video` group:

```bash
sudo usermod -aG video $USER
```

Log out and back in for changes to take effect.

## Why this works

Ubuntu’s `libcamera` comes from the upstream repository and doesn’t include Raspberry Pi-specific drivers. Raspberry Pi maintains a fork with:

- Support for Raspberry Pi camera hardware
- PiSP (Pi Signal Processor) integration
- Optimized camera tuning files
- Hardware encoder integration

By building from Raspberry Pi’s fork, you get full camera functionality on Ubuntu.

## Performance notes

- **Hardware encoding:** The `yuv420` codec uses the Raspberry Pi’s hardware H.264 encoder (`rpivid`), which provides excellent performance with minimal CPU usage.
- **Preview:** The EGL preview uses GPU acceleration for smooth display.
- **Quality:** The hardware encoder produces high-quality video comparable to Raspberry Pi OS.

## Tested on

- Raspberry Pi 5 + Ubuntu 24.04 LTS
- Raspberry Pi 4 + Ubuntu 24.04 LTS
- Camera Module 3
- Desktop and Server editions

## Reference

- https://dev.to/minindu_pasan_8f0e03c1063/how-to-setup-raspberry-pi-camera-module-3-on-ubuntu-2404-4pme
