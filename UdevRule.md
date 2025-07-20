# Using udev Rules for VESC in Docker (Jetson Orin AGX)

This guide explains how to set up `udev` rules for a VESC (or similar USB device) on Jetson Orin AGX, and access it securely from within a Docker container.

---

## 🔧 1. Identify Your VESC Device

Plug in your VESC and run:

```bash
udevadm info -a -n /dev/ttyACM1
```

Note the `idVendor` and `idProduct`, e.g.:

```bash
ATTRS{idProduct}=="5740"
ATTRS{idVendor}=="0483"
```

---

## 📄 2. Create a udev Rule (Host Only)

Create the rule file on your host:

```bash
sudo nano /etc/udev/rules.d/99-vesc.rules
```

Example rule:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="vesc"
```

- Sets ownership to `dialout` group
- I tried to use the group. It did not work well within docker. So, I set the mode at 0666.
- Adds a persistent symlink at `/dev/vesc`

Reload and apply:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 👤 3. Add Your User to the `dialout` Group

```bash
sudo usermod -aG dialout $USER
```

Then log out and log back in.

---

## 🐳 4. Access Device in Docker

Run the container with access to the device and group:

```bash
docker run -it \
  --device=/dev/vesc \
  --group-add dialout \
  your_image_name
```

Inside the container, verify:

```bash
ls -l /dev/vesc
groups
```

---

## ⚠️ Note About udev Inside Docker

- `udev` **does not run** inside containers by default.
- Adding rules in the container (`/etc/udev/rules.d/`) has **no effect**.
- Isaac ROS containers **include** `udev` rules for convenience, but you must apply them on the **host**.

---

## ✅ Summary

| Task                        | Where It Belongs       | Purpose                                   |
|----------------------------|------------------------|-------------------------------------------|
| udev rule file             | Host (`/etc/udev/...`) | Controls device permissions & symlinks    |
| Add user to dialout        | Host                   | Grants access to device without `sudo`    |
| Docker `--device` option   | Docker run command     | Passes device into container              |
| Docker `--group-add`       | Docker run command     | Gives container user permission to access |

---

## 🛠 Optional: RealSense & Isaac ROS

RealSense `udev` rules may appear inside Isaac ROS Docker images, but:
- They're **meant to be copied to the host**
- They do **not apply** unless placed in the host `/etc/udev/rules.d/`

Install on host:

```bash
sudo cp realsense-udev-rules/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## 🧪 Optional: Test Your Rule

```bash
udevadm test /sys/class/tty/ttyUSB0
```


# After adding VESC, I added other devices simlarly.