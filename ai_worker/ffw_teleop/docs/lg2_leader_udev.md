# LG2 Leader udev Links

The operator-side launch expects stable leader device names:

```bash
/dev/left_leader
/dev/right_leader
```

Check whether each USB serial device has a unique serial ID:

```bash
udevadm info -q property -n /dev/ttyUSB7 | grep -E 'ID_SERIAL=|ID_PATH='
udevadm info -q property -n /dev/ttyUSB8 | grep -E 'ID_SERIAL=|ID_PATH='
```

If `ID_SERIAL` is present and different for the two leaders, prefer serial-based
rules because the USB port can change:

```udev
SUBSYSTEM=="tty", ENV{ID_SERIAL}=="LEFT_SERIAL_VALUE", SYMLINK+="left_leader", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ENV{ID_SERIAL}=="RIGHT_SERIAL_VALUE", SYMLINK+="right_leader", MODE="0666", GROUP="dialout"
```

If `ID_SERIAL` is missing or identical, use `ID_PATH` and keep each leader on the
same physical USB port:

```udev
SUBSYSTEM=="tty", ENV{ID_PATH}=="LEFT_ID_PATH_VALUE", SYMLINK+="left_leader", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ENV{ID_PATH}=="RIGHT_ID_PATH_VALUE", SYMLINK+="right_leader", MODE="0666", GROUP="dialout"
```

Install the rules on the main PC:

```bash
sudo tee /etc/udev/rules.d/99-lg2-leader.rules >/dev/null <<'EOF'
# Paste the two selected rules here.
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/left_leader /dev/right_leader
```
