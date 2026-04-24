# Motor SETUP

This note explains how to setup Motor for `Motor`.

## First-Time Setup

You must set can USB

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
```

### Setup Order

You setup first Motor ID:
```bash
python3 motor_test_gui_can_ID.py
```

You setup second Motor Origin:
```bash
python3 motor_test_gui_set_pos.py
```