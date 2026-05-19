import evdev
import subprocess
import sys

class Keyboard:
    def __init__(self):
        for i in range(8):
            try:
                output = subprocess.run(
                    f"udevadm info --name=/dev/input/event{i} | grep KEYBOARD",
                    shell=True,
                    capture_output=True,
                )
                if output.stdout:
                    self.device = evdev.InputDevice(f"/dev/input/event{i}")
                    return None
            except:
                pass

        print("Keyboard not found at /dev/input/event[0-7]")
        sys.exit(1)

    def read_key(self):
        event = self.device.read_one()
        if event is not None and event.type == evdev.ecodes.EV_KEY:
            event = evdev.categorize(event)
            if event.keystate in {0, 1}:
                return (event.keycode, event.keystate)
            else:
                return (None, None)
        else:
            return (None, None)
