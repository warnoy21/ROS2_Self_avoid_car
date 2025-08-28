import time
import os

class GPIOController:
    def __init__(self, gpio_number):
        self.gpio_number = gpio_number
        self.gpio_path = f"/sys/class/gpio/gpio{gpio_number}/"

    def export(self):
        if not os.path.exists(self.gpio_path):
            with open("/sys/class/gpio/export", "w") as f:
                f.write(str(self.gpio_number))

    def unexport(self):
        with open("/sys/class/gpio/unexport", "w") as f:
            f.write(str(self.gpio_number))

    def set_direction(self, direction):
        with open(f"{self.gpio_path}direction", "w") as f:
            f.write(direction)

    def read_direction(self):
        with open(f"{self.gpio_path}direction", "r") as f:
            return f.read().strip()

    def set_value(self, value):
        with open(f"{self.gpio_path}value", "w") as f:
            f.write(str(value))

    def read_value(self):
        with open(f"{self.gpio_path}value", "r") as f:
            return int(f.read().strip())


