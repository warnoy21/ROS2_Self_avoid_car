
import time
import subprocess
import os

class PWM:
    def __init__(self, chip, channel, period_ns):
        self.chip = chip
        self.channel = channel
        self.period = period_ns
        self.base_path = f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"
        self.export_path = f"/sys/class/pwm/pwmchip{chip}/export"
        self._export_pwm()

    def _run(self, command):
        subprocess.run(command, shell=True)

    def _write(self, path, value):
        self._run(f"echo {value} | sudo tee {path} > /dev/null")

    def _export_pwm(self):
        if not os.path.exists(self.base_path):
            self._write(self.export_path, self.channel)
            time.sleep(0.1)  # wait for sysfs to populate

    def enable(self):
        self._write(f"{self.base_path}/period", self.period)
        self._write(f"{self.base_path}/duty_cycle", 0)
        self._write(f"{self.base_path}/enable", 1)

    def set_duty_cycle(self, duty_ns):
        self._write(f"{self.base_path}/duty_cycle", duty_ns)

    def set_brightness(self, brightness_255):
        brightness_255 = max(0, min(255, brightness_255))
        duty = int(brightness_255 * (self.period / 255))
        self.set_duty_cycle(duty)

    def disable(self):
        self._write(f"{self.base_path}/enable", 0)





