import errno
import os


class ServoError(Exception):
    """Raised when PWM sysfs operations fail."""


class Servo:
    """Linux sysfs PWM servo driver."""

    def __init__(self, channel: int, chip: int = 0,
                 freq_hz: int = 50,
                 min_us: int = 500, max_us: int = 2500,
                 min_angle: float = 0.0, max_angle: float = 180.0,
                 name: str = 'servo'):
        self.channel = int(channel)
        self.chip = int(chip)
        self.freq_hz = int(freq_hz)
        self.period_ns = int(1_000_000_000 / self.freq_hz)
        self.min_us = int(min_us)
        self.max_us = int(max_us)
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self.name = name
        self.last_angle = self.min_angle

        self._base = f'/sys/class/pwm/pwmchip{self.chip}'
        self._path = f'{self._base}/pwm{self.channel}'
        self._enabled = False

    def _write(self, name: str, value: int) -> None:
        path = os.path.join(self._path, name)
        try:
            with open(path, 'w') as f:
                f.write(str(value))
        except OSError as exc:
            raise ServoError(
                f'Failed to write {value} to {path}: {exc}') from exc

    def _export(self) -> None:
        if os.path.exists(self._path):
            return
        export_path = os.path.join(self._base, 'export')
        try:
            with open(export_path, 'w') as f:
                f.write(str(self.channel))
        except OSError as exc:
            if exc.errno != errno.EBUSY:
                raise ServoError(
                    f'Failed to export PWM {self.chip}/{self.channel}: {exc}') from exc

    def enable(self) -> None:
        """Export, set period and enable the PWM channel."""
        self._export()
        self._write('period', self.period_ns)
        self._write('duty_cycle', 0)
        self._write('enable', 1)
        self._enabled = True

    def disable(self) -> None:
        """Disable and unexport the PWM channel."""
        if not os.path.exists(self._path):
            self._enabled = False
            return
        try:
            self._write('enable', 0)
        except ServoError:
            pass
        try:
            with open(os.path.join(self._base, 'unexport'), 'w') as f:
                f.write(str(self.channel))
        except OSError:
            pass
        self._enabled = False

    def angle_to_duty_ns(self, angle: float) -> int:
        """Map an angle in degrees to duty cycle in nanoseconds."""
        clamped = max(self.min_angle, min(self.max_angle, float(angle)))
        pulse_us = self.min_us + (clamped / 180.0) * (self.max_us - self.min_us)
        return int(pulse_us * 1000)

    def set_angle(self, angle: float) -> None:
        """Move servo to the requested angle (clamped to limits)."""
        if not self._enabled:
            self.enable()
        clamped = max(self.min_angle, min(self.max_angle, float(angle)))
        self.last_angle = clamped
        duty_ns = self.angle_to_duty_ns(clamped)
        self._write('duty_cycle', duty_ns)
