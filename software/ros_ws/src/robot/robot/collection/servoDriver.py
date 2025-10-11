from rpi_hardware_pwm import HardwarePWM

# Channel 0 is GPIO 12
# Channel 1 is GPIO 13

class Servo180:
    def __init__(self, channel, start_angle=0, min_angle=-90, max_angle=90, min_pulse=1, max_pulse=2, frame_width=20):
        self.pwmHandle = HardwarePWM(pwm_channel=channel, hz=1 / (frame_width / 1000), chip=0)
        self.channel = channel
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.frame_width = frame_width
        self.angle = min(max(start_angle, self.min_angle), self.max_angle)
        self.pwmHandle.start(self.angle)
    
    def set_angle(self, angle):
        self.angle = min(max(angle, self.min_angle), self.max_angle)
        duty_cycle = (((self.angle - self.min_angle) / (self.max_angle - self.min_angle)) * (self.max_pulse - self.min_pulse) + self.min_pulse) / self.frame_width * 100
        self.pwmHandle.change_duty_cycle(duty_cycle)
        return duty_cycle

    def stop(self):
        self.pwmHandle.stop()
        self.angle = None