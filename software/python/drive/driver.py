# MicroPython classes for driving single motors, two motor robots, and stepper motors with 
# the Core Electronics Makerverse 2ch Motor Driver.
# 
# Written by Brenton Schulz
# Initial release: FEB-14-22

from gpiozero import PWMOutputDevice, Pin
import time

_FORWARD = 1
_REVERSE = 0

class motor():
    def __init__(self, pwmPin = 0, dirPin = 1, speed = 100, pwmFreq = 200):
        if isinstance(pwmPin, int):
            pwmPin = Pin(pwmPin, Pin.OUT)
        elif isinstance(pwmPin, Pin):
            pwmPin.init(mode=Pin.OUT)
        else:
            raise TypeError("Argument 'pwm' must be an integer or Pin object")
            
        if isinstance(dirPin, int):
            dirPin = Pin(dirPin, Pin.OUT)
        elif isinstance(dirPin, Pin):
            dirPin.init(mode=Pin.OUT)
        else:
            raise TypeError("Argument 'direction' must be an integer or Pin object")
                     
        self.pwmFreq = pwmFreq
                     
        self.pwm = PWMOutputDevice(pwmPin)
        self.pwm.freq(self.pwmFreq)
        self.pwm.duty_u16(0)
        self.pwmDuty = 63353
        self.speedValue = speed
        self.dirPin = dirPin
        self.direction = _FORWARD
        self.dirPin.value(self.direction)
        
    def speed(self, speed):
        if speed < 0:
            self.direction = _REVERSE
        else:
            self.direction = _FORWARD
        speed = abs(speed)
        self.speedValue = speed
        self.pwmDuty = int(speed/100*65535)
        if self.pwmDuty > 65535:
            self.pwmDuty = 65535
        self.go()
        
    def stop(self):
        self.pwm.duty_u16(0)
        
    def go(self):
        self.dirPin.value(self.direction)
        self.pwm.duty_u16(self.pwmDuty)
        
    def forward(self):
        self.direction = _FORWARD
        self.go()
        
    def reverse(self):
        self.direction = _REVERSE
        self.go()

    def drive(self, speed, duration_ms):
        self.speed(speed)
        time.sleep_ms(duration_ms)
        self.stop()

class twoMotorRobot():
    def __init__(self, pwmPinLeft = 0, dirPinLeft = 1, pwmPinRight = 2, dirPinRight = 3):
        self.motorLeft = motor(pwmPinLeft, dirPinLeft)
        self.motorRight = motor(pwmPinRight, dirPinRight)
    
    def speed(self, speed = 100):
        self.motorLeft.speed(speed)
        self.motorRight.speed(speed)
    
    def turnLeft(self):
        self.motorLeft.stop()
        self.motorRight.forward()
        
    def turnRight(self):
        self.motorLeft.forward()
        self.motorRight.stop()
        
    def rotateRight(self):
        self.motorLeft.reverse()
        self.motorRight.forward()
        
    def rotateLeft(self):
        self.motorLeft.forward()
        self.motorRight.reverse()
        
    def driveForward(self):
        self.motorLeft.forward()
        self.motorRight.forward()
        
    def driveReverse(self):
        self.motorLeft.reverse()
        self.motorRight.reverse()
        
    def stop(self):
        self.motorLeft.stop()
        self.motorRight.stop()
    
class bipolarStepper():
    def __init__(self, pwmPinA = 0, dirPinA = 1, pwmPinB = 2, dirPinB = 3, RPM = 10, stepsPerRotation = 200):
        if isinstance(pwmPinA, int):
            pwmPinA = Pin(pwmPinA, Pin.OUT)
        elif isinstance(pwmPinA, Pin):
            pwmPinA = pwmPinA.init(mode=Pin.OUT)
        else:
            raise TypeError("Argument 'pwmA' must be an integer or Pin object")
        
        if isinstance(pwmPinB, int):
            pwmPinB = Pin(pwmPinB, Pin.OUT)
        elif isinstance(pwmPinB, Pin):
            pwmPinB = pwmPinB.init(mode=Pin.OUT)
        else:
            raise TypeError("Argument 'pwmA' must be an integer or Pin object")
        
        if isinstance(dirPinA, int):
            dirPinA = Pin(dirPinA, Pin.OUT)
        elif isinstance(dirPinA, Pin):
            dirPinA = dirPinA.init(mode=Pin.OUT)
        else:
            raise TypeError("Argument 'pwmA' must be an integer or Pin object")
        
        if isinstance(dirPinB, int):
            dirPinB = Pin(dirPinB, Pin.OUT)
        elif isinstance(dirPinB, Pin):
            dirPinB = dirPinB.init(mode=Pin.OUT)
        else:
            raise TypeError("Argument 'pwmA' must be an integer or Pin object")
        
        self.pwmA = pwmPinA
        self.pwmB = pwmPinB
        self.dirA = dirPinA
        self.dirB = dirPinB
        
        self.pwmA.on()
        self.pwmB.on()
        self.dirA.on()
        self.dirB.on()
        
        self.next = "A"
        
        self.steps = 0
        
        self.stepDelay_ms = int(60000/(RPM*stepsPerRotation))
        
        self.RPM = RPM
        self.stepsPerRotation = stepsPerRotation

    def setRPM(self, RPM):
        self.stepDelay_ms = int(60000/(RPM*self.stepsPerRotation))

    def setHome(self):
        self.steps = 0

    def returnHome(self):
        while self.steps > 0:
            self.backwardStep()
            time.sleep_ms(self.stepDelay_ms)
        while self.steps < 0:
            self.forwardStep()
            time.sleep_ms(self.stepDelay_ms)

    def getSteps(self):
        return self.steps
    
    def getAngle(self):
        return self.steps%self.stepsPerRotation / self.stepsPerRotation * 360

    def forwardStep(self):
        if self.next == "A":
            if self.dirA.value() == 1:
                self.dirA.off()
            else:
                self.dirA.on()
            self.next = "B"
        else:
            if self.dirB.value() == 1:
                self.dirB.off()
            else:
                self.dirB.on()
            self.next = "A"
        self.steps += 1
        
    def backwardStep(self):
        if self.next == "A":
            if self.dirB.value() == 1:
                self.dirB.off()
            else:
                self.dirB.on()
            self.next = "B"
        else:
            if self.dirA.value() == 1:
                self.dirA.off()
            else:
                self.dirA.on()
            self.next = "A"
        self.steps -= 1
        
    def rotate(self, steps = 0, angle = None):
        if angle is not None:
            steps = round(angle/360.0*self.stepsPerRotation)
        
        if steps < 0:
            while steps < 0:
                self.backwardStep()
                steps += 1
                time.sleep_ms(self.stepDelay_ms)
        else:
            while steps > 0:
                self.forwardStep()
                steps -= 1
                time.sleep_ms(self.stepDelay_ms)