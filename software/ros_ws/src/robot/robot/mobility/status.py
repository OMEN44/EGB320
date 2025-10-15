from gpiozero import LED

class statusLEDs:
    def __init__(self):
        self.red = LED(17)
        self.yellow = LED(27)
        self.green = LED(22)

    def clearAll(self):
        self.red.off()
        self.yellow.off()
        self.green.off()

    def setRed(self, state=True):
        if state:
            self.red.on()
        else:
            self.red.off()

    def setYellow(self, state=True):
        if state:
            self.yellow.on()
        else:
            self.yellow.off()

    def setGreen(self, state=True):
        if state:
            self.green.on()
        else:
            self.green.off()