from gpiozero import LED

class statusLEDs:
    def __init__(self):
        self._red = LED(17)
        self._yellow = LED(27)
        self._green = LED(22)

    def clearAll(self):
        self._red.off()
        self._yellow.off()
        self._green.off()

    def setRed(self, state=True):
        if state:
            self._red.on()
        else:
            self._red.off()

    def setYellow(self, state=True):
        if state:
            self._yellow.on()
        else:
            self._yellow.off()

    def setGreen(self, state=True):
        if state:
            self._green.on()
        else:
            self._green.off()