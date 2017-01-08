import RPi.GPIO as GPIO
import time


class US():
    def rotate(self, angle):
        duty = float(angle) / 9.0 + 2.5
        self.pwm.ChangeDutyCycle(duty)
    
    def distance(self):
        GPIO.output(self.triggerPin, True)
        time.sleep(0.00001)  
        GPIO.output(self.triggerPin, False)
        start = time.time()
        stop = time.time()
        while GPIO.input(self.echoPin) == 0:
            start = time.time()
        while GPIO.input(self.echoPin) == 1:
            stop = time.time()
        #GPIO.wait_for_edge(self.echoPin, GPIO.FALLING, timeout=100)
        stop = time.time()
        return ((stop - start) * 34300.0) / 2.0
    
    def findBestWay(self):
        # returns the angle where to go next
        func = reversed if self.rev else list
        arr = [0] * self.steps
        for i in func(range(self.steps)):
            #print(i * 180.0 / (self.steps-1))
            self.rotate(i * 180.0 / (self.steps-1))
            time.sleep(0.2)
            arr[i] = self.distance()
            # dist > 500cm or dist < 5cm means that a timeout occured => value wrong
            if arr[i] > 500 or arr[i] < 5:
                for j in range(3):
                    arr[i] = self.distance()
                    if arr[i] < 500 and arr[i] > 5:
                        break
                    else:
                        arr[i] = -1

        self.rev = not self.rev
        return arr
    
    def __init__(self, servoPin = 22, triggerPin = 12, echoPin = 13):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        # save vars
        self.echoPin    = echoPin
        self.triggerPin = triggerPin
        GPIO.setup(self.echoPin, GPIO.IN)
        GPIO.setup(self.triggerPin, GPIO.OUT)
        GPIO.output(self.triggerPin, False)
        # init servo
        self.servoPin = servoPin
        GPIO.setup(self.servoPin, GPIO.OUT)
        self.pwm = GPIO.PWM(servoPin, 100)
        self.pwm.start(5)
        self.steps = 7 # should be odd
        self.rev = False
        # set servo on 90 degree (mid)
        self.rotate(90)
        time.sleep(0.5)