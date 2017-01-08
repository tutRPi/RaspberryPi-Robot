import RPi.GPIO as GPIO
import time
import threading
import math
import xbox
import lirc # https://github.com/tompreston/python-lirc
from l293d import L293D
from ultrasonic import US
from MCP3008 import MCP3008


class Robot():
    def __init__(self, motor_left_pin1=17, motor_left_pin2=27, motor_right_pin1=23, motor_right_pin2=24,
                 line_follow_pin_left=19, line_follow_pin_right=6, servo_pin=22, us_trigger_pin=12, us_echo_pin=13 ):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # init modules
        self.ultrasonic = US(servo_pin, us_trigger_pin, us_echo_pin)
        self.motor = L293D(motor_left_pin1, motor_left_pin2, motor_right_pin1, motor_right_pin2)
        self.line_follow_pin_left  = line_follow_pin_left
        self.line_follow_pin_right = line_follow_pin_right
        
        GPIO.setup(self.line_follow_pin_left,  GPIO.IN)
        GPIO.setup(self.line_follow_pin_right, GPIO.IN)
        
        
    
    def lineFollowModeOn(self):
     
        status_left  = False
        status_right = False
        
        while True:
            status_left  = bool(GPIO.input(self.line_follow_pin_left))  # False: not on line / sensor too distant from bottom
            status_right = bool(GPIO.input(self.line_follow_pin_right)) # True: on line
            
            if status_left and status_right:
                # one the line, follow straight on
                self.motor.forward()
            elif status_left:
                # line is on the left, move left (motor right)
                self.motor.forwardRight()
            elif status_right:
                # line is on the right, move right (motor left)
                self.motor.forwardLeft()
            else:
                # have gone astray, search line. first go back some cm
                self.motor.backward()
                time.sleep(7.5/self.motor.DIST_PER_SEC)
                self.motor.stop()
                # rotate x degrees to search the line
                degrees_to_search = 45.0
                self.motor.forwardRight()
                s = GPIO.wait_for_edge(self.line_follow_pin_left, GPIO.RISING, timeout=int(1000 * self.motor.SEC_PER_TURN / 360.0 * degrees_to_search))
                self.motor.stop()
                if s is not None:
                    # line found, continue
                    continue
                else:
                    # nothing found, go back to original position
                    self.motor.backwardRight()
                    time.sleep(self.motor.SEC_PER_TURN / 360.0 * degrees_to_search)
                    # search in other side
                    self.motor.forwardLeft()
                    s = GPIO.wait_for_edge(self.line_follow_pin_right, GPIO.RISING, timeout=int(1000 * self.motor.SEC_PER_TURN / 360.0 * degrees_to_search))
                    self.motor.stop()
                    if s is not None:
                        # line found, continue
                        continue
                    else:
                        # line could not be found, go back to original position, stop
                        self.motor.backwardLeft()
                        time.sleep(self.motor.SEC_PER_TURN / 360.0 * degrees_to_search)
                        self.motor.stop()
                        break
            time.sleep(0.001)
    
    def IRControl(self):
        sockid = lirc.init("robot_ir", "lircrc")
        dist_per_leypress = 30.0 #cm
        while True:
            #self.motor.stop()
            code = lirc.nextcode()[0]
            print(code)
            if code == "FORWARD":
                self.motor.forward()
                time.sleep(dist_per_leypress / self.motor.DIST_PER_SEC)
                self.motor.stop()
            elif code == "LEFT_FORWARD":
                # 30 degree turn left, then stop
                self.motor.forwardRight()
                time.sleep(self.motor.SEC_PER_TURN / 360.0 * 30.0)
                self.motor.stop()
            elif code == "RIGHT_FORWARD":
                # 30 degree turn right, then stop
                self.motor.forwardLeft()
                time.sleep(self.motor.SEC_PER_TURN / 360.0 * 30.0)
                self.motor.stop()
            elif code == "LEFT":
                self.motor.forwardRight()
                time.sleep(self.motor.SEC_PER_TURN / 360.0 * 90.0)
                self.motor.stop()
            elif code == "RIGHT":
                self.motor.forwardLeft()
                time.sleep(self.motor.SEC_PER_TURN / 360.0 * 90.0)
                self.motor.stop()
            elif code == "BACKWARD":
                self.motor.backward()
                time.sleep(dist_per_leypress / self.motor.DIST_PER_SEC)
                self.motor.stop()
            elif code == "LEFT_BACKWARD":
                # 30 degree turn left back, then stop
                self.motor.backwardRight()
                time.sleep(self.motor.SEC_PER_TURN / 360.0 * 30.0)
                self.motor.stop()
            elif code == "RIGHT_BACKWARD":
                # 30 degree turn right back, then stop
                self.motor.backwardLeft()
                time.sleep(self.motor.SEC_PER_TURN / 360.0 * 30.0)
                self.motor.stop()
            elif code == "STOP":
                self.motor.stop()
            elif code == "LINE_FOLLOW_ON":
                self.lineFollowModeOn()
    
    def autoPilotUSon(self):
        actualIndex = int(self.ultrasonic.steps / 2)
        degreePerStep = 180.0 / (self.ultrasonic.steps-1)
        while True:
            dists = self.ultrasonic.findBestWay()
            print(dists)
            maxIndex = dists.index(max(dists))
            steps = abs(90.0 - maxIndex * degreePerStep) / degreePerStep + 1
            # if distance is more than 500cm, the measurement is probably wrong -> stop
            if dists[maxIndex] > self.motor.DIST_PER_SEC / 2:# and dists[maxIndex] < 500:
                if maxIndex == int(self.ultrasonic.steps / 2):
                    # straight forward
                    self.motor.forward()
                elif maxIndex < int(self.ultrasonic.steps / 2):
                    # turn right
                    self.motor.forwardLeft()
                    time.sleep(self.motor.SEC_PER_TURN / 360.0 * degreePerStep * steps)
                    self.motor.forward()
                elif maxIndex > int(self.ultrasonic.steps / 2):
                    # turn left
                    self.motor.forwardRight()
                    time.sleep(self.motor.SEC_PER_TURN / 360.0 * degreePerStep * steps)
                    self.motor.forward()
                actualIndex = maxIndex
            else:
                print(dists[maxIndex], self.motor.DIST_PER_SEC)
                self.motor.stop()
                return
    
    def followVoice(self):
        adc = MCP3008()
        while True:
            front = adc.read(0)
            right = adc.read(1)
            left = adc.read(2)
            # tiny values are invalid => map them to 1023
            if front < 10: front = 1023;
            # motors make noise of about 150, if motor is running
            if right < 50: right = 1023;
            if left < 50: left = 1023;
            print(left, front, right)
            #print(front,right,left)
            if front < 1000 and right > 1000 and left > 1000:
                self.motor.forwardRight()
            elif front > 1000 and right < 1000 and left > 1000:
                self.motor.forwardLeft()
            elif front > 1000 and right > 1000 and left < 1000:
                self.motor.forwardRight()
            elif front < 1000 and right < 1000 and left > 1000:
                # front / right
                self.motor.forwardRight()
                time.sleep(self.motor.SEC_PER_TURN / 360.0 * 45)
            elif front < 1000 and right > 1000 and left < 1000:
                # front / left
                self.motor.forwardLeft()
                time.sleep(self.motor.SEC_PER_TURN / 360.0 * 45)
            else:
                self.motor.stop()
            time.sleep(0.1)
    
    def moveByAxis(self, x, y):
        
        if x == 0.0 and y == 0.0:
            # nowhere to move, stop
            self.motor.stop()
        elif x == 0:
            # only forward / backward
            if y < 0:
                self.motor.backward()
            else:
                self.motor.forward()
        else:
            angle = math.degrees(math.atan(y/x))
            angle += 180 if x < 0 else 360
            angle = angle % 360
            
            if angle == 0:
                self.motor.forwardRight()
            elif angle == 180:
                self.motor.forwardLeft()
            if angle > 0 and angle < 90:
                # forwardRight
                self.motor.forwardRight()
                time.sleep(self.motor.SEC_PER_TURN / 360.0 * angle)
                self.motor.forward()
                time.sleep(5.0 / self.motor.DIST_PER_SEC) # move 5cm forward
            elif angle > 90 and angle < 180:
                # forwardLeft
                angle -= 90
                self.motor.forwardLeft()
                time.sleep(self.motor.SEC_PER_TURN / 360.0 * angle)
                self.motor.forward()
                time.sleep(5.0 / self.motor.DIST_PER_SEC) # move 5cm forward
                
    
    def xboxControl(self):
        joy = xbox.Joystick()
        while not joy.Back():
        
            
            # button A to stop
            if joy.A():
                self.motor.stop()
            else:
                # left joystick to move
                x, y = joy.leftStick()
                self.moveByAxis(x, y)
                    
        joy.close()