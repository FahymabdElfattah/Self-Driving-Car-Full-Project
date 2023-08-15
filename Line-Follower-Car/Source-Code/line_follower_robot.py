'''
Project: Self-Driving Car
Author: FAHYM ABD ELFATTAH
Supervisor: EL HADBI ASSIA

'''

import RPi.GPIO as GPIO
import time


PIN_IN1 = 16   #Motor1  L298N Pin in1 
PIN_IN2 = 18   #Motor1  L298N Pin in2
PIN_IN3 = 38   #Motor2  L298N Pin in3
PIN_IN4 = 40   #Motor2  L298N Pin in4

L_S = 11       #ir sensor Right
C_S = 15       #ir sensor Center
R_S = 37       #ir sensor Left

def setup():
    GPIO.setmode(GPIO.BOARD)   #Sets the numbering mode for the GPIO pins to use the physical pin numbering scheme
    GPIO.setwarnings(False)    #Disables certain warnings related to pin states or conflicts

    GPIO.setup(PIN_IN1, GPIO.OUT)  #declare as output for L298N Pin in1
    GPIO.setup(PIN_IN2, GPIO.OUT)  #declare as output for L298N Pin in2
    GPIO.setup(PIN_IN3, GPIO.OUT)  #declare as output for L298N Pin in3
    GPIO.setup(PIN_IN4, GPIO.OUT)  #declare as output for L298N Pin in4

    GPIO.setup(L_S,GPIO.IN)  #declare ir sensor as input
    GPIO.setup(C_S,GPIO.IN)  #declare ir sensor as input
    GPIO.setup(R_S,GPIO.IN)  #declare ir sensor as input


def forword():  
    GPIO.output(PIN_IN1, GPIO.HIGH)  #Set the forward direction of motor M1 to HIGH
    GPIO.output(PIN_IN2, GPIO.LOW)   #Set the backward direction of motor M1 to LOW
    GPIO.output(PIN_IN3, GPIO.LOW)   #Set the forward direction of motor M2 to LOW
    GPIO.output(PIN_IN4, GPIO.HIGH)  #Set the backward direction of motor M2 to HIGH

def turnRight():
     GPIO.output(PIN_IN1, GPIO.LOW)   #Set the forward direction of motor M1 to LOW
     GPIO.output(PIN_IN2, GPIO.HIGH)  #Set the backward direction of motor M1 to HIGH
     GPIO.output(PIN_IN3, GPIO.LOW)   #Set the forward direction of motor M2 to LOW
     GPIO.output(PIN_IN4, GPIO.HIGH)  #Set the backward direction of motor M2 to HIGH

def turnLeft():
     GPIO.output(PIN_IN1, GPIO.HIGH)  #Set the forward direction of motor M1 to HIGH
     GPIO.output(PIN_IN2, GPIO.LOW)   #Set the backward direction of motor M1 to LOW
     GPIO.output(PIN_IN3, GPIO.HIGH)  #Set the forward direction of motor M2 to HIGH
     GPIO.output(PIN_IN4, GPIO.LOW)   #Set the backward direction of motor M2 to LOW

def stop():
    GPIO.output(PIN_IN1, GPIO.LOW)    #Set the forward direction of motor M1 to LOW
    GPIO.output(PIN_IN2, GPIO.LOW)    #Set the backward direction of motor M1 to LOW
    GPIO.output(PIN_IN3, GPIO.LOW)    #Set the forward direction of motor M2 to LOW
    GPIO.output(PIN_IN4, GPIO.LOW)    #Set the backward direction of motor M2 to LOW


     

try:
      print("Line Follower Robot (-_-)")
      setup()
      stop()
      while(True):

           if GPIO.input(C_S) == 1 :
                forword() #if Center Sensor Black color then it will call forword function
           if (GPIO.input(R_S) == 0) and (GPIO.input(L_S) == 0) : 
                forword() #if Right Sensor and Left Sensor are at White color then it will call forword function

           if (GPIO.input(R_S) == 1) and (GPIO.input(L_S) == 0) : 
                turnRight() #if Right Sensor is Black and Left Sensor is White then it will call turn Right function

           if (GPIO.input(R_S) == 0) and (GPIO.input(L_S) == 1) :
                turnLeft() #if Right Sensor is White and Left Sensor is Black then it will call turn Left function
            
           if (GPIO.input(R_S) == 1) and (GPIO.input(L_S) == 1):
                stop() #if Right Sensor and Left Sensor are at Black color then it will call Stop function



except KeyboardInterrupt:
    GPIO.cleanup()   # Clean up GPIO on program exit

