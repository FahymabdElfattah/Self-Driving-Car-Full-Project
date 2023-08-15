'''
Project: Self-Driving Car
Author: FAHYM ABD ELFATTAH
Supervisor: EL HADBI ASSIA

'''

import RPi.GPIO as GPIO
from time import sleep

PIN_ENA1 = 32  #Motor1  L298N Pin ena1
PIN_IN1 = 16   #Motor1  L298N Pin in1 
PIN_IN2 = 18   #Motor1  L298N Pin in2
PIN_ENA2 = 33  #Motor1  L298N Pin ena2
PIN_IN3 = 38   #Motor2  L298N Pin in3
PIN_IN4 = 40   #Motor2  L298N Pin in4

pwm1 = None  # Declare pwm1 as a global variable
pwm2 = None  # Declare pwm2 as a global variable


def setup():
    global pwm1, pwm2          # Access the global variables pwm1 and pwm2
    GPIO.setmode(GPIO.BOARD)   #Sets the numbering mode for the GPIO pins to use the physical pin numbering scheme
    GPIO.setwarnings(False)    #Disables certain warnings related to pin states or conflicts

    GPIO.setup(PIN_ENA1,GPIO.OUT)
    GPIO.setup(PIN_ENA2,GPIO.OUT)
   
    GPIO.setup(PIN_IN1, GPIO.OUT)  #declare as output for L298N Pin in1
    GPIO.setup(PIN_IN2, GPIO.OUT)  #declare as output for L298N Pin in2
    GPIO.setup(PIN_IN3, GPIO.OUT)  #declare as output for L298N Pin in3
    GPIO.setup(PIN_IN4, GPIO.OUT)  #declare as output for L298N Pin in4
    
    pwm1 = GPIO.PWM(PIN_ENA1,1000)
    pwm2 = GPIO.PWM(PIN_ENA2,1000)
    
    pwm1.start(0)
    pwm2.start(0)


def forword(duty):
    pwm1.ChangeDutyCycle(duty)
    pwm2.ChangeDutyCycle(duty)
    
    GPIO.output(PIN_IN1, GPIO.HIGH)  #Set the forward direction of motor M1 to HIGH
    GPIO.output(PIN_IN2, GPIO.LOW)   #Set the backward direction of motor M1 to LOW
    GPIO.output(PIN_IN3, GPIO.LOW)   #Set the forward direction of motor M2 to LOW
    GPIO.output(PIN_IN4, GPIO.HIGH)  #Set the backward direction of motor M2 to HIGH

def turnLeft():
     GPIO.output(PIN_IN1, GPIO.LOW)   #Set the forward direction of motor M1 to LOW
     GPIO.output(PIN_IN2, GPIO.HIGH)  #Set the backward direction of motor M1 to HIGH
     GPIO.output(PIN_IN3, GPIO.LOW)   #Set the forward direction of motor M2 to LOW
     GPIO.output(PIN_IN4, GPIO.HIGH)  #Set the backward direction of motor M2 to HIGH

def turnRight():
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
      print("Self Driving Car...")
      setup()
      stop()
      forword(90)
      sleep(7)
      forword(100)
      sleep(7)
      stop()
          
      
      
           

except KeyboardInterrupt:
    GPIO.cleanup()   # Clean up GPIO on program exit




