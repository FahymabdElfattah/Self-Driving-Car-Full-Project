import RPi.GPIO as GPIO
import time


S_L = 37 #11
S_C = 15
S_R = 11 #37

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(S_L,GPIO.IN)
GPIO.setup(S_C,GPIO.IN)
GPIO.setup(S_R,GPIO.IN)


#set the behaviour of led as output

try:
    print("S_L= ",GPIO.input(S_L))
    print("S_C = ",GPIO.input(S_C))
    print("S_R = ",GPIO.input(S_R))
        



except KeyboardInterrupt:
    GPIO.cleanup()


