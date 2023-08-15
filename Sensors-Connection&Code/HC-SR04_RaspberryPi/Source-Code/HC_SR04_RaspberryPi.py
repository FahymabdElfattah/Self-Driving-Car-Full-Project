import RPi.GPIO as GPIO
import time

PIN_TRIGGER = 7
PIN_ECHO = 13

def Init_GPIO():
      GPIO.setwarnings(False)
      GPIO.setmode(GPIO.BOARD)
      GPIO.setup(PIN_TRIGGER, GPIO.OUT)
      GPIO.setup(PIN_ECHO, GPIO.IN)

      GPIO.output(PIN_TRIGGER, GPIO.LOW)

def Get_Distance():
      #print ("Waiting for sensor to settle")
      time.sleep(2)
      #print ("Calculating distance")
      GPIO.output(PIN_TRIGGER, GPIO.HIGH)
      time.sleep(0.000001) 
      GPIO.output(PIN_TRIGGER, GPIO.LOW)
      while GPIO.input(PIN_ECHO)==0:
            pulse_start_time = time.time()
            obj_det = False
      while GPIO.input(PIN_ECHO)==1:
            pulse_end_time = time.time()
            obj_det = True
      pulse_duration = pulse_end_time - pulse_start_time
      distance = round(pulse_duration * 17150, 2)
      return distance

try:
     Init_GPIO()
     D = Get_Distance()
     if D < 10 :
           print("Object dtection") 
     else :
          print(" None")

      

      

      
    

      

except KeyboardInterrupt:
    print("Exception Error")
    # Clean up GPIO on program exit
    GPIO.cleanup()
