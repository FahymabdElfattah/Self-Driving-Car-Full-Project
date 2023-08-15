import RPi.GPIO as GPIO
import time

# Set the GPIO mode
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# Define the pins
PIN_ENA = 32
PIN_IN1 = 16   # Motor1 L298N Pin in1 
PIN_IN2 = 18   # Motor1 L298N Pin in2
PIN_ENB = 33
PIN_IN3 = 38   # Motor2 L298N Pin in3
PIN_IN4 = 40   # Motor2 L298N Pin in4

# Set up the GPIO pins
GPIO.setup(PIN_ENA, GPIO.OUT)
GPIO.setup(PIN_IN1, GPIO.OUT)
GPIO.setup(PIN_IN2, GPIO.OUT)
GPIO.setup(PIN_ENB, GPIO.OUT)
GPIO.setup(PIN_IN3, GPIO.OUT)
GPIO.setup(PIN_IN4, GPIO.OUT)

pwm_a = GPIO.PWM(PIN_ENA,100)
pwm_b = GPIO.PWM(PIN_ENB,100)
    
def set_motor_speeds(left_speed, right_speed):    
#     pwm_freq_a = abs(left_speed)   
#     pwm_freq_b = abs(right_speed)   
#     pwm_a = GPIO.PWM(PIN_ENA, pwm_freq_a)
#     pwm_b = GPIO.PWM(PIN_ENB, pwm_freq_b)
    
    if left_speed >= 0:
        GPIO.output(PIN_IN1, GPIO.HIGH)
        GPIO.output(PIN_IN2, GPIO.LOW)
    else:
        GPIO.output(PIN_IN1, GPIO.LOW)
        GPIO.output(PIN_IN2, GPIO.HIGH)

    if right_speed < 0:
        GPIO.output(PIN_IN3, GPIO.HIGH)
        GPIO.output(PIN_IN4, GPIO.LOW)
    else:
        GPIO.output(PIN_IN3, GPIO.LOW)
        GPIO.output(PIN_IN4, GPIO.HIGH)

    pwm_a.start(abs(left_speed))
    pwm_b.start(abs(right_speed))

try:
    while True:
        set_motor_speeds(60,80)
            
        
        
        
        

except KeyboardInterrupt:
    pass

# Clean up GPIO settings
GPIO.cleanup()

