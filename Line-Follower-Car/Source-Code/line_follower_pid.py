import RPi.GPIO as GPIO
import time

# Infrared proximity sensor pins
I0_S = 7
I1_S = 11  
I2_S = 15  
I3_S = 37  
I4_S = 13

# Define the pins
PIN_ENA = 32
PIN_IN1 = 16   # Motor1 L298N Pin in1 
PIN_IN2 = 18   # Motor1 L298N Pin in2
PIN_ENB = 33
PIN_IN3 = 38   # Motor2 L298N Pin in3
PIN_IN4 = 40   # Motor2 L298N Pin in4

GPIO.setmode(GPIO.BOARD)   #Sets the numbering mode for the GPIO pins to use the physical pin numbering scheme
GPIO.setwarnings(False)
GPIO.setup(I0_S,GPIO.IN)  #declare ir sensor as input
GPIO.setup(I1_S,GPIO.IN)  #declare ir sensor as input
GPIO.setup(I2_S,GPIO.IN)  #declare ir sensor as input
GPIO.setup(I3_S,GPIO.IN)  #declare ir sensor as input
GPIO.setup(I4_S,GPIO.IN)  #declare ir sensor as input

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


# PID parameters
kp = 0.1  # Proportional constant
ki = 0.01  # Integral constant
kd = 0.01  # Derivative constant

# Motor speeds
base_speed = 50

# Function to read sensor values
def read_sensors():
    i0 =  GPIO.input(I0_S)
    i1 = GPIO.input(I1_S)
    i2 = GPIO.input(I2_S)
    i3 = GPIO.input(I3_S)
    i4 = GPIO.input(I3_S)
    return i0,i1,i2,i3,i4

try:
    last_error = 0
    integral = 0

    while True:
        i0,i1,i2,i3,i4 = read_sensors()
        Sensor = [i0,i1,i2,i3,i4]
        Coeff = [-2,-1,0,1,2]
        for x in range(0,4,1):
            error = Sensor[x]*Coeff[x]

        
        
        # Calculate PID terms
        proportional = error
        integral += error
        derivative = error - last_error
        
        # Calculate PID output
        pid_output = (kp * proportional) + (ki * integral) + (kd * derivative)
        
        # Apply PID output to motor speeds
        left_speed = base_speed + pid_output
        right_speed = base_speed - pid_output
        
        # Clip motor speeds to valid range
        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))
        
        set_motor_speeds(left_speed,right_speed)

        last_error = error
        time.sleep(0.02)  # Adjust the delay as needed

except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()

