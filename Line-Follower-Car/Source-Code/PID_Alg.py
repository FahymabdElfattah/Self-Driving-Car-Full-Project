import RPi.GPIO as GPIO
import time

# Infrared proximity sensor pins
I0_S = 7
I1_S = 11  
I2_S = 15  
I3_S = 37  
I4_S = 13

GPIO.setmode(GPIO.BOARD)   #Sets the numbering mode for the GPIO pins to use the physical pin numbering scheme
GPIO.setwarnings(False)
GPIO.setup(I0_S,GPIO.IN)  #declare ir sensor as input
GPIO.setup(I1_S,GPIO.IN)  #declare ir sensor as input
GPIO.setup(I2_S,GPIO.IN)  #declare ir sensor as input
GPIO.setup(I3_S,GPIO.IN)  #declare ir sensor as input
GPIO.setup(I4_S,GPIO.IN)  #declare ir sensor as input


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
        left_speed = base_speed - pid_output
        right_speed = base_speed + pid_output
        
        # Clip motor speeds to valid range
        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))
        
        print("left_speed : ",left_speed)
        print("right_speed : ",right_speed)

        last_error = error
        time.sleep(0.02)  # Adjust the delay as needed

except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()

'''
In the PID controller formula:

#Output = Kp * (Error) + Ki * ∫(Error) dt + Kd * d(Error)/dt
The terms Kp, Ki, and Kd are the proportional, integral, and derivative constants, respectively. 
These constants are used to control the influence of each component on the final control output. 
Let's break down the calculation of these terms in the code and explain their relation to the PID formula:

1. Proportional Term (P):

# proportional = error
The proportional term is simply set to the current error value. 
This term contributes to the control output in proportion to the current error. 
If the error is large, the control output will be more aggressive, and if the error is small, 
the control output will be milder. This helps the system respond quickly to changes and 
reduce steady-state error.

2. Integral Term (I):

#integral += error
The integral term accumulates the sum of past errors over time. 
In the code, the integral variable is updated by adding the current error in each iteration.
The integral term helps in eliminating steady-state error and correcting for any cumulative
offset that may exist over time.

3. Derivative Term (D):

#derivative = error - last_error
The derivative term calculates the rate of change of the error. 
The difference between the current error and the previous error (error - last_error) 
gives an indication of how quickly the error is changing. This term helps to predict the 
future trend of the error. If the error is rapidly decreasing, the derivative term will apply 
dampening to prevent overshooting.


Now, let's relate these calculated terms back to the PID formula:

    Kp * (Error): This corresponds to the proportional term. In the code, proportional holds the 
                   value of Error, and multiplying it by Kp applies the proportional control action.

    Ki * ∫(Error) dt: This corresponds to the integral term. The integral accumulates the integral 
                        of the error over time, and multiplying it by Ki applies the integral control 
                        action.

    Kd * d(Error)/dt: This corresponds to the derivative term. The derivative calculates the rate 
                        of change of the error, and multiplying it by Kd applies the derivative control 
                        action.

So, the code calculates these terms (proportional, integral, and derivative) based on the current error 
and its history, and then combines them to compute the overall control action (pid_output) that is applied 
to the motor speeds. This control action helps the line-following car navigate and maintain its desired 
path.
'''
