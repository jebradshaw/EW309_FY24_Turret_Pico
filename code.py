# EW309 Turret Launcher Test J. Bradshaw 20240108

import time
import board             # contains constants for board specific pins
import digitalio         # module for setting up digital logic output on a pin
import busio             # module used for 
import adafruit_bno055   # IMU module
import pwmio             # used for the PWM output on the Motor Ports
from simple_pid import PID  #https://github.com/m-lundberg/simple-pid
    #  also see https://micropython-simple-pid.readthedocs.io/en/latest/

PI = 3.1415926545359
TO_RAD = 0.017453292 #PI/180.0

# PID Controller parameters for Yaw Axis
pidYaw = PID(5.0, 5.0, 0.0, setpoint = 0.0)
pidYaw.output_limits = (-1.0, 1.0)    # Output value will be between +/-1
pidYaw.auto_mode = True   # pid is enabled again

# PID Controller parameters for Pitch Axis
pidPitch = PID(5.0, 5.0, 0.0, setpoint = 0.0)
pidPitch.output_limits = (-1.0, 1.0)    # Output value will be between +/-1
pidPitch.auto_mode = True   # pid is enabled again

# Create the I2C interface.
SDA = board.GP6 #SDA = board.GP4
SCL = board.GP7 #SCL = board.GP5
i2c = busio.I2C(SCL, SDA)

sensor = adafruit_bno055.BNO055_I2C(i2c)
sensor.mode = adafruit_bno055.IMUPLUS_MODE #NDOF_MODE

led = board.GP25 #pico led pin
led = pwmio.PWMOut(led,frequency=20000) # freq of PWM is 20KHz on pin GP25 (led)

fire = board.GP14 #pico pin attached to belt motor driver MOSFET - WHITE Connector
fire = digitalio.DigitalInOut(fire)
fire.direction = digitalio.Direction.OUTPUT

spin = board.GP15 #pico pin attached to spinner motors driver MOSFET - YELLOW Connector
spin = digitalio.DigitalInOut(spin)
spin.direction = digitalio.Direction.OUTPUT

# digital I/O to control the pitch motor driver (H-Bridge is L298 on J2)
motPitch_ME2 = board.GP8  # ME2 on J2 - When gun barrel points down, negative pitch angle on IMU,
                         # negitive motor corrective action
motPitch_ME2 = pwmio.PWMOut(motPitch_ME2,frequency=20000) # freq of PWM is 20KHz on pin GP8 (ME2)

motPitch_IN2 = board.GP9  # IN2 on J2
motPitch_IN2 = digitalio.DigitalInOut(motPitch_IN2)
motPitch_IN2.direction = digitalio.Direction.OUTPUT

motPitch_IN1 = board.GP10  # IN1 on J2
motPitch_IN1 = digitalio.DigitalInOut(motPitch_IN1)
motPitch_IN1.direction = digitalio.Direction.OUTPUT

# digital I/O to control the Yaw motor driver (H-Bridge is L298 on J1)
motYaw_ME1 = board.GP11  # ME1 on J1 - When gun barrel turns right (clockwise looking  down),
                       #  positive Yaw angle on IMU, negative motor corrective action
motYaw_ME1 = pwmio.PWMOut(motYaw_ME1,frequency=20000) # freq of PWM is 20KHz on pin GP11 (ME1)

motYaw_IN2 = board.GP12  # IN2 on J1
motYaw_IN2 = digitalio.DigitalInOut(motYaw_IN2)
motYaw_IN2.direction = digitalio.Direction.OUTPUT

motYaw_IN1 = board.GP13  # IN1 on J1
motYaw_IN1 = digitalio.DigitalInOut(motYaw_IN1)
motYaw_IN1.direction = digitalio.Direction.OUTPUT

# history variables for cooperative multitasking (time-slicing)
imu_last = 0.0
led_last = 0.0
pitch_last = 0.0
yaw_last = 0.0

brightness = 0   # how bright the led is
fadeAmount = 6000  # how many increments to fade the led by

yawSetPoint = 0.0
pitchSetPoint = 0.0
con_state = 0
yaw_err_unwrapped = 0.0

errorYaw = 0.0
errorPitch = 0.0

# Motor Yaw control function for L298 Board H-Bridge driver Port J1
def mot_yaw_control(dc):
    if dc > 1.0:
        dc = 1.0
    if dc < -1.0:
        dc = -1.0
    
    dc = dc * (int(2** 16)-1)
    #dc *= 65535.0
         
    if dc > 65:       # 0.001*65535.0:
        motYaw_IN1.value = False
        motYaw_IN2.value = True
        motYaw_ME1.duty_cycle = int(dc)
  
    elif dc < -65:     # -0.001*65535.0:
        motYaw_IN2.value = False
        motYaw_IN1.value = True
        motYaw_ME1.duty_cycle = abs(int(dc))
        
    else:
        motYaw_IN1.value = False
        motYaw_IN2.value = False
        motYaw_ME1.duty_cycle = 0
        
# Motor Pitch control function for L298 Board H-Bridge driver Port J2
def mot_pitch_control(dc):
    if dc > 1.0:
        dc = 1.0
    if dc < -1.0:
        dc = -1.0
    
    #dc *= 65535.0
    dc = dc * (int(2** 16)-1)
         
    if dc > 65:      # 0.001*65535.0:
        motPitch_IN1.value = False
        motPitch_IN2.value = True
        motPitch_ME2.duty_cycle = int(dc)
  
    elif dc < -65:    # -0.001*65535.0:
        motPitch_IN2.value = False
        motPitch_IN1.value = True
        motPitch_ME2.duty_cycle = abs(int(dc))
        
    else:
        motPitch_IN1.value = False
        motPitch_IN2.value = False
        motPitch_ME2.duty_cycle = 0        

# Takes current corrected yaw measurement (yaw_cor) and current yaw Set Point (yawSP) in RADIANS
# Outputs the correct error accouunting for the zero crossover, takes the shortest path
def yawContoller(yaw_cor, yawSP):
    #first check for the sign, which direction is faster to turn in   
    #set the set point
    yaw_sp = yawSP
    yaw_error = 0.0
    yaw_sign_calc = 0.0
    yaw_temp_error = 0.0
    
    yaw_err_unwrapped = yaw_sp - yaw_cor
    
    if yaw_sp >= yaw_cor:   # If the set point is greater then the corrected heading
        yaw_error = yaw_sp - yaw_cor       # get the difference
        
        if yaw_error <= PI:        #Turn left
            con_state = 1
            yaw_sign_calc = 1.0
            
            #is the error < -PI
            if (yaw_error < 0.0) and (abs(yaw_error) > PI):
                yaw_error += 2.0*PI
              
            #is the error < -PI
            if (yaw_error > 0.0) and (abs(yaw_error) > PI):
                yaw_error = 2.0*PI - yaw_error
            
            #calculate the heading offset from error relative to setpoint for controller  
            if yaw_error < 0.0:
                yaw_temp_error = yaw_sp + yaw_error
            else:
                yaw_temp_error = yaw_sp - yaw_error

        elif yaw_error > PI:       #Turn right
            con_state = 2
            yaw_sign_calc = -1.0
            
            #is the error < -PI
            if (yaw_error < 0.0) and (abs(yaw_error) > PI):
                yaw_error += 2.0*PI
              
            #is the error < -PI
            if (yaw_error > 0.0) and (abs(yaw_error) > PI):
                yaw_error = 2.0*PI - yaw_error
            
            #calculate the heading offset from error relative to setpoint for controller  
            if yaw_error < 0.0:
                yaw_temp_error = yaw_sp - yaw_error
            else:
                yaw_temp_error = yaw_sp + yaw_error

    elif yaw_sp < yaw_cor:
        yaw_error = yaw_cor - yaw_sp
        
        if yaw_error <= PI:    #difference is
            con_state = 3 
            yaw_sign_calc=-1.0              #Turn left

            #is the error < -PI
            if (yaw_error < 0.0) and (abs(yaw_error) > PI):
                yaw_error += 2.0*PI
                
            #is the error < -PI
            if (yaw_error > 0.0) and (abs(yaw_error) > PI):
                yaw_error = 2.0*PI - yaw_error
            
            #calculate the heading offset from error relative to setpoint for controller  
            if yaw_error < 0.0:
                yaw_temp_error = yaw_sp - yaw_error
            else:
                yaw_temp_error = yaw_sp + yaw_error

        elif yaw_error > PI:   #180
            con_state = 4
            yaw_sign_calc = 1.0           #turn right
            
            #is the error < -PI
            if (yaw_error < 0.0) and (abs(yaw_error) > PI):
                yaw_error += 2.0*PI
                
            #is the error < -PI
            if (yaw_error > 0.0) and (abs(yaw_error) > PI):
                yaw_error = 2.0*PI - yaw_error
            
            #calculate the heading offset from error relative to setpoint for controller  
            if yaw_error < 0.0:
                yaw_temp_error = yaw_sp + yaw_error
            else:
                yaw_temp_error = yaw_sp - yaw_error
                
    return yaw_temp_error

# main loop
while True:
    tm = time.monotonic()
    print("Time Is:{} ".format(time.monotonic()), end='')
    
    if tm > imu_last + .02:
        imu_last = tm        
        #print("Euler angle: {}\n".format(sensor.euler), end='')
        
    if tm > led_last + 0.05:
        led_last = tm        
        brightness += fadeAmount
        if brightness > 65535:
            brightness = 65535
        if brightness < 0:
            brightness = 0
        led.duty_cycle = brightness
        #print("led bright:{} ".format(brightness), end='')        
        if brightness <= 0 or brightness >= 65535:
            fadeAmount = -fadeAmount        
        #led.value = 1 #set led high
        #led.value = 0 #set led low
        
    # PID Yaw Controller
    if tm > yaw_last + .01:
        yaw_last = tm
        yawMeas = sensor.euler[0] # get the current Yaw reading from the IMU
        
        errorYaw = yawContoller(yawMeas * TO_RAD, yawSetPoint * TO_RAD)
            
        controlYaw = pidYaw(errorYaw) # Compute the new output from the PID
        
        mot_yaw_control(-controlYaw)

    # PID Yaw Controller
    if tm > pitch_last + .01:
        pitch_last = tm
        pitchMeas = sensor.euler[1] # get the current Pitch reading from the IMU
        
        errorPitch = pitchSetPoint - pitchMeas        
            
        controlPitch = pidPitch(errorPitch * TO_RAD) # Compute the new output from the PID
        
        mot_pitch_control(controlPitch)
        
    print("Yaw:{},".format(yawMeas), end='')
    print("Yerr:{},".format(errorYaw), end='')
    print("Pitch:{},".format(pitchMeas), end='')    
    print("Perr:{},".format(errorPitch), end='')    
    #print("ConState:{}".format(con_state), end='')
    print("")
                            
    #print('')
    time.sleep(.01)    