from matrix_lite import gpio
from matrix_lite import led
import numpy as np
from adafruit_rplidar import RPLidar
from math import floor
import matplotlib.pyplot as plt

def motorInit():
    gpio.setMode(8, 'output')
    gpio.setFunction(8, 'pwm')
    gpio.setMode(14, 'output')
    gpio.setFunction(14, 'pwm')
    
    gpio.setFunction(12, 'DIGITAL')
    gpio.setFunction(10, 'DIGITAL')
    gpio.setFunction(6, 'DIGITAL')
    gpio.setFunction(4, 'DIGITAL')
    gpio.setMode(10, 'output')
    gpio.setMode(12, 'output')
    gpio.setMode(6, 'output')
    gpio.setMode(4, 'output')

def motorSpeedLeft(direction, speed):
    if direction == 1: ## FORWARD!
        gpio.setDigital(6, 'ON') #Drive forward: ON
        gpio.setDigital(4, 'OFF') #Drive backwards: OFF
    
    elif direction == 0: #BACKWARDS!!
        gpio.setDigital(6, 'OFF') #Drive forward: OFF
        gpio.setDigital(4, 'ON') #Drive backwards: ON

    if speed < 0:
        speed = 0
    if speed > 100:
        speed = 100
        
    gpio.setPWM({
        "pin": 8,
        "percentage": speed,
        "frequency": 2000,
        })

def motorSpeedRight(direction, speed):

    if direction == 1: ## FORWARD!
        gpio.setDigital(12, 'ON') #Drive forward: ON
        gpio.setDigital(10, 'OFF') #Drive backwards: OFF

    elif direction == 0: #BACKWARDS!!
        gpio.setDigital(12, 'OFF') #Drive forward: OFF
        gpio.setDigital(10, 'ON') #Drive backwards: ON

    if speed < 0:
        speed = 0
    if speed > 100:
        speed = 100
        
    gpio.setPWM({
        "pin": 14,
        "percentage": speed,
        "frequency": 2000,
        })
        
# def followTarget(_middle_xy, motor_speed_left, motor_speed_right, perceptron_left_wheel = Perceptron(), perceptron_right_wheel = Perceptron()):
#     error_margin = 60 #Number of pixels to left or right that is considered within target
#     current_error_left = _middle_xy[0] - 320 # Changes coordinate system to have negative value on left side.
#     current_error_right = 320 - _middle_xy[0] # Changes voordinate system to have negative value on right side
#     print("Error left = ", current_error_left, " Error Right = ", current_error_right)
#     
#     if (current_error_right < -error_margin or current_error_right > error_margin) and (current_error_left < -error_margin or current_error_left > error_margin):
#         current_error_left /= 110  #Få værdien mellem 0 og 3
#         current_error_right /= 110 #Få værdien mellem 0 og 3        
#         output_left = perceptron_left_wheel.calcOutputAndUpdateWeight(current_error_left, perceptron_left_wheel.previous_reflex)
#         output_right = perceptron_right_wheel.calcOutputAndUpdateWeight(current_error_right, perceptron_right_wheel.previous_reflex)
#         print([output_left, output_right])
#         motor_speed_left += output_left
#         motor_speed_right += output_right
#         
#         if motor_speed_left < 20:
#             motor_speed_left = 20
#         if motor_speed_right < 20:
#             motor_speed_right = 20
#         if motor_speed_left > 60:
#             motor_speed_left = 60
#         if motor_speed_right > 60:
#             motor_speed_right = 60
#     else:
#         motor_speed_left = 30
#         motor_speed_right = 30
#     #Outcomment to test without motors running
#     #motor_speed_left = 0
#     #motor_speed_right = 0
#     return [motor_speed_left, motor_speed_right]
    

def toggleLED(led_number = [1], color = (0, 0, 0, 0)):
    led_vec = [(0,0,0,0)] * led.length
    for i in led_number:
        led_vec[i] = color
    led.set(led_vec)
    
    
    
def getLidarScan(lidar_data_queue):
    PORT_NAME = '/dev/ttyUSB0'
    lidar = RPLidar(None, PORT_NAME)
    scan_data = [0] * 360
    
    try:
        print(lidar.info)
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                scan_data[min([359, floor(angle)])] = distance
            ## Make sure there is only the newest data in the queue
            if lidar_data_queue.empty() is True:
                lidar_data_queue.put(scan_data)
            else:
                temp = lidar_data_queue.get()
                lidar_data_queue.put(scan_data)
    
    except KeyboardInterrupt:
        print('Stoping.')
    lidar.stop()
    lidar.disconnect()
                 
        
            

    
def makePlots(weights_l, weights_r, weights_distance, distance):
    # create a figure 
    fig = plt.figure() 
      
    # define subplots and their positions in figure 
    plt_weights_l = fig.add_subplot(221) 
    plt_weights_r = fig.add_subplot(222) 
    plt_weights_dist = fig.add_subplot(223) 
    plt_distance = fig.add_subplot(224)
    x_weights_l = np.arange(0, len(weights_l))
    x_weights_r = np.arange(0, len(weights_r))
    x_weights_dist = np.arange(0, len(weights_distance))
    x_distance = np.arange(0, len(distance))
    
    plt_weights_l.plot(x_weights_l, weights_l, color ='r') 
    plt_weights_l.set_title('Weights Left')
    
    plt_weights_r.plot(x_weights_r, weights_r, color ='b') 
    plt_weights_r.set_title('Weights Right')
    
    plt_weights_dist.plot(x_weights_dist, weights_distance, color ='g') 
    plt_weights_dist.set_title('Weights Distance')
    
    plt_distance.plot(x_distance, distance, color ='k') 
    plt_distance.set_title('Distance') 

    # adjusting space between subplots 
    fig.subplots_adjust(hspace=.5,wspace=0.5) 
  
    # function to show the plot 
    plt.savefig('graphs.png')
