from DIYfunctionsAndClasses import *
class perceptronSystem:
    def __init__(self):
        self.percep_l_wheel = Perceptron()
        self.percep_r_wheel = Perceptron()
        
    def followTarget(_middle_xy, motor_speed_left, motor_speed_right):
    error_margin = 60 #Number of pixels to left or right that is considered within target
    current_error_left = _middle_xy[0] - 320 # Changes coordinate system to have negative value on left side.
    current_error_right = 320 - _middle_xy[0] # Changes voordinate system to have negative value on right side
    print("Error left = ", current_error_left, " Error Right = ", current_error_right)
    
    if (current_error_right < -error_margin or current_error_right > error_margin) and (current_error_left < -error_margin or current_error_left > error_margin):
        current_error_left /= 110  #Få værdien mellem 0 og 3
        current_error_right /= 110 #Få værdien mellem 0 og 3        
        output_left  = self.percep_l_wheel.calcOutputAndUpdateWeight(current_error_left, self.percep_l_wheel.previous_reflex)
        output_right = self.percep_r_wheel.calcOutputAndUpdateWeight(current_error_right, self.percep_r_wheel.previous_reflex)
        print([output_left, output_right])
        motor_speed_left += output_left
        motor_speed_right += output_right
        
        if motor_speed_left < 20:
            motor_speed_left = 20
        if motor_speed_right < 20:
            motor_speed_right = 20
        if motor_speed_left > 60:
            motor_speed_left = 60
        if motor_speed_right > 60:
            motor_speed_right = 60
    else:
        motor_speed_left = 30
        motor_speed_right = 30
    #Outcomment to test without motors running
    #motor_speed_left = 0
    #motor_speed_right = 0
    return [motor_speed_left, motor_speed_right]
        

    
    
        