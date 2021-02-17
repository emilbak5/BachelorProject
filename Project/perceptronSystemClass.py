from DIYfunctionsAndClasses import *
from perceptronClass import *
class perceptronSystem:
    def __init__(self):
        self.percep_l_wheel = Perceptron()
        self.percep_r_wheel = Perceptron()
        self.motor_speed_left = 20
        self.motor_speed_right = 20
        self.person_detected = False
        self.default_speed = 30
        ## Used to measure distance
        self.percep_distance = Perceptron()
        self.person_width = 1800
        self.perceived_focal = 500
        self.target_distance = 1.5
        
    def followTarget(self, middle_xy, distance_to_person):
        ## For following target
        error_margin_width = 60 #Number of pixels to left or right that is considered within target
        current_error_left = middle_xy[0] - 320 # Changes coordinate system to have negative value on left side.
        current_error_right = 320 - middle_xy[0] # Changes voordinate system to have negative value on right side
        #print("Error left = ", current_error_left, " Error Right = ", current_error_right)
        ## For maintaining distance
        current_error_distance = distance_to_person - self.target_distance
        change_in_speeds_for_distance = self.percep_distance.calcOutputAndUpdateWeight(current_error_distance)
        
        if (current_error_right < -error_margin_width or current_error_right > error_margin_width) and (current_error_left < -error_margin_width or current_error_left > error_margin_width):
            current_error_left /= 110  #Få værdien mellem 0 og 3
            current_error_right /= 110 #Få værdien mellem 0 og 3        
            output_left  = self.percep_l_wheel.calcOutputAndUpdateWeight(current_error_left)
            output_right = self.percep_r_wheel.calcOutputAndUpdateWeight(current_error_right)

            self.motor_speed_left += output_left + change_in_speeds_for_distance
            self.motor_speed_right += output_right + change_in_speeds_for_distance
            
            if self.motor_speed_left < 20:
                self.motor_speed_left = 20
            if self.motor_speed_right < 20:
                self.motor_speed_right = 20
            if self.motor_speed_left > 60:
                self.motor_speed_left = 60
            if self.motor_speed_right > 60:
                self.motor_speed_right = 60
        else:
            self.motor_speed_left = 30
            self.motor_speed_right = 30
        print("Left motor: ", self.motor_speed_left, " Right motor: ", self.motor_speed_right)
        #Outcomment to test without motors running
        #self.motor_speed_left = 0 
        #self.motor_speed_right = 0 
        return [self.motor_speed_left, self.motor_speed_right]
        

    def getDistanceToPerson(self, object_width_in_pixels):
        #distance = (focal_length * real_height * image_height) / (object_height * sensor_height)
        distance = (self.person_width * self.perceived_focal) / object_width_in_pixels
        return distance / 1000 #Divided på 100 to get in meters
    
    def getPercievedFocal(self, P, D, H):
        F = (P * D) / H
        return F
    
    def disTarget(self, distance_to_person):
                
        current_error_distance = ((distance_to_person / 1000) - self.target_distance)
        print("Distance to person :", distance_to_person /1000, " target_distance: ", target_distance)
        print(current_error_distance)
        if current_error_distance < 3 and current_error_distance > -3:
            
            #self.motor_speed_left += output_distance
            #self.motor_speed_right += output_distance
#             print(self.motor_speed_left, self.motor_speed_right)
#             if self.motor_speed_left < 20:
#                 self.motor_speed_left = 20
#             if self.motor_speed_right < 20:
#                 self.motor_speed_right = 20
#             if self.motor_speed_left > 60:
#                 self.motor_speed_left = 60
#             if self.motor_speed_right > 60:
#                 self.motor_speed_right = 60
#         else:
#             self.motor_speed_left = 0
#             self.motor_speed_right = 0
        print ("Change in motor speed from distance = ", output_distance)
        return output_distance
        
        