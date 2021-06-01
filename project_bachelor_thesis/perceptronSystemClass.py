from DIYfunctionsAndClasses import *
from perceptronClass import *
import matplotlib.pyplot as plt
import csv
import math

class perceptronSystem:
    def __init__(self):
        self.percep_l_wheel = Perceptron(acti_func = 'sigmoid', summation_gain=0.5, name = 'Perceptron left wheel')
        self.percep_r_wheel = Perceptron(acti_func = 'sigmoid', summation_gain=0.5, name = 'Perceptron right wheel')
        #self.percep_l_obstacle = Perceptron(acti_func = 'sigmoid', learning_rate=0.5, variable_weight=2, name = 'Perceptron left obstacle')
        #self.percep_r_obstacle = Perceptron(acti_func = 'sigmoid', learning_rate=0.5, variable_weight=2, name = 'Perceptron right obstacle')
        self.percep_obstacle = Perceptron(acti_func = 'sigmoid', learning_rate=0.1, variable_weight=1, name = 'Perceptron obstacle')

        self.motor_speed_left_follow = 0
        self.motor_speed_right_follow = 0
        self.motor_speed_distance = 20
        self.motor_speed_total_left = 20
        self.motor_speed_total_right = 20
        self.motor_speed_obstacle_avoidance_left = 0
        self.motor_speed_obstacle_avoidance_right = 0
        
        self.start_avoiding_obstacle_dist = 700
        self.reflex_avoiding_obstacle_dist = 400
        self.closest_obstacle_when_starting_reflex = 0
        self.angle_right = 40
        self.angle_left = 320
        
        self.person_detected = False
        self.obstacle_in_reflex_range = False
        self.avoiding_obtacle_by_turning_left = False
        self.first_reflex_turn = False # Makes it possible to get average speed when entering reflex
        self.first_arc_turn = False
        self.obstacle_turning_range = False
        self.robot_is_stationary = True
        self.just_stopped_turning = True
        self.making_arc_avoidance = False
        self.making_avoidance_turn = False
        self.avoiding_by_turning_right = False
        self.avoiding_by_turning_left = False
        self.obstacle_avoided = False
        self.default_speed = 30
        self.side_left_person_last_detected = False
        ## Used to measure distance
        self.percep_distance = Perceptron(acti_func = 'sigmoid', learning_rate=0.7, constant_weight=1, variable_weight = 30, name = 'Perceptron distance')
        #self.percep_far_distance = Perceptron(acti_func = 'sigmoid', learning_rate = 0.001, summation_gain=1, name = 'Perceptron far distance')
        #self.percep_close_distance = Perceptron(acti_func = 'sigmoid', learning_rate = 0.001, summation_gain=1, name = 'Perceptron close distance')
        self.person_width = 500
        self.perceived_focal = 306
        self.target_distance = 1.6
        self.previous_distance = 0 #is used when distance current distance is faulty
        self.is_first_run = True
        self.distances_in_4_frames = [] # Used to calculate average distance over 3 frams to make errors less significant
        self.person_detected_queue = [True]*15 # Used to check if person have been missing for 15 frames in a row
        self.previous_closest_obstacle = (0, 0)
        
        
        self.distances_for_test1 = []
        self.weights_for_test1 = []
        self.target_distance_for_test1 = []
        
        self.distances_for_test2 = []
        self.weights_for_test2 = []
        self.target_distance_for_test2 = []
        self.error_in_pixels_for_test2 = []
        
        self.distance_from_object_for_test3 = []
        self.avoidance_distance_for_test_3 = []
        self.reflex_distance_for_test3 = []
        self.test_iteration = 0
        self.current_object_distance = 0
        
        self.distances_for_test4 = []
        self.target_distance_for_test4 = []

        self.distance_from_object_for_test4 = []
        self.avoidance_distance_for_test_4 = []
        self.reflex_distance_for_test4 = []
        self.weight_distance_test4 = []
        self.weight_avoidance_test4 = []
        self.test4_iteration = 0
        self.test4_current_object_distance = 0
        
        
    def followTarget(self, middle_xy, distance_to_person, lidar_data):
        ## For following target       
        ## For maintaining distance
        #self.test1Distance(distance_to_person)
        #self.test2Follow(distance_to_person)

            
        
        if distance_to_person < 1:
            self.motor_speed_total_left = 0
            self.motor_speed_total_right = 0
            self.robot_is_stationary = True
            #self.is_first_run = True # Is needed if person walk out of frame and enters another place
            return [self.motor_speed_total_left, self.motor_speed_total_right]
        else:
            if self.robot_is_stationary == True:
                self.motor_speed_total_left = 20
                self.motor_speed_total_right = 20
                self.motor_speed_distance = 20
                self.robot_is_stationary = False
                return [self.motor_speed_total_left, self.motor_speed_total_right]    
        
        
        closest_obstacle = self.getClosestObjectAndDefineState(lidar_data)
        if closest_obstacle == (0, 0): ## Make sure to ignore new data if there is none
            closest_obstacle = self.previous_closest_obstacle
        else:
            self.previous_closest_obstacle = closest_obstacle       
        

        #self.test3Obstacle(closest_obstacle[1])
            
        if self.obstacle_in_reflex_range == False and self.first_reflex_turn == False:
            
            self.first_arc_turn = False
            
            toggleLED({})
            self.motor_speed_distance = self.getMaintainDistanceOutput(distance_to_person)
            if self.motor_speed_distance > 65:
                self.motor_speed_distance = 65
            
            change_for_obstacle_avoidance = self.getObstacleAvoidanceOutput(closest_obstacle)
            self.motor_speed_obstacle_avoidance_left = change_for_obstacle_avoidance[0]
            self.motor_speed_obstacle_avoidance_right = change_for_obstacle_avoidance[1]
            print('Speed for avoiding:', self.motor_speed_obstacle_avoidance_left, self.motor_speed_obstacle_avoidance_right)

            #print('Change in 0bstacle_speeds')
            if change_for_obstacle_avoidance != [0, 0]:
                self.making_avoidance_turn = True
                toggleLED([1,17], (0, 255, 0, 0))
            else:
                self.making_avoidance_turn = False
                toggleLED({})

                
            #print(self.motor_speed_obstacle_avoidance_left, self.motor_speed_obstacle_avoidance_right)

            
            change_for_follow_target = self.getFollowTargetOutout(middle_xy)
            #print(change_for_follow_target)
            self.motor_speed_left_follow = change_for_follow_target[0]
            self.motor_speed_right_follow = change_for_follow_target[1]
            
            self.motor_speed_total_left = self.motor_speed_distance + self.motor_speed_left_follow + self.motor_speed_obstacle_avoidance_left
            self.motor_speed_total_right = self.motor_speed_distance + self.motor_speed_right_follow + self.motor_speed_obstacle_avoidance_right
            print(self.motor_speed_total_left, self.motor_speed_total_right)
        elif self.obstacle_in_reflex_range == True and self.first_arc_turn == False:
            toggleLED([1, 17], (255, 0, 0, 0))
            print('Turning with reflex')
            reflex_motor_speeds = self.avoidObstacleWithReflex(closest_obstacle)
            self.motor_speed_total_left = reflex_motor_speeds[0]
            self.motor_speed_total_right = reflex_motor_speeds[1]
            
        elif (self.obstacle_in_reflex_range == False and self.first_reflex_turn == True):
            arc_motor_speeds = self.makeArcTurn(self.closest_obstacle_when_starting_reflex)
            
            self.motor_speed_total_left = arc_motor_speeds[0]
            self.motor_speed_total_right = arc_motor_speeds[1]
        
            
            
            
            


            
            
            
            
        #self.motor_speed_total_left = 0
        #self.motor_speed_total_right = 0
        self.test4Combined(distance_to_person, closest_obstacle[1])
        
        return [self.motor_speed_total_left, self.motor_speed_total_right]
        

    def getDistanceToPerson(self, object_width_in_pixels):
        #distance = (focal_length * real_height * image_height) / (object_height * sensor_height)
        distance = ((self.person_width * self.perceived_focal) / object_width_in_pixels) / 1000 #Divided på 1000 to get in meters
        
        self.distances_in_4_frames.insert(0, distance)
        if len(self.distances_in_4_frames) > 6:
            self.distances_in_4_frames.pop()
        
        average_distance = sum(self.distances_in_4_frames) / len(self.distances_in_4_frames) ## makes it more robust to large outburst errors
        #Make sure distance hasnt changed too much in one frame
#         if self.is_first_run == False:
#             if abs(distance - self.previous_distance) > 0.3: 
#                 distance = self.previous_distance
#             else:
#                 self.previous_distance = distance
#         else:
#             self.is_first_run = False
#             self.previous_distance = distance

            
        #print(object_width_in_pixels)
        return average_distance 
    
    def getPercievedFocal(self, P, D, H):
        F = (P * D) / H
        return F
    
    def getClosestObjectAndDefineState(self, lidar_data): #by state is meant the bools about how close the object is 

        if lidar_data == [0]:  # If no lidar data is avalible dont do anything
            return (0, 0)
        
        closest_obstacle = (0, 20000) #  Large number so scans always will be smaller (angle, dist)
        for angle in range(360):
            distance = lidar_data[angle]
            if distance > 0:                 # ignore initially ungathered data points
                if angle < self.angle_right or angle > self.angle_left :
                    if distance < self.start_avoiding_obstacle_dist:
                        if distance < closest_obstacle[1]:
                            closest_obstacle = (angle, distance)
        
        if closest_obstacle[1] < self.start_avoiding_obstacle_dist:
            self.obstacle_turning_range = True
            if closest_obstacle[1] < self.reflex_avoiding_obstacle_dist:
                self.obstacle_turning_range = False
                self.obstacle_in_reflex_range = True
        else:
            self.obstacle_in_reflex_range = False
            self.obstacle_turning_range = False
        
        print(closest_obstacle)
        return closest_obstacle        

    def getObstacleAvoidanceOutput(self, closest_obstacle):
        change_in_motors = [0, 0]
        error_in_distance = self.start_avoiding_obstacle_dist - closest_obstacle[1]
        if error_in_distance < 0:
            error_in_distance = 0 # Makes sure it is only activated when within 800
            self.motor_speed_obstacle_avoidance_left = 0 #Resets speeds to zero when there is no obstacle.
            self.motor_speed_obstacle_avoidance_right = 0
            self.avoiding_by_turning_left = False
            self.avoiding_by_turning_right = False
            
#         if self.making_avoidance_turn == True:
#             
#             if self.avoiding_by_turning_left == True:
#                 if closest_obstacle[1] > self.start_avoiding_obstacle_dist or closest_obstacle[0] < self.angle_left:
#                     self.obstacle_avoided = True
#                     self.making_avoidance_turn = False
#             elif self.avoiding_by_turning_right == True:
#                 if losest_obstacle[1] > self.start_avoiding_obstacle_dist or closest_obstacle[0] > self.angle.right:
#                     self.obstacle_avoided = True
#                     self.making_avoidance_turn = False
            
        if error_in_distance > 0:
            print('turning WITHOUT reflex')
            error_in_distance = error_in_distance / (self.start_avoiding_obstacle_dist / 10) # Make number between 0 and 2 for actifunc
            
            if closest_obstacle[0] < self.angle_right : # Obstacle is on the right side and therefore should turn right
                self.avoiding_obstacle_by_turning_left = True
                change_in_motors[1] = self.percep_obstacle.calcOutputAndUpdateWeighAvoidance(error_in_distance, learning_on=False)
                change_in_motors[0] = self.percep_obstacle.calcOutputAndUpdateWeighAvoidance(-error_in_distance, learning_on=False)

                
            elif closest_obstacle[0] > self.angle_left : # if obstacle is on the left
                self.avoiding_obstacle_by_turning_left = False
                change_in_motors[0] = self.percep_obstacle.calcOutputAndUpdateWeighAvoidance(error_in_distance, learning_on=False)
                change_in_motors[1] = self.percep_obstacle.calcOutputAndUpdateWeighAvoidance(-error_in_distance, learning_on=False)

                

        return change_in_motors
    
    def avoidObstacleWithReflex(self, closest_obstacle):
        motor_speeds = [0, 0]
        error_in_distance = 700 - closest_obstacle[1]
        error_in_distance = error_in_distance / 200 # Make number between 0 and 4
        if closest_obstacle[0] <= self.angle_right :
            average_speed = (self.motor_speed_total_left + self.motor_speed_total_right) / 2
            self.avoiding_obstacle_by_turning_left = True
            if average_speed > 30 and self.first_reflex_turn == False:
                self.closest_obstacle_when_starting_reflex = closest_obstacle[1]
                left_speed = average_speed - ((1 / 3) * average_speed)
                right_speed = average_speed
                self.first_reflex_turn = True
            elif average_speed > 30 and self.first_reflex_turn == True:
                left_speed = self.motor_speed_total_left
                right_speed = self.motor_speed_total_right
            else:
                left_speed = 20
                right_speed = 30
            motor_speeds = [left_speed, right_speed]
            self.percep_obstacle.calcOutputAndUpdateWeighAvoidance(error_in_distance, learning_on=True)
            
        elif closest_obstacle[0] > self.angle_left :
            average_speed = (self.motor_speed_total_left + self.motor_speed_total_right) / 2
            self.avoiding_obstacle_by_turning_left = False
            if average_speed > 30 and self.first_reflex_turn == False:
                self.closest_obstacle_when_starting_reflex = closest_obstacle[1]
                left_speed = average_speed 
                right_speed = average_speed - ((1 / 3) * average_speed)
                self.first_reflex_turn = True
            elif average_speed > 30 and self.first_reflex_turn == True:
                left_speed = self.motor_speed_total_left
                right_speed = self.motor_speed_total_right   
            else:
                left_speed = 30
                right_speed = 20
            motor_speeds = [left_speed, right_speed]
            self.percep_obstacle.calcOutputAndUpdateWeighAvoidance(error_in_distance, learning_on=True)
            
        return motor_speeds
        
            
                
    def getMaintainDistanceOutput(self, distance_to_target):
        current_error_distance = (distance_to_target - self.target_distance)

        speed_for_distance = self.percep_distance.calcOutputAndUpdateWeightDistance(distance_to_target, current_error_distance)
        return speed_for_distance
    
    def getFollowTargetOutout(self, middle_xy, error_margin_width = 10):
        current_error_pixels = middle_xy[0] - 160
        if current_error_pixels < -error_margin_width or current_error_pixels > error_margin_width:
            self.error_in_pixels_for_test2.append(current_error_pixels)
        else:
            self.error_in_pixels_for_test2.append(0)
        change_in_motors = [0, 0]
#         if self.motor_speed_left_follow == self.motor_speed_right_follow: # This means they have just stopped turning so make both zero.
#             self.motor_speed_left_follow = 0
#             self.motor_speed_right_follow = 0
        
        #if (current_error_pixels < -error_margin_width or current_error_pixels > error_margin_width):
        current_error_pixels /= 16
        if current_error_pixels > 0: # target is on right side
            change_in_motors[0] = self.percep_l_wheel.calcOutputAndUpdateWeightFollow(current_error_pixels)
            change_in_motors[1] = self.percep_r_wheel.calcOutputAndUpdateWeightFollow(-current_error_pixels) # Negative so i will turn less when close to target
            self.side_left_person_last_detected = False            
        else:
            current_error_pixels *= -1 # Flips the error so right wheel recieves positive
            change_in_motors[0] = self.percep_l_wheel.calcOutputAndUpdateWeightFollow(-current_error_pixels)
            change_in_motors[1] = self.percep_r_wheel.calcOutputAndUpdateWeightFollow(current_error_pixels) # Negative so i will turn less when close to target
            self.side_left_person_last_detected = True
                
#         else:
#             average_motor_speed = (self.motor_speed_left_follow + self.motor_speed_right_follow) / 2 # Makes sure the motor speed does not jerk
#             self.motor_speed_left_follow = 0
#             self.motor_speed_right_follow = 0
#             return [average_motor_speed, average_motor_speed]
            
        return change_in_motors
    
    def limitMotorSpeeds(self):
        if self.motor_speed_distance > 65:
            self.motor_speed_distance = 65
    
    def makeArcTurn(self, arc_radius):
        
        motor_speeds = [0, 0]
        left_wheel = 0
        right_wheel = 0
        arc_radius = arc_radius / 1000 # Get radius in meters (is just the distance to closest_object
        print('Arc radius = ', arc_radius)
        wheel_ratio = self.getArcRatio(arc_radius)
        average_motor_speed = max(self.motor_speed_total_left, self.motor_speed_total_right) 
        if self.avoiding_obstacle_by_turning_left == True and self.first_arc_turn == False:
            print('left')
            self.first_arc_turn = True
            self.first_reflex_turn = False #Reset this flag for next obstacle
            self.making_avoidance_turn = False
            left_wheel = average_motor_speed
            right_wheel = average_motor_speed * wheel_ratio
        elif self.avoiding_obstacle_by_turning_left == False and self.first_arc_turn == False:
            print('right')
            self.first_reflex_turn = False #Reset this flag for next obstacle
            self.making_avoidance_turn = False
            self.first_arc_turn = True
            left_wheel = average_motor_speed * wheel_ratio
            right_wheel = average_motor_speed
        elif self.first_arc_turn == True:
            #if self.obstacle_in_reflex_range == True:
             #   self.first_arc_turn = False # in case is turns too much towards the obstacle without having found its target again
            left_wheel = self.motor_speed_total_left
            right_wheel = self.motor_speed_total_right
                
            
            
        motor_speeds = [left_wheel, right_wheel]
        print(motor_speeds)
        return motor_speeds
        
    def getArcRatio(self, arc_radius):
        
        wheel_seperation = 0.1 ## in meters
        
        inside_wheel = 2 * math.pi * (arc_radius - (wheel_seperation / 2))
        outside_wheel = 2 * math.pi * (arc_radius + (wheel_seperation / 2))
        
        ratio = inside_wheel / outside_wheel
        return ratio
    
    def askAboutWeights(self):
        user_input = input('Press y to load previous weights, else press anything else')
        if user_input == 'y':
            f = open('previous_weights.txt', 'r+')

            weights = f.readlines()
            weights = [float(i) for i in weights]
            print(weights)
            self.percep_l_wheel.variable_weight = weights[0]
            self.percep_r_wheel.variable_weight = weights[1]
            self.percep_obstacle.variable_weight = weights[2]
            self.percep_distance.variable_weight = weights[3]
            self.test4_iteration = int(weights[4] + 1)
            
            
            f.close
        else:
            self.test4_iteration = 1
            
    def saveWeights(self):
        f = open('previous_weights.txt', 'r+')
        f.truncate(0)
        f.write(str(self.percep_l_wheel.variable_weight) + '\n')
        f.write(str(self.percep_r_wheel.variable_weight) + '\n')
        f.write(str(self.percep_obstacle.variable_weight) + '\n')
        f.write(str(self.percep_distance.variable_weight) + '\n')
        f.write(str(self.test4_iteration) + '\n')
        print(self.percep_l_wheel.variable_weight, self.percep_r_wheel.variable_weight, self.percep_obstacle.variable_weight, self.percep_distance.variable_weight)
        f.close
    
    def test3Obstacle(self, _object_distance):
        if _object_distance == 20000:
            _object_distance = 1000
        self.distance_from_object_for_test3.append(_object_distance)
        self.avoidance_distance_for_test_3.append(self.start_avoiding_obstacle_dist)
        self.reflex_distance_for_test3.append(self.reflex_avoiding_obstacle_dist)
        
        
    def test2Follow(self, _distance):
        self.distances_for_test2.append(_distance)
        self.target_distance_for_test2.append(self.target_distance)
        #self.error_in_pixels_for_test2. this is saved in another function
    def test1Distance(self, _distance):
        self.distances_for_test1.append(_distance)
        self.target_distance_for_test1.append(self.target_distance)
        #self.weights_for_test1(self.percep_distance.variable_weight) # this is saved in perceptron class file
    def test4Combined(self, _distance, _object_distance):
        if _object_distance == 20000:
            _object_distance = 1000
        self.distances_for_test4.append(_distance)
        self.target_distance_for_test4.append(self.target_distance)
        self.distance_from_object_for_test4.append(_object_distance)
        self.avoidance_distance_for_test_4.append(self.start_avoiding_obstacle_dist)
        self.reflex_distance_for_test4.append(self.reflex_avoiding_obstacle_dist)
        

    
    def save_test4(self):
        plt.plot(self.distance_from_object_for_test4, label='Actual Distance')
        plt.plot(self.avoidance_distance_for_test_4, label='Avoidance Distance')
        plt.plot(self.reflex_distance_for_test4, label='Reflex Distance')
        fignamefirst = 'test4-'
        fignamelast = 'figdistavoid.png'
        figname = fignamefirst + str(self.test4_iteration) + fignamelast
        plt.savefig(figname)
        plt.clf()
        fignamefirst = 'test4-'
        fignamelast = 'figavoidweight.png'
        figname = fignamefirst + str(self.test4_iteration) + fignamelast
        plt.plot(self.percep_obstacle.weights_for_avoidance_percep_test4)
        plt.savefig(figname)
        plt.clf()
        fignamefirst = 'test4-'
        fignamelast = 'figdist.png'
        figname = fignamefirst + str(self.test4_iteration) + fignamelast
        plt.plot(self.distances_for_test4, label='Actual Distance')
        plt.plot(self.target_distance_for_test4, label='Target Distance')
        plt.savefig(figname)
        plt.clf()
        fignamefirst = 'test4-'
        fignamelast = 'figdistweight.png'
        figname = fignamefirst + str(self.test4_iteration) + fignamelast
        plt.plot(self.percep_distance.weights_for_dist_percep_test4)
        plt.savefig(figname)
        
        filename = 'test4-'
        filetype = '.csv'
        filename = filename + str(self.test4_iteration) + filetype
        print(filename)
        
        file = open(filename, 'w')
        file.truncate()
        with file:
            writer = csv.writer(file)
            writer.writerow(self.distance_from_object_for_test4)
            writer.writerow(self.avoidance_distance_for_test_4)
            writer.writerow(self.reflex_distance_for_test4)
            writer.writerow(self.percep_obstacle.weights_for_avoidance_percep_test4)
            writer.writerow(self.distances_for_test4)
            writer.writerow(self.target_distance_for_test4)
            writer.writerow(self.percep_distance.weights_for_dist_percep_test4)
        
    
    def save_test3(self):
        plt.plot(self.distance_from_object_for_test3, label='Actual Distance')
        plt.plot(self.avoidance_distance_for_test_3, label='Avoidance Distance')
        plt.plot(self.reflex_distance_for_test3, label='Reflex Distance')
        fignamefirst = 'test3-'
        fignamelast = 'figdist.png'
        figname = fignamefirst + str(self.test_iteration) + fignamelast
        plt.savefig(figname)
        plt.clf()
        fignamefirst = 'test3-'
        fignamelast = 'figweight.png'
        figname = fignamefirst + str(self.test_iteration) + fignamelast
        plt.plot(self.percep_obstacle.weights_for_test3)
        plt.savefig(figname)
        
        filename = 'test3-'
        filetype = '.csv'
        filename = filename + str(self.test_iteration) + filetype
        print(filename)
        
        file = open(filename, 'w')
        file.truncate()
        with file:
            writer = csv.writer(file)
            writer.writerow(self.distance_from_object_for_test3)
            writer.writerow(self.avoidance_distance_for_test_3)
            writer.writerow(self.reflex_distance_for_test3)
            writer.writerow(self.percep_obstacle.weights_for_test3)
        
        
        
    def save_test2(self):
        plt.plot(self.distances_for_test2, label='Actual Distance')
        plt.plot(self.target_distance_for_test2, label='Target Distance')
        plt.savefig('test2figdist.png')
        plt.clf()
        plt.plot(self.percep_distance.weights_for_test2)
        plt.savefig('test2figweights.png')
        plt.clf()
        plt.plot(self.error_in_pixels_for_test2, label='Error In Pixels')
        plt.savefig('test2figpixel.png')
        
        file = open('test2.csv', 'w')
        file.truncate()
        with file:
            writer = csv.writer(file)
            writer.writerow(self.distances_for_test2)
            writer.writerow(self.target_distance_for_test2)
            writer.writerow(self.percep_distance.weights_for_test2)
            writer.writerow(self.error_in_pixels_for_test2)
            
    def save_test1(self):
        plt.plot(self.distances_for_test1, label='Actual Distance')
        plt.plot(self.target_distance_for_test1, label='Target Distance')
        plt.savefig('test1fig.png')
        plt.clf()
        plt.plot(self.percep_distance.error_in_dist_test1)
        plt.savefig('test1figerror.png')
        plt.clf()
        plt.plot(self.percep_distance.derrivative_error_test1)
        plt.savefig('test1figderrivative.png')
        plt.clf()
        plt.plot(self.percep_distance.weights_for_test1)
        plt.savefig('test1figweights.png')
        
        


        file = open('test1.csv', 'w')
        file.truncate()
        with file:
            writer = csv.writer(file)
            writer.writerow(self.distances_for_test1)
            writer.writerow(self.target_distance_for_test1)
            writer.writerow(self.percep_distance.weights_for_test1)

        
        