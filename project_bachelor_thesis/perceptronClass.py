import numpy as np

class Perceptron:
    def __init__(self, acti_func = 'sigmoid', summation_gain = 1, nr_of_inputs = 2, learning_rate = 0.1, variable_weight = 1, constant_weight = 1, name = 'default'):
        self.nr_of_inputs = nr_of_inputs
        self.learning_rate = learning_rate
        self.variable_weight = variable_weight
        self.constant_weight = constant_weight
        self.summation_gain = summation_gain
        self.previous_reflex = 0.1
        self.current_reflex = 0.1
        self.name = name
        self.acti_func = acti_func
        self.weights_for_test1 = []
        self.error_in_dist_test1 = []
        self.derrivative_error_test1 = []

        self.weights_for_test2 = []
        self.weights_for_test3 = []
        
        self.weights_for_dist_percep_test4 = []
        self.weights_for_avoidance_percep_test4 = []

    def hyperbolicActiFunc(self, sum_of_weights_and_inputs):
        return_value = ((np.exp(sum_of_weights_and_inputs)) - (np.exp(-sum_of_weights_and_inputs))) / (np.exp(sum_of_weights_and_inputs) + np.exp(-sum_of_weights_and_inputs))
        return return_value
    
    def sigmoidActiFunc(self, sum_of_weights_and_inputs, max_output = 1, uod = 1):
        return_value = max_output / (1 + np.exp(-sum_of_weights_and_inputs * uod)) ##0.25 spreads out the "universe of discourse so input range is larger
        return return_value
    def justGainActiFunc(self, sum_of_weights_and_inputs, gain = 1):
        return_value = gain * sum_of_weights_and_inputs
        return return_value

    # if there is only 2 inputs distance_input is will not affect the output.
    def calcOutputAndUpdateWeightFollow(self, error_in_distance):
        # Calculate output
        output = self.summation_gain * self.sigmoidActiFunc(error_in_distance, max_output=30, uod=0.25)
        
        return output
    
    def calcOutputAndUpdateWeightDistance(self, distance, error_in_distance):
        summation = (distance * self.variable_weight) + (error_in_distance * self.constant_weight)

        output = self.summation_gain * self.justGainActiFunc(summation, gain=1)
            
        self.variable_weight += self.learning_rate * error_in_distance # * distance
        print(self.current_reflex - self.previous_reflex)
        print(self.name, "weight: ", self.variable_weight)
        self.previous_reflex = self.current_reflex
        self.current_reflex = error_in_distance
        self.weights_for_test1.append(self.variable_weight)
        #self.error_in_dist_test1.append(error_in_distance)
        #self.derrivative_error_test1.append(self.learning_rate * distance * error_in_distance * (abs(self.current_reflex - self.previous_reflex)))
        self.weights_for_dist_percep_test4.append(self.variable_weight)
        
        return output
    
    def calcOutputAndUpdateWeighAvoidance(self, prediction, learning_on = False):
        
        summation = (prediction * self.variable_weight) + (self.current_reflex * self.constant_weight)
        output = self.summation_gain * self.sigmoidActiFunc(summation, max_output=20, uod=0.2)
        
        if learning_on == True:
            self.variable_weight += self.learning_rate * prediction * abs((self.current_reflex - self.previous_reflex))
        print(self.name, "weight: ", self.variable_weight)
        self.weights_for_test3.append(self.variable_weight)
        self.previous_reflex = self.current_reflex
        self.current_reflex = prediction
        self.weights_for_avoidance_percep_test4.append(self.variable_weight)
        
        return output
        
        

