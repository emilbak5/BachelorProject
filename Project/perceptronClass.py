import numpy as np

class Perceptron:
    def __init__(self, nr_of_inputs = 2, learning_rate = 0.1, variable_weight = 1, constant_weight = 1):
        self.nr_of_inputs = nr_of_inputs
        self.learning_rate = learning_rate
        self.variable_weight = variable_weight
        self.constant_weight = constant_weight
        self.summation_gain = 1
        self.previous_reflex = 0
        self.current_reflex = 0

    def hyperbolicActiFunc(self, sum_of_weights_and_inputs):
        return_value = (np.exp(sum_of_weights_and_inputs) - np.exp(-sum_of_weights_and_inputs)) / (np.exp(sum_of_weights_and_inputs) + np.exp(-sum_of_weights_and_inputs))
        return return_value
    
    def sigmoidActiFunc(self, sum_of_weights_and_inputs):
        return_value = 1 / (1 + np.exp(-sum_of_weights_and_inputs))
        return return_value

    # if there is only 2 inputs distance_input is will not affect the output.
    def calcOutputAndUpdateWeight(self, predictive_input, distance_input = 0):
        # Calculate output
        summation = self.summation_gain * ((predictive_input * self.variable_weight) + (self.current_reflex * self.constant_weight) + (distance_input * self.constant_weight))
        #output = self.sigmoidActiFunc(summation)
        output = self.hyperbolicActiFunc(summation)

        # Update weight
        self.variable_weight += self.learning_rate * abs(predictive_input) * (abs(self.current_reflex - self.previous_reflex))
        print("weight: ", self.variable_weight)
        self.previous_reflex = self.current_reflex
        self.current_reflex = predictive_input
        
        return output