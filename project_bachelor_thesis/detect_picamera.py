# python3
#
# Copyright 2019 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Example using TF Lite to detect objects with the Raspberry Pi camera."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import io
import re
import time

from annotation import Annotator

import numpy as np
import picamera
import csv
import multiprocessing
from time import sleep


from PIL import Image

from adafruit_rplidar import RPLidar

from tflite_runtime.interpreter import Interpreter
from DIYfunctionsAndClasses import *
from perceptronSystemClass import *
from tflite_runtime.interpreter import load_delegate

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 320

motor_default_speed = 30
perceptron_network = perceptronSystem()
perceptron_network.askAboutWeights()
frame_times_for_coral_test = []



    

def load_labels(path):
    """Loads the labels file. Supports files with or without index numbers."""
    with open(path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        labels = {}
        for row_number, content in enumerate(lines):
            pair = re.split(r'[:\s]+', content.strip(), maxsplit=1)
            if len(pair) == 2 and pair[0].strip().isdigit():
                labels[int(pair[0])] = pair[1].strip()
            else:
                labels[row_number] = pair[0].strip()
    return labels


def set_input_tensor(interpreter, image):
    """Sets the input tensor."""
    tensor_index = interpreter.get_input_details()[0]['index']
    input_tensor = interpreter.tensor(tensor_index)()[0]
    input_tensor[:, :] = image


def get_output_tensor(interpreter, index):
    """Returns the output tensor at the given index."""
    output_details = interpreter.get_output_details()[index]
    tensor = np.squeeze(interpreter.get_tensor(output_details['index']))
    return tensor


def detect_objects(interpreter, image, threshold):
    """Returns a list of detection results, each a dictionary of object info."""
    set_input_tensor(interpreter, image)
    interpreter.invoke()

    # Get all output details
    boxes = get_output_tensor(interpreter, 0)
    classes = get_output_tensor(interpreter, 1)
    scores = get_output_tensor(interpreter, 2)
    count = int(get_output_tensor(interpreter, 3))

    results = []
    for i in range(count):
        if scores[i] >= threshold:
            result = {
                'bounding_box': boxes[i],
                'class_id': classes[i],
                'score': scores[i]
            }
            results.append(result)
    return results


def annotate_objects(annotator, results, labels):
    """Draws the bounding box and label for each object in the results."""
    middle_xy = [0, 0, 0]
    global perceptron_network
    for obj in results:
        # Convert the bounding box figures from relative coordinates
        # to absolute coordinates based on the original resolution
        ymin, xmin, ymax, xmax = obj['bounding_box']
        xmin = int(xmin * CAMERA_WIDTH)
        xmax = int(xmax * CAMERA_WIDTH)
        ymin = int(ymin * CAMERA_HEIGHT)
        ymax = int(ymax * CAMERA_HEIGHT)

        # Custom get middle of fram
        if labels[obj['class_id']] == 'person':
            middle_x = int(xmin + ((xmax - xmin) / 2))
            middle_y = int(ymin + ((ymax - ymin) / 2))
            middle_xy[0] = middle_x
            middle_xy[1] = middle_y
            # middle_xy[2] = ymax - ymin # Object height.
            middle_xy[2] = xmax - xmin  # object width
            #print('Person detected at: ', middle_xy)
            annotator.drawDot(middle_x, middle_y)
            annotator.drawLine(middle_x, middle_y)
            print('Error in pixels:', 160 - middle_x)
            annotator._draw.line([(160, 0), (160, 320)], fill = 'blue', width = 0)
            annotator.bounding_box([xmin, ymin, xmax, ymax])
            annotator.text([xmin, ymin], '%s\n%.2f' % (labels[obj['class_id']], obj['score']))
            if perceptron_network.person_detected == False:
                motorInit()
                motorSpeedLeft(1, perceptron_network.default_speed)
                motorSpeedRight(1, perceptron_network.default_speed)
                perceptron_network.person_detected = True

        # Overlay the box, label, and score on the camera preview
#     annotator.bounding_box([xmin, ymin, xmax, ymax])
#     annotator.text([xmin, ymin],
#                    '%s\n%.2f' % (labels[obj['class_id']], obj['score']))
    return middle_xy


def main(lidar_data_queue):
    global perceptron_network
    

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--model', help='File path of .tflite file.', required=True)
    parser.add_argument(
        '--labels', help='File path of labels file.', required=True)
    parser.add_argument(
        '--threshold',
        help='Score threshold for detected objects.',
        required=False,
        type=float,
        default=0.6)
    args = parser.parse_args()

    labels = load_labels(args.labels)
    interpreter = Interpreter(args.model, experimental_delegates=[
                              load_delegate('libedgetpu.so.1.0')])
    interpreter.allocate_tensors()
    _, input_height, input_width, _ = interpreter.get_input_details()[0]['shape']
    print(input_height, input_width)

    count = 0
   
    with picamera.PiCamera(resolution=(CAMERA_WIDTH, CAMERA_HEIGHT)) as camera:
        camera.rotation = 180
        camera.start_preview()
        try:
            stream = io.BytesIO()
            annotator = Annotator(camera)
            for _ in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
                start_time = time.monotonic()
                stream.seek(0)
                image = Image.open(stream).convert('RGB').resize(
                    (input_width, input_height), Image.ANTIALIAS)

#                 start_time = time.monotonic()
                results = detect_objects(interpreter, image, args.threshold)
                #elapsed_ms = (time.monotonic() - start_time) * 1000

                annotator.clear()
                middle_xy = annotate_objects(annotator, results, labels)
                #annotator.text([5, 0], '%.1fms' % (elapsed_ms))
                # annotator.update()                

                                    
                if perceptron_network.person_detected == True:  # this only changes the first time a person is detected
                    data = []
                    if lidar_data_queue.empty() is False:
                        data = lidar_data_queue.get()
                    else:
                        data = [0]
                        
                    if middle_xy[0] != 0:# or ((middle_xy[0] < 300 and middle_xy != 0) and perceptron_network.first_arc_turn == True) or (middle_xy[0] > 20 and perceptron_network.first_arc_turn == True):

                            
                        perceptron_network.person_detected_queue.insert(0, True)  # keep track og frames with person in it
                        perceptron_network.person_detected_queue.pop()
                        
                        object_width = middle_xy[2]
                        print('object width: ',object_width)
                        if perceptron_network.first_arc_turn == True:
                            object_width = 80
                        distance_to_target = perceptron_network.getDistanceToPerson(object_width)
                        print('distance = ', distance_to_target)
                        new_motor_speeds = perceptron_network.followTarget(middle_xy, distance_to_target, data)
                        motorSpeedLeft(1, round(perceptron_network.motor_speed_total_left))
                        motorSpeedRight(1, round(perceptron_network.motor_speed_total_right))
                        print("Left motor: ", round(perceptron_network.motor_speed_total_left), " Right motor: ", round(perceptron_network.motor_speed_total_right))
                        print('..........................................')
                        
                    elif perceptron_network.first_arc_turn == True or (perceptron_network.first_reflex_turn == True):
                        arc_motor_speeds = perceptron_network.makeArcTurn(perceptron_network.reflex_avoiding_obstacle_dist + 400)
                        perceptron_network.motor_speed_total_left = arc_motor_speeds[0]
                        perceptron_network.motor_speed_total_right = arc_motor_speeds[1]
                        motorSpeedLeft(1, arc_motor_speeds[0])
                        motorSpeedRight(1, arc_motor_speeds[1])
                        print('second', perceptron_network.motor_speed_total_left)
                    elif perceptron_network.first_arc_turn == True or (perceptron_network.making_avoidance_turn == True):
                        arc_motor_speeds = perceptron_network.makeArcTurn(perceptron_network.start_avoiding_obstacle_dist + 300)
                        perceptron_network.motor_speed_total_left = arc_motor_speeds[0]
                        perceptron_network.motor_speed_total_right = arc_motor_speeds[1]
                        motorSpeedLeft(1, arc_motor_speeds[0])
                        motorSpeedRight(1, arc_motor_speeds[1])
                        print('second', perceptron_network.motor_speed_total_left)
                        
                    else:
                        perceptron_network.person_detected_queue.insert(0, False)
                        perceptron_network.person_detected_queue.pop()
                        # Is all the last 15 frames was without a person
                        if any(perceptron_network.person_detected_queue) == False:
                            perceptron_network.motor_speed_total_left = 0
                            perceptron_network.motor_speed_total_right = 0
                            perceptron_network.motor_speed_distance = 0
                            print("Locating target....")

                            perceptron_network.robot_is_stationary = True
                            if perceptron_network.side_left_person_last_detected == True:
                                motorSpeedLeft(0, 19)
                                motorSpeedRight(1, 19)
                            elif perceptron_network.side_left_person_last_detected == False:
                                motorSpeedLeft(1, 19)
                                motorSpeedRight(0, 19)

                        # For calibrating the focal length
    #                 focal = perceptron_network.getPercievedFocal(object_width, 2000, 500)
    #                 print('focal = ', focal)
                elapsed_ms = (time.monotonic() - start_time) * 1000
                annotator.text([5, 0], '%.1fms' % (elapsed_ms))
                annotator.update()
                frame_times_for_coral_test.append(elapsed_ms)

                #print(perceptron_network.getPercievedFocal(object_height, distance_test, person_height))

                stream.seek(0)
                stream.truncate()
        except KeyboardInterrupt:
            print('Saving distance data and shutting down')
            motorSpeedLeft(1, 0)
            motorSpeedRight(1, 0)
            toggleLED({})
            
            frame_average = sum(frame_times_for_coral_test) / len(frame_times_for_coral_test)
            #perceptron_network.save_test1()
            #perceptron_network.save_test2()
            #perceptron_network.save_test3()
            #perceptron_network.save_test4()
            perceptron_network.saveWeights()



            
            
            

#        makePlots(perceptron_network.percep_l_wheel.weights_for_test, perceptron_network.percep_r_wheel.weights_for_test, perceptron_network.percep_far_distance.weights_for_test, perceptron_network.distances_for_test)

#         file = open('distances.csv', 'w')
#         file.truncate()
#         with file:
#             writer = csv.writer(file)
#             writer.writerow(perceptron_network.distances_for_test)
#             writer.writerow(perceptron_network.percep_l_wheel.weights_for_test)
#             writer.writerow(perceptron_network.percep_r_wheel.weights_for_test)
#             writer.writerow(perceptron_network.percep_distance.weights_for_test)

        finally:
            camera.stop_preview()


if __name__ == '__main__':
    
    
    lidar_data_queue = multiprocessing.Queue()
    process_lidar = multiprocessing.Process(target=getLidarScan, args=(lidar_data_queue,))
    process_main = multiprocessing.Process(target=main, args=(lidar_data_queue,))
    process_lidar.start()
    process_main.start()
    process_lidar.join()
    process_main.join()
    
