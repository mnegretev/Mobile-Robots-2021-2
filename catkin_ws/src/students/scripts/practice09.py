#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-2
# PRACTICE 09 - PERCEPTRON TRAINING BY GRADIENT RULE
#
# Instructions:
# Complete the code to train a perceptron using the gradient rule.
# Perceptron's output should indicate wheter a given input image
# corresponds to the trained digit or not. 
#

import sys
import numpy
import cv2
import math
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError

NAME = "APELLIDO_PATERNO_APELLIDO_MATERNO"

def evaluate(weights, image):
    #
    # TODO:
    # Calculate the output of perceptron given an image.
    # Use of numpy.dot is strongly recommended to save execution time.
    # Return perceptron output.
    #    
    return 0.5

def train_perceptron(weights, images, labels, desired_digit):
    print "Training perceptron for digit " + str(desired_digit) + " with " + str(len(images)) + " images. "
    #
    # TODO:
    # Train the perceptron (array of weights) given a set of training images, a set of labels (corresponding digits)
    # and a desired digit to be trained.
    # Perceptron is an array of floats representing the input weights and threshold
    # Last value in 'weights' corresponds to threshold.
    # You can use gradient rule or perceptron rule, but gradient rule is recommended.
    # Use of numpy.dot, numpy.add, numpy.zeros, numpy.asarray and numpy.linalg.norm is suggested.
    # Return the trained weights.
    #
    tol      = 0.001                                   #Max gradient magnitude to consider a local minimum
    attempts = 5000                                    #Max number of training iterations
    epsilon  = 0.2/len(weights)                        #Constant to ponderate gradient
    weights  = numpy.asarray(weights)                  #Array of perceptron's weights
    inputs   = [numpy.asarray(d+[-1]) for d in images] #Threshold is a weight whose corresponding input is always -1

    #
    # WHILE |gradient| > tol and attempts > 0 and not rospy.is_shutdown():
    #     Set gradient to vector zero
    #     FOR EACH x IN inputs:
    #         y_hat = Perceptron's output for input signal 'x' with the current weights
    #         y     = Desired output (1 if label for x corresponds to the desired digit. 0, otherwise)
    #         g_j   = Gradient term corresponding to input 'x'  
    #         gradient = gradient + g_j
    #     weights = weights - epsilon*gradient
    #     attempts = attempts - 1
    #     
    return weights

def load_dataset_digit(file_name):
    #It is assumed the file contains 1000 images with 28x28 grayscale pixels each one.
    print "Loading data set from file " +  file_name
    f_data = [ord(c)/255.0 for c in open(file_name, "rb").read(784000)]
    images = []
    for i in range(1000):
        images.append(f_data[784*i:784*(i+1)])
    print "Loaded " + str(len(images)) + " images."
    return images

def load_dataset_all_digits(folder):
    print "Loading data set from " + folder
    if not folder.endswith("/"):
        folder += "/"
    training_dataset = []
    training_labels  = []
    testing_dataset  = []
    testing_labels   = []
    for i in range(10):
        digit_dataset = load_dataset_digit(folder + "data" + str(i))
        training_dataset += digit_dataset[0:len(digit_dataset)/2]
        training_labels  += [i for j in range(len(digit_dataset)/2)]
        testing_dataset  += digit_dataset[len(digit_dataset)/2:len(digit_dataset)]
        testing_labels   += [i for j in range(len(digit_dataset)/2)]
    return training_dataset, training_labels, testing_dataset, testing_labels

def main():
    print "PRACTICE 09 - " + NAME
    rospy.init_node("practice09")
    rospack = rospkg.RosPack()
    dataset_folder = rospack.get_path("bring_up") + "/handwriting_digits/"
    if rospy.has_param("~dataset_folder"):
        dataset_folder = rospy.get_param("~dataset_folder")
    desired_digit = 0
    if rospy.has_param("~digit"):
        desired_digit = rospy.get_param("~digit")

    training_dataset, training_labels, testing_dataset, testing_labels = load_dataset_all_digits(dataset_folder)
    perceptron = [0 for i in range(784+1)]
    perceptron = train_perceptron(perceptron, training_dataset, training_labels, desired_digit)
    loop = rospy.Rate(10)
    img = testing_dataset[numpy.random.randint(0, 4999)]
    print("")
    print("Press digit key to test perceptron...")
    while not rospy.is_shutdown():
        test_digit = cv2.waitKey(10) - 48
        if test_digit >= 0 and test_digit <= 9:
            img = testing_dataset[test_digit*500 + numpy.random.randint(0, 499)]
            y = evaluate(perceptron, img + [-1])
            print "Perceptron output: " + str(y)
            if y > 0.5:
                print("Image corresponds to trained digit.")
            else:
                print("Image does not correspond to the trained digit.")
        cv2.imshow("Digit", numpy.reshape(numpy.asarray(img, dtype="float32"), (28,28,1)))
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

