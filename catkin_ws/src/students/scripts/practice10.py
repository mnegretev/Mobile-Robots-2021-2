#!/usr/bin/env python
import cv2
import sys
import random
import numpy
import rospy
import rospkg

NAME = "GUZMAN_MARTINEZ"

class NeuralNetwork(object):
    def __init__(self, layer_sizes):
        #
        # The list 'layer_sizes' indicates the number of neurons in each layer.
        # Remember that the first layer indicates the dimension of the inputs and thus,
        # there is no bias vector fot the first layer.
        # For this practice, layer_sizes should be something like [784, n2, n3, ..., nl, 10]
        # All weights are initialized with zero. In each layer we have a matrix of weights where
        # row j contains all the weights of j-th neuron in that layer. For this example,
        # the first matrix would be of order n2 x 784 and last matrix would be 10 x nl.
        #
        self.num_layers  = len(layer_sizes)
        self.layer_sizes = layer_sizes
        self.biases  = [numpy.random.randn(y,1) for y in layer_sizes[1:]]
        self.weights = [numpy.random.randn(y,x) for x,y in zip(layer_sizes[:-1], layer_sizes[1:])]
        
    def feedforward(self, x):
        #
        # This function gets the output of the network when input is 'x'.
        #
        for i in range(len(self.biases)):
            z = numpy.dot(self.weights[i], x) + self.biases[i]
            x = 1.0 / (1.0 + numpy.exp(-z))  #output of the current layer is the input of the next one
        return x

    def feedforward_verbose(self, x):
        #
        # This function gets the output of the network when input is 'x'.
        # Returns a list containing the output of each layer.
        #
        y = []
        y.append(x)
        for i in range(len(self.biases)):
            z = numpy.dot(self.weights[i], x) + self.biases[i]
            x = 1.0 / (1.0 + numpy.exp(-z))  #output of the current layer is the input of the next one
            y.append(x)
        return y

    def backpropagate(self, x, yt):
        #
        # Thir function returns a tuple (nabla_w, nabla_b) containing the gradient of cost function C w.r.t
        # each weight and bias of all the network. The gradient is calculated assuming only one training
        # example is given. The training example is given by the input 'x' and the corresponding label 'y'.
        #
        y = self.feedforward_verbose(x)
        nabla_b = [numpy.zeros(b.shape) for b in self.biases]
        nabla_w = [numpy.zeros(w.shape) for w in self.weights]
        delta = (y[-1] - yt)*y[-1]*(1 - y[-1])
        nabla_b[-1] = delta
        nabla_w[-1] = delta*y[-2].transpose()
        for i in range(2, self.num_layers):
            #delta = numpy.sum(delta)*y[-i]*(1.0 - y[-i])
            delta = numpy.dot(self.weights[-i+1].transpose(), delta)*y[-i]*(1.0 - y[-i])
            nabla_b[-i] = delta
            nabla_w[-i] = numpy.dot(delta,y[-i-1].transpose())
        return nabla_w, nabla_b

    def update_with_batch(self, batch, eta):
        #
        # This function exectutes gradient descend for the subset of examples
        # given by 'batch' with learning rate 'eta'
        # 'batch' is a list of training examples [(x,y), ..., (x,y)]
        #
        nabla_b = [numpy.zeros(b.shape) for b in self.biases]
        nabla_w = [numpy.zeros(w.shape) for w in self.weights]
        M = len(batch)
        for x,y in batch:
            if rospy.is_shutdown():
                break
            delta_nabla_w, delta_nabla_b = self.backpropagate(x,y)
            nabla_w = [nw+dnw for nw,dnw in zip(nabla_w, delta_nabla_w)]
            nabla_b = [nb+dnb for nb,dnb in zip(nabla_b, delta_nabla_b)]
        self.weights = [w-eta*nw/M for w,nw in zip(self.weights, nabla_w)]
        self.biases  = [b-eta*nb/M for b,nb in zip(self.biases , nabla_b)]
        return nabla_w, nabla_b

    def get_gradient_mag(self, nabla_w, nabla_b):
        n2 = [nw*nw for nw in nabla_w]
        b2 = [nb*nb for nb in nabla_b]
        mag_w = sum([numpy.sum(n) for n in n2])
        mag_b = sum([numpy.sum(b) for b in b2])
        return mag_w + mag_b

    def train_by_SGD(self, training_data, epochs, batch_size, eta):
        for j in range(epochs):
            if rospy.is_shutdown():
                break
            random.shuffle(training_data)
            batches = [training_data[k:k+batch_size] for k in range(0,len(training_data), batch_size)]
            for batch in batches:
                if rospy.is_shutdown():
                    break
                nabla_w, nabla_b = self.update_with_batch(batch, eta)
                sys.stdout.write("\rGradient magnitude: %f                " % (self.get_gradient_mag(nabla_w, nabla_b)))
                sys.stdout.flush()
            print("Epoch: " + str(j))
    #
    ### END OF CLASS
    #


def load_dataset(folder):
    print "Loading data set from " + folder
    if not folder.endswith("/"):
        folder += "/"
    training_dataset = []
    training_labels  = []
    testing_dataset  = []
    testing_labels   = []
    for i in range(10):
        f_data = [ord(c)/255.0 for c in open(folder + "data" + str(i), "rb").read(784000)]
        images = []
        for j in range(1000):
            img = numpy.asarray(f_data[784*j:784*(j+1)]).reshape([784,1])
            images.append(img)
        label = numpy.asarray([1 if i == j else 0 for j in range(10)]).reshape([10,1])
        training_dataset += images[0:len(images)//2]
        training_labels  += [label for j in range(len(images)//2)]
        testing_dataset  += images[len(images)//2:len(images)]
        testing_labels   += [label for j in range(len(images)//2)]
    return zip(training_dataset, training_labels), zip(testing_dataset, testing_labels)

def main():
    print("PRACTICE 10 - " + NAME)
    rospy.init_node("practice10")
    rospack = rospkg.RosPack()
    dataset_folder = rospack.get_path("bring_up") + "/handwriting_digits/"
    epochs        = 30
    batch_size    = 10
    learning_rate = 3.0
    
    if rospy.has_param("~dataset_folder"):
        dataset_folder = rospy.get_param("~dataset_folder")
    if rospy.has_param("~epochs"):
        epochs = rospy.get_param("~epochs")
    if rospy.has_param("~learning_rate"):
        learning_rate = rospy.get_param("~learning_rate") 

    if not dataset_folder.endswith("/"):
        dataset_folder += "/"
    training_dataset, testing_dataset = load_dataset(dataset_folder)
    
    try:
        saved_data = numpy.load(dataset_folder+"network.npz",allow_pickle=True)
        layers = [b.shape[0] for b in saved_data['b']]
        layers = [saved_data['w'][0].shape[1]] + layers
        nn = NeuralNetwork(layers)
        nn.biases  = saved_data['b']
        nn.weights = saved_data['w']
        print("Loading data from previously trained model with layers " + str(layers))
    except:
        nn = NeuralNetwork([784,30,10])
        pass
    
    nn.train_by_SGD(training_dataset, epochs, batch_size, learning_rate)
    numpy.savez("network",w=nn.weights, b=nn.biases)
    
    print("")
    print("Press key to test network or ESC to exit...")
    float_formatter = "{:.3f}".format
    numpy.set_printoptions(formatter={'float_kind':float_formatter})
    img,label = testing_dataset[numpy.random.randint(0, 4999)]
    cmd = cv2.waitKey(0)
    while cmd != 27 and not rospy.is_shutdown():
        img,label = testing_dataset[numpy.random.randint(0, 4999)]
        y = nn.feedforward(img).transpose()
        print("Perceptron output: " + str(y))
        print("Expected output  : " + str(label.transpose()))
        print("Recognized digit : " + str(numpy.argmax(y)))
        print("")
        cv2.imshow("Digit", numpy.reshape(numpy.asarray(img, dtype="float32"), (28,28,1)))
        cmd = cv2.waitKey(0)
    

if __name__ == '__main__':
    main()
