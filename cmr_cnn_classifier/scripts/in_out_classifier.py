#!/usr/bin/env python

##@package in_out_classifier
#Implements a class for image classification based on ResNet and the places
#dataset


import torch
from torch.autograd import Variable as V
import torchvision.models as models
from torchvision import transforms as trn
from torch.nn import functional as F
import os
import numpy as np
from scipy.misc import imresize as imresize
import cv2
from PIL import Image
from matplotlib import cm
import rospkg

##in_out_classifier class.
class in_out_classifier:

    ##The constructor. Loads the model
    def __init__(self):

        self.load()

    ## Classify method to classify an cv2 image
    #  @param self The object pointer.
    #  @param image cv2 RGB image
    #  @return 0 if classified as indoor, 1 if classified as outdoor
    def classify(self, image):

        img = Image.fromarray(image)
        input_img = V(self.tf(img).unsqueeze(0))
        # forward pass
        logit = self.model.forward(input_img)
        h_x = F.softmax(logit, 1).data.squeeze()
        probs, idx = h_x.sort(0, True)
        probs = probs.numpy()
        idx = idx.numpy()

        io_image = np.mean(self.labels_IO[idx[:10]]) # vote for the indoor or outdoor

        if io_image < 0.5:
            return 0 # indoor
        else:
            return 1 # outdoor

    ## Initialize object instances
    #  @param self The object pointer.
    def load(self):
        # load the labels
        self.rospack = rospkg.RosPack()
        self.labels_IO = self.load_labels()

        # load the model
        self.features_blobs = []
        self.model = self.load_model()

        # load the transformer
        self.tf = self.returnTF()

    ## Loads the indoor outdoor labels
    #  @param self The object pointer.
    def load_labels(self):
        # prepare all the labels
        # scene category relevant

        # indoor and outdoor relevant
        IO_path = self.rospack.get_path('cmr_cnn_classifier')+'/scripts/places/'
        file_name_IO = IO_path + 'IO_places365.txt'
        with open(file_name_IO) as f:
            lines = f.readlines()
            labels_IO = []
            for line in lines:
                items = line.rstrip().split()
                labels_IO.append(int(items[-1]) -1) # 0 is indoor, 1 is outdoor
        labels_IO = np.array(labels_IO)
        return labels_IO

    def hook_feature(self,module, input, output):
        self.features_blobs.append(np.squeeze(output.data.cpu().numpy()))

    def returnTF(self):
        # load the image transformer
        tf = trn.Compose([
            trn.Resize((224, 224)),
            trn.ToTensor(),
            trn.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])
        return tf

    ## Loads the pytorch model
    #  @param self The object pointer.
    def load_model(self):
        # this model has a last conv feature map as 14x14

        model_file = 'wideresnet18_places365.pth.tar'
        if not os.access(model_file, os.W_OK):
            os.system('wget http://places2.csail.mit.edu/models_places365/' + model_file)
            #os.system('wget https://raw.githubusercontent.com/csailvision/places365/master/wideresnet.py')

        import wideresnet
        model = wideresnet.resnet18(num_classes=365)
        checkpoint = torch.load(model_file, map_location=lambda storage, loc: storage)
        state_dict = {str.replace(k,'module.',''): v for k,v in checkpoint['state_dict'].items()}
        model.load_state_dict(state_dict)
        model.eval()

        # hook the feature extractor
        features_names = ['layer4','avgpool'] # this is the last conv layer of the resnet
        for name in features_names:
            model._modules.get(name).register_forward_hook(self.hook_feature)
        return model
