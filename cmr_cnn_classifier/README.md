# **cmr_cnn_classifier**: CNN for visual environment detection
This package contains a node to run a Resnet classifier trained on the [Places365 dataset](http://places2.csail.mit.edu).
Based on the current front camera image, the node does a classification and then publishes a std_msgs/String either with *indoor* or *outdoor* as value.

Start the node:
```
roslaunch cmr_cnn_classifier in_out_classifier.launch
```
