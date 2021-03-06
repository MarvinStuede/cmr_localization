# Helper script to replace tf names in a bag file
#!/usr/bin/env python
import rosbag
from tf.msg import tfMessage
base_path = '/home/stuede/Messdaten/kitti_odom_dataset/data/'
#bag_list = ['kitti_2011_09_30_drive_0018_synced.bag', 'kitti_2011_09_30_drive_0020_synced.bag', 'kitti_2011_09_30_drive_0027_synced.bag', 'kitti_2011_09_30_drive_0028_synced.bag', 'kitti_2011_09_30_drive_0033_synced.bag', 'kitti_2011_09_30_drive_0034_synced.bag']

bag_list = ['kitti_odometry_sequence_00.bag','kitti_odometry_sequence_02.bag','kitti_odometry_sequence_05.bag','kitti_odometry_sequence_06.bag','kitti_odometry_sequence_07.bag','kitti_odometry_sequence_08.bag','kitti_odometry_sequence_09.bag' ]

for bag in bag_list:
	print('Processing bag '+bag)
	with rosbag.Bag(base_path+bag+'.out', 'w') as outbag:
	    for topic, msg, t in rosbag.Bag(base_path+bag).read_messages():
		if topic == "/tf" and msg.transforms:
		    newList = [];
		    for m in msg.transforms:
			if m.header.frame_id == "camera_init":
			    m.child_frame_id = "camera_gray_left_gt"
			newList.append(m)
		    if len(newList)>0:
			msg.transforms = newList
			outbag.write(topic, msg, t)
		else:
		    outbag.write(topic, msg, t)
