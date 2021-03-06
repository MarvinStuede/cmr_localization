#!/bin/bash
#Script to download and uncompress a bag file for testing
ppath=$(rospack find cmr_localization)
bagname=cmg_imes_flur_test_u.bag
wget --no-verbose --content-disposition https://high-seas.projekt.uni-hannover.de/f/f0105ffbe1044edeb731/?dl=1 -O "$ppath/test/$bagname"
#rosbag decompress "$ppath/test/$bagname"
