import os
from os.path import join
import argparse
import rospy

import argparse
import numpy as np

from ros_utils import *
from nus_utils import LABEL_REMAP_TO_RGB, LABEL_REMAP

# Parse arguments from command line
parser = argparse.ArgumentParser(description='Visualize point cloud and detections')
parser.add_argument('--pc', type=str, default="data/leva/test", help='Path to point cloud directory')
parser.add_argument('--dets', type=str, default="data/leva/prediction", help='Path to detections directory')

def filter_dets(dets, remap=True):
    """
    Filter out detections before visualization
    
    """
    if remap:
        dets[:, -2] = [LABEL_REMAP[int(label)] for label in dets[:, -2]]
    
    # Filter out detections with low confidence
    dets = dets[dets[:, -1] >= 0.1]

    # Filter out detections in out of domain classes
    dets = dets[np.isin(dets[:, -2], [0])]

    return dets

def main(args):
    # Create ROS node
    rospy.init_node('LevaDetr', anonymous=True)

    pc_pub = rospy.Publisher("/pointcloud", PointCloud2, queue_size=5)
    dt_pub = rospy.Publisher("/detections", MarkerArray, queue_size=5)

    pc_dir = args.pc
    dets_dir = args.dets

    pc_files = [ file for file in os.listdir(pc_dir) if file.endswith('.bin') ]
    # sort
    pc_files = sorted(pc_files, key=lambda x: int(x.split('.')[0].split('_')[-1]))
    dets_files = [ file.replace('bin', 'txt') for file in pc_files ]
    rate = rospy.Rate(1)

    # Load point cloud and detections
    for pc_file, dets_file in zip(pc_files, dets_files):
        pc_path = join(pc_dir, pc_file)
        dt_path = join(dets_dir, dets_file)

        pc = np.fromfile(pc_path, dtype=np.float32).reshape(-1, 5) # x y z intensity timestamp
        dets = np.loadtxt(dt_path, dtype=np.float32)

        dets = filter_dets(dets, remap=True)

        # Publish point cloud and detections to ROS
        ts = rospy.Time.now()

        pub_pc_to_rviz(pc, pc_pub, ts, point_type="x y z i t", frame_id="os_sensor", seq=0, publish=True)
        pub_dt_to_rviz(dets, dt_pub, ts, frame_id="os_sensor", namespace="os_sensor", colormap=LABEL_REMAP_TO_RGB, publish=True)
        # rate.sleep()
        # pub_pc_to_rviz(pc, pc_pub, ts, point_type="x y z i t", frame_id="os_sensor", seq=0, publish=True)
        # pub_dt_to_rviz(dets, dt_pub, ts, frame_id="os_sensor", namespace="os_sensor", colormap=LABEL_REMAP_TO_RGB, publish=True)
        # import pdb; pdb.set_trace()

        rate.sleep()

if __name__ == "__main__":
    args = parser.parse_args()
    main(args)