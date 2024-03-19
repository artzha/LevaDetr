
import rospy
import numpy as np

from scipy.spatial.transform import Rotation as R

import std_msgs 
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

def pub_pc_to_rviz(pc, pc_pub, ts, point_type="x y z", frame_id="os_sensor", seq=0, publish=True):
    if not isinstance(ts, rospy.Time):
        ts = rospy.Time.from_sec(ts)

    def add_field(curr_bytes_np, next_pc, field_name, fields):
        """
        curr_bytes_np - expect Nxbytes array
        next_pc - expects Nx1 array
        datatype - uint32, uint16
        """
        field2dtype = {
            "x":    np.array([], dtype=np.float32),
            "y":    np.array([], dtype=np.float32),
            "z":    np.array([], dtype=np.float32),
            "i":    np.array([], dtype=np.float32),
            "t":    np.array([], dtype=np.uint32),
            "re":   np.array([], dtype=np.uint16),
            "ri":   np.array([], dtype=np.uint16),
            "am":   np.array([], dtype=np.uint16),
            "ra":   np.array([], dtype=np.uint32),
            "r":    np.array([], dtype=np.float32),
            "g":    np.array([], dtype=np.float32),
            "b":    np.array([], dtype=np.float32)
        }
        field2pftype = {
            "x": PointField.FLOAT32,  "y": PointField.FLOAT32,  "z": PointField.FLOAT32,
            "i": PointField.FLOAT32,  "t": PointField.UINT32,  "re": PointField.UINT16,  
            "ri": PointField.UINT16, "am": PointField.UINT16, "ra": PointField.UINT32,
            "r": PointField.FLOAT32, "g": PointField.FLOAT32, "b": PointField.UINT32
        }
        field2pfname = {
            "x": "x", "y": "y", "z": "z", 
            "i": "intensity", "t": "t", 
            "re": "reflectivity",
            "ri": "ring",
            "am": "ambient", 
            "ra": "range",
            "r": "r",
            "g": "g",
            "b": "b"
        }

        #1 Populate byte data
        dtypetemp = field2dtype[field_name]

        next_entry_count = next_pc.shape[-1]
        next_bytes = next_pc.astype(dtypetemp.dtype).tobytes()

        next_bytes_width = dtypetemp.itemsize * next_entry_count
        next_bytes_np = np.frombuffer(next_bytes, dtype=np.uint8).reshape(-1, next_bytes_width)

        all_bytes_np = np.hstack((curr_bytes_np, next_bytes_np))

        #2 Populate fields
        pfname  = field2pfname[field_name]
        pftype  = field2pftype[field_name]
        pfpos   = curr_bytes_np.shape[-1]
        fields.append(PointField(pfname, pfpos, pftype, 1))
        
        return all_bytes_np, fields

    #1 Populate pc2 fields
    pc = pc.reshape(-1, pc.shape[-1]) # Reshape pc to N x pc_fields
    all_bytes_np = np.empty((pc.shape[0], 0), dtype=np.uint8)
    all_fields_list = []
    field_names = point_type.split(" ")
    for field_idx, field_name in enumerate(field_names):
        next_field_col_np = pc[:, field_idx].reshape(-1, 1)
        all_bytes_np, all_fields_list = add_field(
            all_bytes_np, next_field_col_np, field_name, all_fields_list
        )

    #2 Make pc2 object
    pc_msg = PointCloud2()
    pc_msg.width        = 1
    pc_msg.height       = pc.shape[0]

    pc_msg.header            = std_msgs.msg.Header()
    pc_msg.header.stamp      = ts
    pc_msg.header.frame_id   = frame_id
    pc_msg.header.seq        = seq

    pc_msg.point_step = all_bytes_np.shape[-1]
    pc_msg.row_step     = pc_msg.width * pc_msg.point_step
    pc_msg.fields       = all_fields_list
    pc_msg.data         = all_bytes_np.tobytes()
    pc_msg.is_dense     = True

    if publish:
        pc_pub.publish(pc_msg)

    return pc_msg

def pub_dt_to_rviz(dets, publisher, ts, frame_id, namespace='os_sensor', colormap=None, publish=True):
    """
    Publish detections to rviz

    Args:
        dets: detections, Nx8 array
        publisher: ROS publisher
        ts: time stamp
        frame_id: frame id
        namespace: namespace of the marker
        publish: whether to publish the marker

    Returns:
        marker_array: a MarkerArray message
    """
    if not isinstance(ts, rospy.Time):
        ts = rospy.Time.from_sec(ts)

    #1 Clear old markerarray
    clear_marker_array(publisher)

    marker_array = MarkerArray()
    for i, det in enumerate(dets):
        cx, cy, cz, l, w, h, vx, vy, yaw, id, score = det
        roll, pitch = 0, 0 # Assume roll and pitch are 0

        if colormap is not None:
            r, g, b = colormap[int(id)]
        else:
            r, g, b = 1.0, 0.0, 0.0
        marker = create_3d_bbox_marker(
            cx, cy, cz, l, w, h, roll, pitch, -yaw, frame_id, ts, namespace, i,
            r=r/255.0, g=g/255.0, b=b/255.0, a=1.0
        )
        marker_array.markers.append(marker)
        

    if publish:
        publisher.publish(marker_array)
    return marker_array


def clear_marker_array(publisher):
    """
    Clear previous bbox markers
    """
    bbox_markers = MarkerArray()
    bbox_marker = Marker()
    bbox_marker.id = 0
    bbox_marker.ns = "delete_markerarray"
    bbox_marker.action = Marker.DELETEALL

    bbox_markers.markers.append(bbox_marker)
    publisher.publish(bbox_markers)

def create_3d_bbox_marker(cx, cy, cz, l, w, h, roll, pitch, yaw,
                          frame_id, time_stamp, namespace='', marker_id=0,
                          r=1.0, g=0.0, b=0.0, a=1.0, scale=0.1):
    """
    Create a 3D bounding box marker

    Args:
        cx, cy, cz: center of the bounding box
        l, w, h: length, width, and height of the bounding box
        roll, pitch, yaw: orientation of the bounding box
        frame_id: frame id of header msgs
        time_stamp: time stamp of header msgs
        namespace: namespace of the bounding box
        marker_id: id of the bounding box
        scale: scale of line width
        r, g, b, a: color and transparency of the bounding box
    
    Returns:
        marker: a 3D bounding box marker
    """

    # Create transformation matrix from the given roll, pitch, yaw
    rot_mat = R.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_matrix()

    # Half-length, half-width, and half-height
    hl, hw, hh = l / 2.0, w / 2.0, h / 2.0

    # Define 8 corners of the bounding box in the local frame
    local_corners = np.array([
        [hl, hw, hh],  [hl, hw, -hh],  [hl, -hw, hh],  [hl, -hw, -hh],
        [-hl, hw, hh], [-hl, hw, -hh], [-hl, -hw, hh], [-hl, -hw, -hh]
    ]).T

    # Transform corners to the frame
    frame_corners = rot_mat.dot(local_corners)
    frame_corners += np.array([[cx], [cy], [cz]])
    
    # Define lines for the bounding box (each line by two corner indices)
    lines = [
        [0, 1], [0, 2], [1, 3], [2, 3],
        [4, 5], [4, 6], [5, 7], [6, 7],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ]

    # Create the LineList marker
    marker = Marker()
    marker.header.frame_id = frame_id
    if type(time_stamp) is not rospy.Time:
        time_stamp = rospy.Time.from_sec(time_stamp)
    marker.header.stamp = time_stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = scale
    marker.color.a = a
    marker.color.r = r 
    marker.color.g = g
    marker.color.b = b 
    marker.pose.orientation.w = 1.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.lifetime = rospy.Duration(0)

    for start, end in lines:
        start_pt = Point(*frame_corners[:, start])
        end_pt = Point(*frame_corners[:, end])
        marker.points.extend([start_pt, end_pt])

    return marker