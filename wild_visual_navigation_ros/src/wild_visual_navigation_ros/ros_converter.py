from anymal_msgs.msg import AnymalState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

from liegroups.torch import SO3, SE3
import numpy as np
import torch
import torchvision.transforms as transforms

CV_BRIDGE = CvBridge()
TO_TENSOR = transforms.ToTensor()
TO_PIL_IMAGE = transforms.ToPILImage()
BASE_DIM = 7 + 6  # pose + twist


def robot_state_to_torch(robot_state, device="cpu"):
    assert isinstance(robot_state, Odometry)

    # preallocate torch state
    torch_state = torch.zeros(BASE_DIM, dtype=torch.float32).to(device)
    state_labels = []

    # Base
    # Pose
    state_labels.extend(["tx", "ty", "tz", "qx", "qy", "qz", "qw"])
    torch_state[0] = robot_state.pose.pose.position.x
    torch_state[1] = robot_state.pose.pose.position.y
    torch_state[2] = robot_state.pose.pose.position.z
    torch_state[3] = robot_state.pose.pose.orientation.x
    torch_state[4] = robot_state.pose.pose.orientation.y
    torch_state[5] = robot_state.pose.pose.orientation.z
    torch_state[6] = robot_state.pose.pose.orientation.w

    # Twist
    state_labels.extend(["vx", "vy", "vz", "wx", "wy", "wz"])
    torch_state[7] = robot_state.twist.twist.linear.x
    torch_state[8] = robot_state.twist.twist.linear.y
    torch_state[9] = robot_state.twist.twist.linear.z
    torch_state[10] = robot_state.twist.twist.angular.x
    torch_state[11] = robot_state.twist.twist.angular.y
    torch_state[12] = robot_state.twist.twist.angular.z

    return torch_state, state_labels


def anymal_state_to_torch(anymal_state, device="cpu"):
    assert isinstance(anymal_state, AnymalState)
    LEG_DIM = 12
    ANYMAL_DIM = LEG_DIM * 4

    # preallocate torch state
    torch_state = torch.zeros(BASE_DIM + ANYMAL_DIM, dtype=torch.float32).to(device)
    state_labels = []

    # Base
    # Pose
    state_labels.extend(["tx", "ty", "tz", "qx", "qy", "qz", "qw"])
    torch_state[0] = anymal_state.pose.pose.position.x
    torch_state[1] = anymal_state.pose.pose.position.y
    torch_state[2] = anymal_state.pose.pose.position.z
    torch_state[3] = anymal_state.pose.pose.orientation.x
    torch_state[4] = anymal_state.pose.pose.orientation.y
    torch_state[5] = anymal_state.pose.pose.orientation.z
    torch_state[6] = anymal_state.pose.pose.orientation.w

    # Twist
    state_labels.extend(["vx", "vy", "vz", "wx", "wy", "wz"])
    torch_state[7] = anymal_state.twist.twist.linear.x
    torch_state[8] = anymal_state.twist.twist.linear.y
    torch_state[9] = anymal_state.twist.twist.linear.z
    torch_state[10] = anymal_state.twist.twist.angular.x
    torch_state[11] = anymal_state.twist.twist.angular.y
    torch_state[12] = anymal_state.twist.twist.angular.z

    # Joints
    # Position
    N = 13
    state_labels.extend(["position_{x}" for x in anymal_state.joints.name])
    torch_state[N : N + LEG_DIM] = torch.FloatTensor(anymal_state.joints.position)
    # Velocity
    N = N + LEG_DIM
    state_labels.extend(["velocity_{x}" for x in anymal_state.joints.name])
    torch_state[N : N + LEG_DIM] = torch.FloatTensor(anymal_state.joints.velocity)
    # Acceleration
    N = N + LEG_DIM
    state_labels.extend(["acceleration_{x}" for x in anymal_state.joints.name])
    torch_state[N : N + LEG_DIM] = torch.FloatTensor(anymal_state.joints.acceleration)
    # Effort
    N = N + LEG_DIM
    state_labels.extend(["effort_{x}" for x in anymal_state.joints.name])
    torch_state[N : N + LEG_DIM] = torch.FloatTensor(anymal_state.joints.effort)

    return torch_state, state_labels


def ros_cam_info_to_tensors(caminfo_msg, device="cpu"):
    K = torch.eye(4, dtype=torch.float32).to(device)
    K[:3, :3] = torch.FloatTensor(caminfo_msg.K).reshape(3, 3)
    K = K.unsqueeze(0)
    H = torch.IntTensor([caminfo_msg.height]).to(device)
    W = torch.IntTensor([caminfo_msg.width]).to(device)
    return K, H, W


def ros_pose_to_torch(ros_pose, device="cpu"):
    q = torch.FloatTensor(
        [ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z, ros_pose.orientation.w]
    )
    t = torch.FloatTensor([ros_pose.position.x, ros_pose.position.y, ros_pose.position.z])
    return SE3(SO3.from_quaternion(q, ordering="xyzw"), t).as_matrix().to(device)


def ros_tf_to_torch(tf_pose, device="cpu"):
    assert len(tf_pose) == 2
    assert isinstance(tf_pose, tuple)

    t = torch.FloatTensor(tf_pose[0])
    q = torch.FloatTensor(tf_pose[1])
    return SE3(SO3.from_quaternion(q, ordering="xyzw"), t).as_matrix().to(device)


def ros_image_to_torch(ros_img, desired_encoding="rgb8", device="cpu"):
    np_image = CV_BRIDGE.imgmsg_to_cv2(ros_img, desired_encoding=desired_encoding)
    return TO_TENSOR(np_image).to(device)


def torch_to_ros_image(torch_img, desired_encoding="rgb8"):
    np_image = np.array(TO_PIL_IMAGE(torch_img.cpu()))
    ros_image = CV_BRIDGE.cv2_to_imgmsg(np_image, encoding=desired_encoding)
    return ros_image


def torch_to_ros_pose(torch_pose):
    q = SO3.from_matrix(torch_pose[:3, :3].cpu()).to_quaternion(ordering="xyzw")
    t = torch_pose[:3, 3].cpu()
    pose = Pose()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    pose.position.x = t[0]
    pose.position.y = t[1]
    pose.position.z = t[2]

    return pose
