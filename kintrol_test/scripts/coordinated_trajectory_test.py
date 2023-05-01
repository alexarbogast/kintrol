import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from kintrol.msg import TrajectoryExecutionAction, TrajectoryExecutionGoal

from hydra_controllers.srv import SwitchCoordination

V_MAX = 100.0 / 1000.0;  # m/s
A_MAX = 400.0 / 1000.0;  # m/s^2
J_MAX = 1000.0 / 1000.0; # m/s^3

BATCH = False

def trajectory_execution_client():
    client1 = actionlib.SimpleActionClient('/robot1/trajectory_execution_action', 
                                           TrajectoryExecutionAction)
    client1.wait_for_server()
    client2 = actionlib.SimpleActionClient('/robot2/trajectory_execution_action', 
                                           TrajectoryExecutionAction)
    client2.wait_for_server()
    client3 = actionlib.SimpleActionClient('/robot3/trajectory_execution_action', 
                                           TrajectoryExecutionAction)
    client3.wait_for_server()

    rospy.wait_for_service('/hydra_controller/switch_coordination')
    switch_coordination_client = rospy.ServiceProxy('/hydra_controller/switch_coordination',
                                                     SwitchCoordination)
    resp1 = switch_coordination_client(arm_id="rob1", coordinated=True)
    resp2 = switch_coordination_client(arm_id="rob2", coordinated=True)
    resp3 = switch_coordination_client(arm_id="rob3", coordinated=True)
    if not (resp1.success and resp2.success and resp3.success):
        print("Failed to switch to coordinated controller")
        return False
    
    # TODO: wait for state message to find initial position
    # TODO: find start positions
    # typhoon extruder
    start1 = np.array([-0.117879, -0.204162,  0.389365])
    start2 = np.array([0.235749, 6.74803e-06, 0.389365])
    start3 = np.array([-0.117883, 0.20416, 0.389365])
    orient = np.array([0, 1, 0, 0])

    test_path1 = PoseArray()
    test_path2 = PoseArray()
    test_path3 = PoseArray()

    # ========= robot 1 path ==========
    start_pose1 = Pose()
    set_pose(start1, orient, start_pose1)
    test_path1.poses.append(start_pose1)
    width = 0.2
    z = 0.00238

    #path = make_cube(np.array([0.25*np.cos(-2*np.pi/3), 0.25*np.sin(-2*np.pi/3), 0.3]), width, np.pi/3)
    #path = [np.array([-0.3, -0.3, 0.02]), np.array([-0.3, 0.3, 0.02]), np.array([0.3, 0.3, 0.02]), np.array([0.3, -0.3, 0.02]), np.array([-0.3, -0.3, 0.02])]
    path = [np.array([-0.3236, -0.2351, z]), np.array([-0.1528, 0.0, z]), np.array([-0.3236, 0.2351, z]), np.array([-0.0472, 0.1453, z]), np.array([0.1236, 0.380, z]), np.array([0.1236, 0.09, z]), np.array([0.4, 0.0, z]), np.array([0.1236, -0.09, z]),  np.array([0.1236, -0.380, z]), np.array([-0.0472, -0.1453, z]), np.array([-0.3236, -0.2351, z])]
    for segment in path:
        pose = Pose()
        set_pose(segment, orient, pose)
        test_path1.poses.append(pose)

    test_path1.poses.append(start_pose1)

    goal1 = make_goal()
    goal1.path = test_path1

    # ========= robot 2 path ==========
    start_pose2 = Pose()
    set_pose(start2, orient, start_pose2)
    test_path2.poses.append(start_pose2)

    #path = make_cube(np.array([0.25, 0.0, 0.3]), width)
    #path = [np.array([0.275, -0.275, 0.02]), np.array([-0.275, -0.275, 0.02]), np.array([-0.275, 0.275, 0.02]), np.array([0.275, 0.275, 0.02]), np.array([0.275, -0.275, 0.02])]
    path = [np.array([0.4, 0.0, z]), np.array([0.1236, -0.09, z]),  np.array([0.1236, -0.380, z]), np.array([-0.0472, -0.1453, z]), np.array([-0.3236, -0.2351, z]), np.array([-0.1528, 0.0, z]), np.array([-0.3236, 0.2351, z]), np.array([-0.0472, 0.1453, z]), np.array([0.1236, 0.380, z]), np.array([0.1236, 0.09, z]), np.array([0.4, 0.0, z])]
    for segment in path:
        pose = Pose()
        set_pose(segment, orient, pose)
        test_path2.poses.append(pose)

    test_path2.poses.append(start_pose2)

    goal2 = make_goal()
    goal2.path = test_path2

    # ========= robot 3 path ==========
    start_pose3 = Pose()
    set_pose(start3, orient, start_pose3)
    test_path3.poses.append(start_pose3)

    #path = make_cube(np.array([0.25*np.cos(2*np.pi/3), 0.25*np.sin(2*np.pi/3), 0.3]), width, -np.pi/3)
    #path = [np.array([-0.250, 0.250, 0.02]), np.array([0.250, 0.250, 0.02]), np.array([0.250, -0.250, 0.02]), np.array([-0.250, -0.250, 0.02]), np.array([-0.250, 0.250, 0.02])]
    path = [np.array([-0.3236, 0.2351, z]), np.array([-0.0472, 0.1453, z]), np.array([0.1236, 0.380, z]), np.array([0.1236, 0.09, z]), np.array([0.4, 0.0, z]), np.array([0.1236, -0.09, z]),  np.array([0.1236, -0.380, z]), np.array([-0.0472, -0.1453, z]), np.array([-0.3236, -0.2351, z]), np.array([-0.1528, 0.0, z]), np.array([-0.3236, 0.2351, z])]
    for segment in path:
        pose = Pose()
        set_pose(segment, orient, pose)
        test_path3.poses.append(pose)

    test_path3.poses.append(start_pose3)

    goal3 = make_goal()
    goal3.path = test_path3

    client1.send_goal(goal1)
    client2.send_goal(goal2)
    client3.send_goal(goal3)
    client1.wait_for_result()
    client2.wait_for_result()
    client3.wait_for_result()

def set_pose(position, orientation, pose: PoseArray):
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.w = orientation[0]
    pose.orientation.x = orientation[1]
    pose.orientation.y = orientation[2]
    pose.orientation.z = orientation[3]

def make_rectangle(center, x, y, z, z_rot=0.0):
    xmin = -x / 2
    xmax =  x / 2
    ymin = -y / 2
    ymax =  y / 2
    zmin = -z / 2
    zmax =  z / 2

    # x: (f)ront, (b)ack
    # y: (l)eft, (r)ight
    # z: (t)op, (d)own
    flu = np.array([xmin, ymax, zmax])
    fru = np.array([xmin, ymin, zmax])
    frd = np.array([xmin, ymin, zmin])
    fld = np.array([xmin, ymax, zmin])
    bld = np.array([xmax, ymax, zmin])
    blu = np.array([xmax, ymax, zmax])
    bru = np.array([xmax, ymin, zmax])
    brd = np.array([xmax, ymin, zmin])

    points = [flu, fru, frd, fld, bld, blu, bru, brd]
    Rz = np.array([[np.cos(z_rot), -np.sin(z_rot), 0],
                   [np.sin(z_rot), np.cos(z_rot),  0],
                   [0,             0,              1]])
    for i in range(len(points)):
        points[i] = Rz @ points[i]
        points[i] = points[i] + center
    
    for point in points:
        print(point)

    return points

def make_cube(center, width, z_rot=0.0):
    return make_rectangle(center, width, width, width, z_rot)

def make_goal():
    goal = TrajectoryExecutionGoal()
    goal.limits.v_max = V_MAX
    goal.limits.a_max = A_MAX
    goal.limits.j_max = J_MAX
    goal.batch = BATCH
    return goal


if __name__ == "__main__":
    try:
        rospy.init_node('coordinated_trajectory_test')

        print("Starting trajectory...")
        success = trajectory_execution_client()
        print("Finished trajectory")
    except rospy.ROSInterruptException:
        pass