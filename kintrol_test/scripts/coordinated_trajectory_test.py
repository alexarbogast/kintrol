import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from kintrol.msg import TrajectoryExecutionAction, TrajectoryExecutionGoal

V_MAX = 400.0 / 1000.0;  # m/s
A_MAX = 800.0 / 1000.0;  # m/s^2
J_MAX = 1000.0 / 1000.0; # m/s^3

BATCH = False

def trajectory_execution_client():
    client1 = actionlib.SimpleActionClient('/robot1/trajectory_execution_action', TrajectoryExecutionAction)
    client1.wait_for_server()
    client2 = actionlib.SimpleActionClient('/robot2/trajectory_execution_action', TrajectoryExecutionAction)
    client2.wait_for_server()
    client3 = actionlib.SimpleActionClient('/robot3/trajectory_execution_action', TrajectoryExecutionAction)
    client3.wait_for_server()
    
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

    path = make_cube(np.array([0.25*np.cos(-2*np.pi/3), 0.25*np.sin(-2*np.pi/3), 0.3]), width, np.pi/3)
    for segment in path:
        pose = Pose()
        set_pose(segment, orient, pose)
        test_path1.poses.append(pose)

    test_path1.poses.append(start_pose1)

    goal1 = TrajectoryExecutionGoal()
    goal1.path = test_path1
    goal1.limits.v_max = V_MAX
    goal1.limits.a_max = A_MAX
    goal1.limits.j_max = J_MAX
    goal1.batch = BATCH

    # ========= robot 2 path ==========
    start_pose2 = Pose()
    set_pose(start2, orient, start_pose2)
    test_path2.poses.append(start_pose2)

    path = make_cube(np.array([0.25, 0.0, 0.3]), width)
    for segment in path:
        pose = Pose()
        set_pose(segment, orient, pose)
        test_path2.poses.append(pose)

    test_path2.poses.append(start_pose2)

    goal2 = TrajectoryExecutionGoal()
    goal2.path = test_path2
    goal2.limits.v_max = V_MAX
    goal2.limits.a_max = A_MAX
    goal2.limits.j_max = J_MAX
    goal2.batch = BATCH

    # ========= robot 3 path ==========
    start_pose3 = Pose()
    set_pose(start3, orient, start_pose3)
    test_path3.poses.append(start_pose3)

    path = make_cube(np.array([0.25*np.cos(2*np.pi/3), 0.25*np.sin(2*np.pi/3), 0.3]), width, -np.pi/3)
    for segment in path:
        pose = Pose()
        set_pose(segment, orient, pose)
        test_path3.poses.append(pose)

    test_path3.poses.append(start_pose3)

    goal3 = TrajectoryExecutionGoal()
    goal3.path = test_path3
    goal3.limits.v_max = V_MAX
    goal3.limits.a_max = A_MAX
    goal3.limits.j_max = J_MAX
    goal3.batch = BATCH

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


if __name__ == "__main__":
    try:
        rospy.init_node('coordinated_trajectory_test')

        print("Starting trajectory...")
        success = trajectory_execution_client()
        print("Finished trajectory")
    except rospy.ROSInterruptException:
        pass