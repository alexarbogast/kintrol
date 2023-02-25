import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from kintrol.msg import TrajectoryExecutionAction, TrajectoryExecutionGoal 

V_MAX = 400.0 / 1000.0;  # m/s
A_MAX = 700.0 / 1000.0; # m/s^2
J_MAX = 3000.0 / 1000.0; # m/s^3

BATCH = False

def trajectory_execution_client():
    client1 = actionlib.SimpleActionClient('/robot1/trajectory_execution_action', TrajectoryExecutionAction)
    client1.wait_for_server()
    client2 = actionlib.SimpleActionClient('/robot2/trajectory_execution_action', TrajectoryExecutionAction)
    client2.wait_for_server()
    client3 = actionlib.SimpleActionClient('/robot3/trajectory_execution_action', TrajectoryExecutionAction)
    client3.wait_for_server()

    # flange
    #start = np.array([0.6281204359253633, -1.150820145534631e-07, 0.507627954931471])
    #orient = np.array([0, 0, 0, 1])
    
    # typhoon extruder
    start = np.array([0.7142504784757611, 1.0829474352021246e-07, 0.34862800753764744])
    orient = np.array([0, 1, 0, 0])

    test_path = PoseArray()

    start_pose = Pose()
    set_pose(start, orient, start_pose)
    test_path.poses.append(start_pose)

    width = 0.3
    path = make_cube(start, width)
    for segment in path:
        pose = Pose()
        set_pose(segment, orient, pose)
        test_path.poses.append(pose)

    test_path.poses.append(start_pose)

    goal = TrajectoryExecutionGoal()
    goal.path = test_path
    goal.limits.v_max = V_MAX
    goal.limits.a_max = A_MAX
    goal.limits.j_max = J_MAX
    goal.batch = BATCH

    client1.send_goal(goal)
    client2.send_goal(goal)
    client3.send_goal(goal)
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
    
def make_cube(center, width):
    w2 = width / 2
    xmin = center[0] - w2
    xmax = center[0] + w2
    ymin = center[1] - w2
    ymax = center[1] + w2
    zmin = center[2] - w2
    zmax = center[2] + w2
    
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

    path = [fru, flu, blu, bru, fru, frd, fru, flu,
            fld, bld, blu, flu, fld, frd, brd, bru,
            blu, bld, brd, bru]

    return path


if __name__ == "__main__":
    try:
        rospy.init_node('trajectory_execution_test')

        print("Starting trajectory...")
        success = trajectory_execution_client()
        print("Finished trajectory")
    except rospy.ROSInterruptException:
        pass