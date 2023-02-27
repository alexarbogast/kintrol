import rospy
import actionlib
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from kintrol.msg import TrajectoryExecutionAction, TrajectoryExecutionGoal

from kintrol_test.scripts.trajectory_test import set_pose, all_edges, make_rectangle

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
    #path = all_edges(start, width)
    path = manipulability_example()
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

def manipulability_example():
    x = 0.9
    y = 0.9
    z = 0.5

    floor_offset = 0.4
    center = np.array([0, 0, z / 2 + floor_offset])

    flu, fru, frd, fld, bld, blu, bru, brd = make_rectangle(center, x, y, z)
    path = [blu, flu, fld, bld, blu, bru, brd, frd, fru, bru, brd, bld]

    return path


if __name__ == "__main__":
    try:
        rospy.init_node('trajectory_execution_test')

        print("Starting trajectory...")
        success = trajectory_execution_client()
        print("Finished trajectory")
    except rospy.ROSInterruptException:
        pass