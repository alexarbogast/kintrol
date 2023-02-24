import rospy
import actionlib
from geometry_msgs.msg import PoseArray, Pose
from kintrol.msg import TrajectoryExecutionAction, TrajectoryExecutionGoal 

V_MAX = 200.0 / 1000.0;  # m/s
A_MAX = 700.0 / 1000.0; # m/s^2
J_MAX = 3000.0 / 1000.0; # m/s^3

def trajectory_execution_client():
    client = actionlib.SimpleActionClient('/robot1/trajectory_execution_action', TrajectoryExecutionAction)
    client.wait_for_server()

    test_path = PoseArray()
        
    start_pose = Pose()
    start_pose.position.x = 0.6281204359253633
    start_pose.position.y = -1.150820145534631e-07
    start_pose.position.z = 0.507627954931471
    test_path.poses.append(start_pose)

    pose0 = Pose()
    pose0.position.x = 0.6
    pose0.position.y = 0.0
    pose0.position.z = 0.1
    test_path.poses.append(pose0)

    pose1 = Pose()
    pose1.position.x = 0.8
    pose1.position.y = 0.3
    pose1.position.z = 0.1
    test_path.poses.append(pose1)

    pose2 = Pose()
    pose2.position.x = 0.4
    pose2.position.y = 0.3
    pose2.position.z = 0.1
    test_path.poses.append(pose2)

    pose3 = Pose()
    pose3.position.x = 0.4
    pose3.position.y = -0.3
    pose3.position.z = 0.1
    test_path.poses.append(pose3)

    pose3 = Pose()
    pose3.position.x = 0.8
    pose3.position.y = -0.3
    pose3.position.z = 0.1
    test_path.poses.append(pose3)

    test_path.poses.append(start_pose)

    goal = TrajectoryExecutionGoal()
    goal.path = test_path
    goal.limits.v_max = V_MAX
    goal.limits.a_max = A_MAX
    goal.limits.j_max = J_MAX

    client.send_goal(goal)
    client.wait_for_result()
    

if __name__ == "__main__":
    try:
        rospy.init_node('trajectory_execution_test')

        print("Starting trajectory...")
        success = trajectory_execution_client()
        print("Finished trajectory")
    except rospy.ROSInterruptException:
        pass