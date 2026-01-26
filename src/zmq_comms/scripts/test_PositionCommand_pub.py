import rospy
from zmq_comms.msg import PositionCommand

rospy.init_node('position_command_publisher', anonymous=True)

rospy.loginfo("PositionCommand publisher node started, waiting for messages...")
pub = rospy.Publisher('/planning/cmd_linear_vel_2', PositionCommand, queue_size=10)
rate = rospy.Rate(1)  # 1 Hz
while not rospy.is_shutdown():
    msg = PositionCommand()
    msg.position.x = 1.0
    msg.position.y = 2.0
    msg.position.z = 3.0
    msg.yaw = 0.5
    pub.publish(msg)
    rospy.loginfo(f"Published PositionCommand: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}, yaw={msg.yaw}")
    rate.sleep()
