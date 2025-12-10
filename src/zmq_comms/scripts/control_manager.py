#!/usr/bin/env python3

# Script to takes in NUS velocity and yaw setpoints and outputs them to mavros commands
# Also keeps track of drone state and PX4 state

import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from std_msgs.msg import Header
import math

class AGENT_STATES:
    INIT = 1
    TAKING_OFF = 2
    HOVERING = 3
    LANDED = 4
    RUNNING = 5
    LANDING_TRIGGERED = 6
    LANDING = 7

TAKEOFF_HEIGHT = 1.1  # meters

class AgentStateManager:
    def velocity_callback(self,data):
        self.vel_yaw_msg.velocity.x = data.twist.linear.x
        self.vel_yaw_msg.velocity.y = data.twist.linear.y
        self.vel_yaw_msg.velocity.z = data.twist.linear.z
        
        vx = self.vel_yaw_msg.velocity.x
        vy = self.vel_yaw_msg.velocity.y
        vz = self.vel_yaw_msg.velocity.z

        self.last_valid_cmd = data.header.stamp

        # Calculate magnitude
        magnitude = math.sqrt(vx**2 + vy**2 + vz**2)
        if (magnitude < 0.10):
            self.agent_state == AGENT_STATES.HOVERING
            print("HOVERING TRIGGERED !!! ")
        else:
            self.agent_state == AGENT_STATES.RUNNING

    def yaw_callback(self, data):
        # Account for starling2 frame
        # setpoint.yaw = data.x + 1.57
        self.vel_yaw_msg.yaw = data.twist.linear.x

        self.last_valid_cmd = data.header.stamp

    def pose_callback(self, data):
        self.agent_pose = data

    def px4_state_cb(self, data):
        self.px4_current_state = data
        rospy.loginfo("PX4_STATE: {}, ARMED_STATE: {}, AGENT_STATE: {}".format(self.px4_current_state.mode, self.px4_current_state.armed, self.agent_state))
        rospy.loginfo("Linear, X: {}, Y: {}, Z: {} -- Yaw: {}".format(self.vel_yaw_msg.velocity.x, self.vel_yaw_msg.velocity.y, self.vel_yaw_msg.velocity.z, self.vel_yaw_msg.yaw))

    def agent_state_callback(self, data):
        self.agent_state = int(data.data) 
        rospy.loginfo("New agent state input!!: {}".format(self.agent_state))
    
    def check_takeoff_finished(self):
        return self.agent_pose.pose.position.z >= 0.90 * TAKEOFF_HEIGHT

    def __init__(self):
        self.vel_yaw_msg = PositionTarget()
        self.position_msg = PositionTarget()
        self.agent_pose = PoseStamped()
        self.yaw = 0.0
        self.agent_state = AGENT_STATES.INIT # CURRENT STATE
        self.px4_current_state = State()

        # Flags
        self.request_available_flag = False 
        self.valid_cmd_flag = False

        rospy.loginfo("Initializing control manager node...")
        rospy.init_node("agent_state_manager", anonymous=True)

        # Publishers
        self.setpoint_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        # Subscribers
        self.velocity_sub = rospy.Subscriber("/planning/cmd_linear_vel_2", TwistStamped, self.velocity_callback)
        self.yaw_sub = rospy.Subscriber("/planning/cmd_yaw_2", TwistStamped, self.yaw_callback)
        self.agent_state_sub = rospy.Subscriber("/agent/state", String, self.agent_state_callback)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)

        # Mavros stuffs
        self.state_sub = rospy.Subscriber("mavros/state", State, self.px4_state_cb)

        rospy.loginfo("Waiting for mavros services")
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # Initialize velocity yaw setpoint msgs
        self.vel_yaw_msg.header = Header()
        self.vel_yaw_msg.header.frame_id ='drone_body'
        self.vel_yaw_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Type mask: Ignore everything except Velocity and YAW
        self.vel_yaw_msg.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            # PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        self.vel_yaw_msg.velocity.x = 0.0
        self.vel_yaw_msg.velocity.y = 0.0
        self.vel_yaw_msg.velocity.z = 0.0 
        self.vel_yaw_msg.yaw = 0

        # Initialize position setpoints msgs
        self.position_msg.header = Header()
        self.position_msg.header.frame_id ='drone_body'
        self.position_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Type mask: Ignore everything except position and YAW
        self.position_msg.type_mask = (
            # PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE | PositionTarget.IGNORE_YAW
        )
        self.position_msg.position.x = 0.0
        self.position_msg.position.y = 0.0
        self.position_msg.position.z = 1.0  # Take off height

        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.land_set_mode = SetModeRequest()
        self.land_set_mode.custom_mode = 'AUTO.LAND'

        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        self.last_req = rospy.Time.now()
        self.land_timer = rospy.Time.now()
        self.last_valid_cmd = rospy.Time.now()

        rate = rospy.Rate(100)

        rospy.loginfo("Sending dummy setpoints before arming")
        for i in range(100):
            if(rospy.is_shutdown()):
                break
            self.setpoint_pub.publish(self.position_msg)
            rate.sleep()
        rospy.loginfo("Finished sending dummy setpoings")

        rospy.loginfo("Initiaizing Finished")

        # Enter Main Control Loop
        while not rospy.is_shutdown():
            self.request_available_flag = (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)
            self.valid_cmd_flag = rospy.Time.now() - self.last_valid_cmd < rospy.Duration(1.0)

            if (self.agent_state == AGENT_STATES.TAKING_OFF):
                if(self.px4_current_state.mode != "OFFBOARD" and self.request_available_flag):
                    if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                        rospy.loginfo("OFFBOARD enabled")
                        self.last_req = rospy.Time.now()
                    else:
                        rospy.logwarn("OFFBOARD enable failed!!!")

                elif(self.px4_current_state.mode == "OFFBOARD" and not self.px4_current_state.armed and self.request_available_flag):
                    if(self.arming_client.call(self.arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")
                        self.last_req = rospy.Time.now()
                    else:
                        rospy.logwarn("Vehicle arming failed!!!")

                self.position_msg.position.x = self.agent_pose.pose.position.x
                self.position_msg.position.y = self.agent_pose.pose.position.y
                self.position_msg.position.z = TAKEOFF_HEIGHT
                self.setpoint_pub.publish(self.position_msg)
                if self.check_takeoff_finished():
                    rospy.loginfo("Takeoff finished detected!")
                    self.agent_state = AGENT_STATES.HOVERING

            elif (self.agent_state == AGENT_STATES.HOVERING):
                self.setpoint_pub.publish(self.position_msg)
                # check for valid commands
                if  (self.valid_cmd_flag):
                    rospy.loginfo("Recieved valid velocity commands, switching to RUNNING state")
                    self.agent_state = AGENT_STATES.RUNNING
                else:
                    rospy.loginfo("Waiting for VALID or RECENT ENOUGH velocity commands to switch to RUNNING state")

            elif (self.agent_state == AGENT_STATES.RUNNING):
                self.position_msg.position.x = self.agent_pose.pose.position.x
                self.position_msg.position.y = self.agent_pose.pose.position.y
                self.position_msg.position.z = self.agent_pose.pose.position.z
                if (self.valid_cmd_flag):
                    self.setpoint_pub.publish(self.vel_yaw_msg)
                else:
                    self.agent_state = AGENT_STATES.HOVERING 
                    print("Going back to Hovering state- No valid velocity commands received recently")
            elif (self.agent_state == AGENT_STATES.LANDING_TRIGGERED):
                if(self.set_mode_client.call(self.land_set_mode).mode_sent == True):
                    rospy.loginfo("LANDING_TRIGGERED mode called")
                    self.agent_state = AGENT_STATES.LANDING
                else:
                    rospy.loginfo("THIS SHOULD NOT HAPPEN- FAILED TO CALL PX4 AUTO.LAND!!!")
            elif (self.agent_state == AGENT_STATES.LANDING):
                # Do nothing- PX4 handles the landing
                pass
            rate.sleep()

if __name__ == "__main__":
    try:
        AgentStateManager()
    except rospy.ROSInterruptException:
        print("ROS Interrupt Exception caught- This should not happen!!! ")
        pass