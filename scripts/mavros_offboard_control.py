#! /usr/bin/env python

import math
import numpy as np

import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3

from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, ParamValue, State, \
                            WaypointList, AttitudeTarget
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode, SetModeRequest, WaypointClear, \
                            WaypointPush

from tf.transformations import quaternion_from_euler
from pymavlink import mavutil

from threading import Thread
from six.moves import xrange



class OffBoardControl:

    def __init__(self):

        # tracking variables
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()
        self.mav_type = None
        self.att = None
        self.position = None

        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
                'mission_wp', 'state', 'imu'
            ]
        }

        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/param/set', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            raise rospy.ROSException("failed to connect to services")

        self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.set_param_srv = rospy.ServiceProxy('mavros/param/set', ParamSet)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push', WaypointPush)

        # ROS subscribers
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude,
                                        self.altitude_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state', ExtendedState,
                                              self.extended_state_callback)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix,
                                               self.global_position_callback)
        self.imu_data_sub = rospy.Subscriber('mavros/imu/data', Imu,
                                               self.imu_data_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home', HomePosition,
                                             self.home_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped,
                                              self.local_position_callback)
        self.mission_wp_sub = rospy.Subscriber('mavros/mission/waypoints', WaypointList,
                                               self.mission_wp_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_callback)

        # ROS publishers
        self.att_setpoint_pub = rospy.Publisher('mavros/setpoint_raw/attitude', 
                                                AttitudeTarget, queue_size=1)
        self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', 
                                                PoseStamped, queue_size=1)

        # tracking setpoints
        self.att_des = AttitudeTarget()
        self.position_des = PoseStamped()
        self.position_des.pose.position.x = 0
        self.position_des.pose.position.y = 0
        self.position_des.pose.position.z = 2 
        self.radius = 1


    # ========= Callback functions ===============
    def altitude_callback(self, data:Altitude):
        self.altitude = data

        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def extended_state_callback(self, data:ExtendedState):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))

        self.extended_state = data

        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True

    def global_position_callback(self, data:NavSatFix):
        self.global_position = data

        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True

    def imu_data_callback(self, data:Imu):
        self.imu_data = data

        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True

    def home_position_callback(self, data:HomePosition):
        self.home_position = data

        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True

    def local_position_callback(self, data:PoseStamped):
        self.local_position = data
        self.att = data.pose.orientation # current attitude 
        self.position = data.pose.position # current position
        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def mission_wp_callback(self, data:WaypointList):
        if self.mission_wp.current_seq != data.current_seq:
            rospy.loginfo("current mission waypoint sequence updated: {0}".
                          format(data.current_seq))

        self.mission_wp = data

        if not self.sub_topics_ready['mission_wp']:
            self.sub_topics_ready['mission_wp'] = True

    def state_callback(self, data:State):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True


    # ========= Helper methods ===============    
    def set_mode(self, mode:str, timeout:int):
        """Set PX4 mode

        Args:
            mode (str): desired PX4 mode string
            timeout (int): timeout in seconds
            
        Raises:
            rospy.ROSException: ros service exception
        """        
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                raise rospy.ROSException(e) 

        assert mode_set, (
            "failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
            format(mode, old_mode, timeout))

    def set_arm(self, arm: bool, timeout:int):
        """Set arm state of PX4. arm=True to arm or False to disarm

        Args:
            arm (bool): arm state. True to arm or False to disarm
            timeout (int): timeout in seconds

        Raises:
            rospy.ROSException: ros service exception
        """        
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                raise rospy.ROSException(e)

        assert arm_set, (
            "failed to set arm | new arm: {0}, old arm: {1} | timeout(seconds): {2}".
            format(arm, old_arm, timeout))

    def set_param(self, param_id, param_value, timeout):
        """param: PX4 param string, ParamValue, timeout(int): seconds"""
        if param_value.integer != 0:
            value = param_value.integer
        else:
            value = param_value.real
        rospy.loginfo("setting PX4 parameter: {0} with value {1}".
        format(param_id, value))
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        param_set = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.set_param_srv(param_id, param_value)
                if res.success:
                    rospy.loginfo("param {0} set to {1} | seconds: {2} of {3}".
                    format(param_id, value, i / loop_freq, timeout))
                break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                raise rospy.ROSException(e)

        assert res.success, (
            "failed to set param | param_id: {0}, param_value: {1} | timeout(seconds): {2}".
            format(param_id, value, timeout))

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                raise rospy.ROSException(e)

        assert simulation_ready, (
            "failed to hear from all subscribed simulation topics | topic ready flags: {0} | timeout(seconds): {1}".
            format(self.sub_topics_ready, timeout))

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                raise rospy.ROSException(e)

        assert landed_state_confirmed, (
            "landed state not detected | desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".
            format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                desired_landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][self.extended_state.landed_state].name,
                   index, timeout))

    def log_topic_vars(self):
        """log the state of topic variables"""
        rospy.loginfo("========================")
        rospy.loginfo("===== topic values =====")
        rospy.loginfo("========================")
        rospy.loginfo("altitude:\n{}".format(self.altitude))
        rospy.loginfo("========================")
        rospy.loginfo("extended_state:\n{}".format(self.extended_state))
        rospy.loginfo("========================")
        rospy.loginfo("global_position:\n{}".format(self.global_position))
        rospy.loginfo("========================")
        rospy.loginfo("home_position:\n{}".format(self.home_position))
        rospy.loginfo("========================")
        rospy.loginfo("local_position:\n{}".format(self.local_position))
        rospy.loginfo("========================")
        rospy.loginfo("mission_wp:\n{}".format(self.mission_wp))
        rospy.loginfo("========================")
        rospy.loginfo("state:\n{}".format(self.state))
        rospy.loginfo("========================")


    # ========= core  functions ===============      
    def send_att(self):
        rate = rospy.Rate(30)  # Hz
        self.att_des.body_rate = Vector3()
        self.att_des.header = Header()
        self.att_des.header.frame_id = "base_footprint"
        self.att_des.orientation = Quaternion(*quaternion_from_euler(-0.25, 0.15,
                                                                 0))
        self.att_des.thrust = 0.7
        self.att_des.type_mask = 7  # ignore body rate

        while not rospy.is_shutdown():
            self.att_des.header.stamp = rospy.Time.now()
            self.att_setpoint_pub.publish(self.att_des)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def send_pos(self):
        rate = rospy.Rate(30)  # Hz
        self.position_des.header = Header()
        self.position_des.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.position_des.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.position_des)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
                # rospy.loginfo_throttle(2,self.position_des)
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.position_des.pose.position.x = x
        self.position_des.pose.position.y = y
        self.position_des.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.position_des.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.position_des.pose.position.x,
                                   self.position_des.pose.position.y,
                                   self.position_des.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                raise rospy.ROSException(e)

        assert reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout))

    def start_attctl_thread(self):
        # send setpoints in separate thread to better prevent failsafe
        self.att_thread = Thread(target=self.send_att, args=())
        self.att_thread.daemon = True
        self.att_thread.start()   

    def start_posctl_thread(self):
        # send setpoints in separate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()        


    # ========= Application functions ===============                    
    def test_attctl(self):
         # boundary to cross
        boundary_x = 200
        boundary_y = 100
        boundary_z = 20
                
        rospy.loginfo("attempting to cross boundary | x: {0}, y: {1}, z: {2}".
                      format(boundary_x, boundary_y, boundary_z))
        # does it cross expected boundaries in 'timeout' seconds?
        timeout = 90  # (int) seconds
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        crossed = False
        for i in xrange(timeout * loop_freq):
            if (self.local_position.pose.position.x > boundary_x and
                    self.local_position.pose.position.y > boundary_y and
                    self.local_position.pose.position.z > boundary_z):
                rospy.loginfo("boundary crossed | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                crossed = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                raise rospy.ROSException(e)
            
        rospy.loginfo("Completed")
        self.set_mode("AUTO.LAND", 5)

    def test_posctl(self):
        """Test offboard position control"""
        # exempting failsafe from lost RC to allow offboard

        positions = ((0, 0, 0), (50, 50, 20), (50, -50, 20), (-50, -50, 20),
                     (0, 0, 20))

        for i in xrange(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], 30)

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':
    rospy.init_node('att_control_px4', anonymous=True)
    offboard_ctl = OffBoardControl()

    # make sure the connection is ready to start the mission
    offboard_ctl.wait_for_topics(60)

    # offboard_ctl.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
    #                             10, -1)  

    rospy.sleep(1)
    offboard_ctl.log_topic_vars()

    # rcl_except = ParamValue(1<<2, 0.0)
    # offboard_ctl.set_param("COM_RCL_EXCEPT", rcl_except, 5)
    # rc_override = ParamValue(3, 0.0) # RC override in both auto and offboard mode
    # offboard_ctl.set_param("COM_RC_OVERRIDE", rc_override, 5)
  
    rospy.loginfo("run mission")

    control_mode = 'pos'
    
    if control_mode =='att':
        offboard_ctl.start_attctl_thread() # send setpoint in separate thread
        offboard_ctl.set_mode("OFFBOARD", 5)
        offboard_ctl.set_arm(True, 5)
        offboard_ctl.test_attctl()
        offboard_ctl.att_thread.join()
              
    elif control_mode =='pos':
        offboard_ctl.start_posctl_thread() # send setpoint in separate thread
        offboard_ctl.set_mode("OFFBOARD", 5)
        offboard_ctl.set_arm(True, 5)
        offboard_ctl.test_posctl()
        offboard_ctl.pos_thread.join()
    rospy.spin()
