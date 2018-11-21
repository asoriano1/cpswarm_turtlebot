#!/usr/bin/env python

import rospy
import smach
import smach_ros
import mavros_msgs

from cpswarm_msgs.msg import *
from swarmros.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from turtlebot_compute_cost.msg import ComputeMapCostAction
from robotnik_leds.srv import *

# define state Idle


class Idle(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Idle')
        # Change LEDs to blue
        blue_leds = leds_valueRequest()
        blue_leds.mode = 77
        blue_leds.msgDefault = 5
        led_serv = rospy.ServiceProxy(
            'robotnik_leds_service/set_color_leds', leds_value)
        try:
            resp = led_serv(blue_leds)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        while True:
            # Check for preempt
            if self.preempt_requested():
                rospy.loginfo("Idle state has been preempted")
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1.0)

        return 'succeeded'


def main():
    rospy.init_node('state_machine_node')

    # Create a TOP level SMACH state machine
    top_sm = smach.StateMachine(['succeeded', 'preempted', 'aborted'])

    # Open the container
    with top_sm:

        #  ===================================== SarThreads =====================================
        # Callback for custom outcomes from SarThreads
        def out_cb(outcome_map):
            if outcome_map['GoHomeEventMonitoring'] == 'invalid':
                rospy.loginfo('Returning goHome Event')
                return 'goHome'

            return 'aborted'

        # Create a Concurrence container
        sarthreads_concurrence = smach.Concurrence(
            outcomes=['goHome', 'aborted'],
            default_outcome='goHome',
            child_termination_cb=lambda so: True,
            outcome_cb=out_cb)

        # Open the container
        with sarthreads_concurrence:

            # ===================================== SarBehavior =====================================
            # Create a State Machine container
            sarbehavior_sm = smach.StateMachine(
                outcomes=['succeeded', 'preempted', 'aborted'])

            # Open the container
            with sarbehavior_sm:

                #  ===================================== IdleThreads =====================================
                # Callback for custom outcomes from IdleThreads
                def out_cb(outcome_map):
                    if outcome_map['IdleEventMonitoring'] == 'invalid':
                        rospy.loginfo('Returning targetFound Event')
                        return 'targetFound'

                    return 'aborted'

                # Create a Concurrence container
                idlethreads_concurrence = smach.Concurrence(
                    outcomes=['targetFound', 'aborted'],
                    default_outcome='targetFound',
                    child_termination_cb=lambda so: True,
                    outcome_cb=out_cb,
                    output_keys=['target_id', 'sender_UUID', 'local_pose'])

                # Open the container
                with idlethreads_concurrence:

                    # ADD Idle to IdleThreads #
                    smach.Concurrence.add('Idle',
                                          Idle()
                                          )

                    def monitor_cb(ud, msg):
                        rospy.loginfo('Executing monitor_cb')
                        ud.target_id = msg.id
                        ud.sender_UUID = msg.swarmio.node
                        ud.local_pose = msg.local_pose
                        return False

                    # ADD IdleEventMonitoring to IdleThreads #
                    smach.Concurrence.add('IdleEventMonitoring',
                                          smach_ros.MonitorState('bridge/events/target_found',
                                                                 LocalTargetPositionEvent,
                                                                 cond_cb=monitor_cb,
                                                                 output_keys=['target_id', 'sender_UUID', 'local_pose']))
                #  ===================================== IdleThreads END =====================================

                # ADD IdleThreads to SarBehavior #
                smach.StateMachine.add('IdleThreads',
                                       idlethreads_concurrence,
                                       transitions={'targetFound': 'ChangeLedsToBlinkGreen'})
 				# ADD ChangeLedsToBlinkGreen to SarBehavior #
                blink_green_leds = leds_valueRequest()
                blink_green_leds.mode = 77
                blink_green_leds.msgDefault = 4
                smach.StateMachine.add('ChangeLedsToBlinkGreen',
                                       smach_ros.ServiceState('robotnik_leds_service/set_color_leds',
                                                              leds_value, request=blink_green_leds),
                                       transitions={'succeeded': 'SelectRover'})

                # ADD SelectRover to SarBehavior #
                smach.StateMachine.add('SelectRover',
                                       smach_ros.SimpleActionState('cmd/compute_cost',
                                                                   ComputeMapCostAction,
                                                                   goal_slots=['target_id', 'sender_UUID', 'local_pose']),
                                       transitions={'succeeded': 'ChangeLedsToGreen', 'aborted': 'IdleThreads'})
                # ADD ChangeLedsToGreen to SarBehavior #
                green_leds = leds_valueRequest()
                green_leds.mode = 77
                green_leds.msgDefault = 3
                smach.StateMachine.add('ChangeLedsToGreen',
                                       smach_ros.ServiceState('robotnik_leds_service/set_color_leds',
                                                              leds_value, request=green_leds),
                                       transitions={'succeeded': 'MoveToTarget', 'aborted': 'IdleThreads'})
                # ADD MoveToTarget to SarBehavior #
                smach.StateMachine.add('MoveToTarget',
                                       smach_ros.SimpleActionState('move_base',
                                                                   MoveBaseAction,
                                                                   goal_slots=['target_pose']),
                                       transitions={
                                           'succeeded': 'Idle2', 'aborted': 'IdleThreads'},
                                       remapping={'target_pose': 'local_pose'})

                # ADD Idle2 to SarBehavior #
                smach.StateMachine.add('Idle2',
                                       Idle(),
                                       transitions={})
            #  ===================================== SarBehavior END =====================================

            # ADD SarBehavior to SarThreads #
            smach.Concurrence.add('SarBehavior', sarbehavior_sm)

            # ADD GoHomeEventMonitoring to SarThreads #
            smach.Concurrence.add('GoHomeEventMonitoring',
                                  smach_ros.MonitorState('bridge/events/go_home',
                                                         SimpleEvent,
                                                         cond_cb=lambda ud, msg: False))
        #  ===================================== SarThreads END =====================================

        # ADD SarThreads to TOP state #
        smach.StateMachine.add('SarThreads',
                               sarthreads_concurrence,
                               transitions={'goHome': 'GoHome'})
        #	transitions={})

        def move_goal_cb(ud, goal):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = 0.0
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.orientation.w = 1
            rospy.loginfo('Going HOME: %.2f, %.2f',
                          goal.target_pose.pose.position.x, target_pose.pose.position.y)
            return goal

        # ADD GoHome to TOP state #
        smach.StateMachine.add('GoHome',
                               smach_ros.SimpleActionState('move_base',
                                                           MoveBaseAction,
                                                           goal_cb=move_goal_cb),
                               transitions={})

    # Create and start the introspection server (uncomment if needed)
    # sis = smach_ros.IntrospectionServer('smach_server', top_sm, '/SM_TOP')
    # sis.start()

    # Execute SMACH plan
    outcome = top_sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    # sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
