#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from filip_package.msg import FibonacciAction, FibonacciFeedback, FibonacciResult

class fibonacci_server:
    def __init__(self, name):
        self.name = name
        self.feedback = FibonacciFeedback()
        self.result = FibonacciResult()

        self.action_server = SimpleActionServer(self.name, FibonacciAction, execute_cb= self.execute_callback, auto_start= False)
        self.action_server.start()

    def execute_callback(self, goal):
        rate = rospy.Rate(1)
        success = True
        self.feedback.sequence[:] = []
        self.feedback.sequence.append(0)
        self.feedback.sequence.append(1)
        rospy.loginfo('[{}] Executing, creating fibonacci sequence of order {} with seeds {}, {}'.format(self.name, goal.order, self.feedback.sequence[0], self.feedback.sequence[1]))

        for i in range(1, goal.order):
            if self.action_server.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('[{}] Preempted'.format(self.name))
                success = False
                self.action_server.set_preempted()
                break

            self.feedback.sequence.append(self.feedback.sequence[i-1] + self.feedback.sequence[i])
            self.action_server.publish_feedback(self.feedback)
            rate.sleep()

        if success:
            self.result.sequence = self.feedback.sequence
            rospy.loginfo('[{}] Succeeded'.format(self.name))
            self.action_server.set_succeeded(self.result)

if __name__ == '__main__':
    print "Starting ROS Fibonacci Server module"
    rospy.init_node('fibonacci_server', anonymous=True, log_level=rospy.DEBUG)
    fib = fibonacci_server('fibonacci_server')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shuttinh down ROS Fibonacci Server module"