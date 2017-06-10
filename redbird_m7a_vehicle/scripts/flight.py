#!/usr/bin/python

import rospy
from redbird_m7a.msg

def main():
    # Initialize node
    rospy.init_node('flight_node', anonymous=True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
