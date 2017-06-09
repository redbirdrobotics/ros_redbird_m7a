#!/usr/bin/python

import rospy
from redbird_m7a.msg

def main():
    # Create publisher
    pub = rospy.Publisher('rcp', Map, queue_size=10)

    # Initialize node
    rospy.init_node('rcp_node', anonymous=True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
