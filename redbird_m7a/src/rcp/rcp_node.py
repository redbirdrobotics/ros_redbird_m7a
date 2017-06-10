#!/usr/bin/python

import rospy

def main():
    # Initialize node
    rospy.init_node('rcp_node', anonymous=True)

    # Test log
    rospy.loginfo("Redbird Control Panel started!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
