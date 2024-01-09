#!/usr/bin/env python

import rosbag
import rospy

def publish_bag_file(bag_file_path, topic_name):
    bag = rosbag.Bag(bag_file_path)
    pub = rospy.Publisher(topic_name, rospy.AnyMsg, queue_size=10)

    rospy.init_node('bag_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    start_time = rospy.Time.now().to_sec()
    for topic, msg, t in bag.read_messages():
        while rospy.Time.now().to_sec() - start_time < t.to_sec():
            rate.sleep()
        pub.publish(msg)

    #bag.close()

if __name__ == '__main__':
    try:
        publish_bag_file('/home/robot/mushr_ws/src/planner/path_to_bag_file.bag', '/path_bag')
    except rospy.ROSInterruptException:
        pass
