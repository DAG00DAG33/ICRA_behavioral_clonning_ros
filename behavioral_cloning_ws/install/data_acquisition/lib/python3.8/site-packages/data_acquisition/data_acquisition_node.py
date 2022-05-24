import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time



import numpy as np

import message_filters


class Data_adquisition_node(Node):
    def __init__(self):
        super().__init__('behavioral_adquisition_node')
        sub_laser = message_filters.Subscriber(self, LaserScan, 'scan')
        sub_driver = message_filters.Subscriber(self, AckermannDriveStamped, 'drive')
        rs = message_filters.ApproximateTimeSynchronizer([sub_laser, sub_driver], 1, 0.1)
        rs.registerCallback(self.listener_callback)

        self.last_msg_time = time.time();


        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_distance', 10.0),
                ('frecuency', 0.1),
                ('max_speed', 2),
                ('initial_count', 0),
                ('data_dir', 'home/data/testing/')
            ])

        self.max_distance=self.get_parameter('max_distance').value
        self.frecuency=self.get_parameter('frecuency').value
        self.max_speed=self.get_parameter('max_speed').value
        self.initial_count=self.get_parameter('initial_count').value
        self.data_dir=self.get_parameter('data_dir').value

        self.count = self.initial_count
        self.get_logger().info("Hello {}".format(self.max_speed))

    def listener_callback(self, laser, drive):
        #self.get_logger().info('angle_min: {}, angle_max: {}, angle_increment: {}, len(range): {}'.format(msg.angle_min, msg.angle_max, msg.angle_increment, len(msg.ranges)))
        if time.time() - self.last_msg_time > self.frecuency:
            self.last_msg_time = time.time()
            ranges = [range if range < self.max_distance else self.max_distance for range in laser.ranges]
            for i in reversed(range(len(ranges))):
                if i > 135 and i < 225:
                    ranges.pop(i)
            ranges = np.array(ranges)
            #self.get_logger().info('drive: {}'.format(drive))
            #self.get_logger().info('ranges: {}'.format(ranges))
            #self.get_logger().info('time: {}'.format(time.time())
            labels = np.array([drive.drive.speed / self.max_speed, drive.drive.steering_angle])
            self.get_logger().info('Y: {}'.format(labels))
            np.save(self.data_dir + 'Y_' + str(self.count) + '.npy' , labels)
            np.save(self.data_dir + 'X_' + str(self.count) + '.npy' , ranges)
            self.count += 1

        #header:
         #stamp:I
            #sec: 0
            #nanosec: 0
          #frame_id: ''
        #angle_min: 0.0
        #angle_max: 0.0
        #angle_increment: 0.0
        #time_increment: 0.0
        #scan_time: 0.0
        #range_min: 0.0
        #range_max: 0.0
        #ranges: []
        #i#ntensities: []

        #header:
          #stamp:
            #sec: 0
            #nanosec: 0
          #frame_id: ''
        #drive:
          #steering_angle: 0.0
          #steering_angle_velocity: 0.0
          #speed: 0.0
          #acceleration: 0.0
          #jerk: 0.0
        #"



def main(args=None):
    rclpy.init(args=args)
    data_adquisition_node_instance = Data_adquisition_node()
    rclpy.spin(data_adquisition_node_instance)
    data_adquistion_node_instacne.destroy_node()
    rclpt.shutdown()

if __name__ == '__main__':
    main()

