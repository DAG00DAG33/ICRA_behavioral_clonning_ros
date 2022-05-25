import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import torch
import numpy as np

import torch.nn as nn
import torch.nn.functional as F


import importlib.util
import sys


class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.pipe = nn.Sequential(
            nn.Dropout(p=0.3),
            nn.Conv1d(1, 5, 40, stride = 5),
            nn.ReLU(),
            nn.Conv1d(5, 5, 20, stride = 3),
            nn.ReLU(),
            nn.Conv1d(5, 7, 15, stride = 2),
            nn.ReLU(),
            nn.Flatten(),
            nn.Dropout(p=0.1),
            nn.Linear(175, 2),
            nn.Sigmoid(),
            #nn.Softmax(dim=1)
        )

    def forward(self, x):
        return self.pipe(x)


class Model_inference_node(Node):
    def __init__(self):
        super().__init__('model_inference_node')
        self.publisher = self.create_publisher(AckermannDriveStamped,'drive', 10)


        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        #self.model1 = Net()
        #self.model2 = Net()
        #self.model = Net()
        #self.model = torch.load('/home/data/model.pt').to(device)
        self.model = torch.jit.load('/home/models/model3.pt')
        self.max_distance = 10

    def listener_callback(self, scan):
            ranges = [range if range < self.max_distance else self.max_distance for range in scan.ranges]
            #for i in reversed(range(len(ranges))):
                #if i > 135 and i < 225:
                    #ranges.pop(i)
            ranges = np.array(ranges)
            labels = self.model(torch.tensor(ranges.reshape(1, 1, -1)).float())
            drive = AckermannDriveStamped()
            print(labels)
            drive.drive.speed = labels[0][0].item() * 5.
            if drive.drive.speed < 0:
                drive.drive.speed = 0.0
            drive.drive.steering_angle = labels[0][1].item()
            self.publisher.publish(drive)


def main(args=None):
    rclpy.init(args=args)
    model_inference_node_instance = Model_inference_node()

    rclpy.spin(model_inference_node_instance)
    model_inference_node_instance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
