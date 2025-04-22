#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from dynus_interfaces.msg import DynTraj, PWPTraj, CoeffPoly3

class DummyTrajPublisher(Node):
    def __init__(self):
        super().__init__("dummy_traj_publisher")

        # create a publisher to publish the dummy trajectory
        self.publisher_ = self.create_publisher(DynTraj, "/dummy_traj", 10)

        # parameters
        pwp_duration = 2.666666

        # create a pwp 
        self.pwp = PWPTraj()

        # get the current time
        current_time_sec = self.get_clock().now().seconds_nanoseconds()[0]
        current_time_nanosec = self.get_clock().now().seconds_nanoseconds()[1]

        # set times for the pwp
        for i in range(6):
            self.pwp.times.append(current_time_sec + current_time_nanosec * 1e-9 + i * pwp_duration)

        # set the coefficients for the pwp
        self.get_coefficients()

        # create a dummy trajectory (DynTraj)
        msg = DynTraj()
        msg.header.stamp.sec = current_time_sec
        msg.header.stamp.nanosec = current_time_nanosec
        msg.header.frame_id = "map"
        msg.bbox.append(0.5)
        msg.bbox.append(0.5)
        msg.bbox.append(0.5)
        msg.id = 0
        msg.pwp = self.pwp

        # publish the dummy trajectory
        self.publisher_.publish(msg)

    def get_coefficients(self):
        
        # coefficients for x
        coeff = CoeffPoly3()
        coeff.a = 0.014905048491092383
        coeff.b = 0.0
        coeff.c = 0.0
        coeff.d = -4.0
        self.pwp.coeff_x.append(coeff)

        coeff = CoeffPoly3()
        coeff.a = -0.006649712933214234
        coeff.b = 0.11924039148237954
        coeff.c = 0.31797438676272033
        coeff.d = -3.717356092231915
        self.pwp.coeff_x.append(coeff)

        coeff = CoeffPoly3()
        coeff.a = -0.010248125509450152
        coeff.b = 0.06604268643125055
        coeff.c = 0.8120626092573766
        coeff.d = -2.1475909169643463
        self.pwp.coeff_x.append(coeff)

        coeff = CoeffPoly3()
        coeff.a = -0.008095337069714983
        coeff.b = -0.015942320087694195
        coeff.c = 0.945663590155146
        coeff.d = 0.2932115063566476
        self.pwp.coeff_x.append(coeff)

        coeff = CoeffPoly3()
        coeff.a = -0.0009913400548245332
        coeff.b = -0.08070501857549284
        coeff.c = 0.6879373460391397
        coeff.d = 2.5481019482378535
        self.pwp.coeff_x.append(coeff)

        # coefficients for y
        for i in range(6 - 1):
            coeff = CoeffPoly3()
            coeff.a = 0.0
            coeff.b = 0.0
            coeff.c = 0.0
            coeff.d = 0.0
            self.pwp.coeff_y.append(coeff)
        
        for i in range(6 - 1):
            coeff = CoeffPoly3()
            coeff.a = 0.0
            coeff.b = 0.0
            coeff.c = 0.0
            coeff.d = 2.5
            self.pwp.coeff_z.append(coeff)

def main(args=None):
    rclpy.init(args=args)
    node = DummyTrajPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
