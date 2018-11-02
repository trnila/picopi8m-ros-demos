#!/usr/bin/env python
import rospy
import numpy as np
from numpy import dot
from numpy.linalg import inv
from std_msgs.msg import Float32 
from demo_fake_m4_sensor.msg import Location 
from filterpy.common import Q_discrete_white_noise

# based on excelent book "Kalman and Bayesian Filters in Python"
class KalmanFilter:
    # time step
    dt = 1

    # standard deviation of measurement noise
    R_std = 1000

    # standard deviation of process noise
    Q_std = 0.5

    # current state [pos, vel, accel]
    x = np.array([[0, 0, 0]]).T 

    # covariance matrix - how we trust values
    P = np.diag([500, 500, 500])

    # transition, how we get from current state to the next state
    F = np.array([[1, dt, 0.5*dt**2],
                  [0,  1, dt],
                  [0, 0, 1]])

    # measurement matrix
    # we are measuring only position, because velocity and acceleration
    # is calculated from position
    H = np.array([[1., 0., 0]])

    # measurement noise
    R = np.array([[R_std**2]])

    # process noise
    Q = Q_discrete_white_noise(dim=3, dt=dt, var=Q_std**2)

    def filter(self, z):
        # predict
        self.x = dot(self.F, self.x)
        self.P = dot(self.F, self.P).dot(self.F.T) + self.Q

        #update
        S = dot(self.H, self.P).dot(self.H.T) + self.R
        K = dot(self.P, self.H.T).dot(inv(S))
        y = z - dot(self.H, self.x)
        self.x += dot(K, y)
        self.P = self.P - dot(K, self.H).dot(self.P)

        return self.x


# filter received location from /fake_sensor/location
# and publish filtered value to /kalman/{position, velocity, acceleration}
def filter_location(data):
    x = kalman.filter(data.position)
 
    loc = Location()
    loc.position = x[0]
    loc.velocity = x[1]
    loc.acceleration = x[2]
    pub.publish(loc)

    rospy.loginfo("Filtered value:\n%s", loc)

kalman = KalmanFilter()

rospy.init_node('kalman', anonymous=True)
pub = rospy.Publisher('/kalman', Location, queue_size=10)
rospy.Subscriber("/fake_sensor/location", Location, filter_location)
rospy.spin()

