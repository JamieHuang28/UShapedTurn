from easydict import EasyDict
import math
import numpy as np

class VehicleModel:
    def __init__(self, wheel_base: float, max_delta: float):
        self.wheel_base = wheel_base
        self.max_delta = max_delta
        self.pose = EasyDict({'x': 0.0, 'y': 0.0, 'yaw': 0.0})
    
    def initPose(self, x, y, yaw):
        self.pose.x = x
        self.pose.y = y
        self.pose.yaw = yaw
    
    def drive(self, velocity: float, delta: float, dt: float):
        self.pose.x = self.pose.x + math.cos(self.pose.yaw) * velocity * dt
        self.pose.y = self.pose.y + math.sin(self.pose.yaw) * velocity * dt
        self.pose.yaw = self.pose.yaw + math.tan(delta) / self.wheel_base * velocity * dt
    
    def path(self, distance, delta):
        trace = EasyDict({'ts':[],'xs':[], 'ys':[], 'yaws':[]})
        kVelocity = 1.0
        kTime = distance / kVelocity
        kDt = 0.1
        for i in np.arange(0, kTime, kDt):
            # drive on step for kDt time period
            self.drive(kVelocity, delta, kDt)
            
            trace.ts.append(i)
            trace.xs.append(self.pose.x)
            trace.ys.append(self.pose.y)
            trace.yaws.append(self.pose.yaw)
        return trace


if __name__ == "__main__":
    import matplotlib
    # matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt

    print("test vehicle model...")
    vm = VehicleModel(3.0, 0.5)
    trace = vm.path(1.0, 0.5)
    print(trace)
    fig, axs = plt.subplots(2,1)
    axs[0].plot(trace.xs, trace.ys)
    axs[0].axis('equal')
    axs[1].plot(trace.ts, trace.yaws)
    plt.savefig('vehicle_model.png')