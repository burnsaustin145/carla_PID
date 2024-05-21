import time
import numpy as np
import matplotlib.pyplot as plt


class DataCollector:
    def __init__(self, Kp, Ki, Kd, name):
        self.x = []
        self.y = []
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.name = name

    # return a numpy array [time, speed] etc
    def collect_data(self, x, y):
        self.x.append(x)
        self.y.append(y)

    def show_data(self, set_point):
        self.x = np.array(self.x)
        self.y = np.array(self.y)
        plt.figure(figsize=(6, 6))
        plt.plot(self.x, self.y)
        plt.plot(self.x, np.full(self.x.shape[0], set_point))
        plt.title(f"{self.name}Kp = {self.Kp}, Ki = {self.Ki}, Kd = {self.Kd}")
        plt.savefig("./data/PID_tuning/" + self.name + str(time.time()) + ".png")
        plt.show()
