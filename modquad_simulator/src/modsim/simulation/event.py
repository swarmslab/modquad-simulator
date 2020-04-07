# TODO this file is a candidate to delete
import time


class EventSimulator(object):
    def __init__(self, event_fun, time_step=0.01, max_iter=6000, simul_speed=5):
        """
                
        :type simul_speed: simulation speed. E.g 1, 2,3 are 1x, 2x, and 3x respectively.
        :type event_fun: function that is called for each simulation step.
        :type time_step: time step
        :type max_iter: maximum number of iterations
        """
        self.event_fun = event_fun
        self.time_step = time_step
        self.max_iter = max_iter
        self.simul_speed = simul_speed

    def run(self):
        t = 0
        for i in range(self.max_iter):
            # Current time
            t0 = time.time()

            self.event_fun(i, t)

            # Synchronize with real time
            sleep = self.time_step / self.simul_speed - (time.time() - t0)
            if sleep < 0:
                # print "Slow processing for simulation step. Duration=", (time.time() - t0)
                sleep = 0

            time.sleep(sleep)

            # time step
            t += self.time_step
