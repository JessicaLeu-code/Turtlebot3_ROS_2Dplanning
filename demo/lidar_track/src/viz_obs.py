import numpy as np
from time import sleep
import matplotlib.pyplot as plt
from matplotlib import cm, animation
from util import *
from multiprocessing import Process
from nav_msgs.msg import Path
import rospy

ZEROS = np.zeros((GRID, GRID))

# input: two geometry_msgs/PoseStamped, output (x, y)
def sub_pose2vox(pose1s, pose2s):
    pose1, pose2 = pose1s.pose, pose2s.pose
    x1, y1 = pose1.position.x, pose1.position.y
    x2, y2 = pose2.position.x, pose2.position.y

    return pt2vox(x1 - x2, y1 - y2)

# separate process that displays an updating image based on data from a queue
class Viz(Process):
    def __init__(self, q):
        super(Viz, self).__init__()
        self.running = True
        self.q = q

    def update_img(self):
        # get rid of all data but the newest
        while not self.q.empty():
            self.data = self.q.get_nowait()

        img, real_lines, path = self.data

        try:
            self.ax.imshow(img, cmap=cm.gray)
        except Exception as e:
            print(e)
            print(img)

        if real_lines:
            for line in real_lines:
                #print([norm(i) for i in real_lines])

                # we need to make sure that we "extend" the square out in the right
                # direction. first, we place the opposite edge of the square
                points = line.poly.polygon.points
                print("BADDDDDDDDDDDDDDDDD", points)
                p0 = pt2vox(points[0].x, points[0].y)
                p1 = pt2vox(points[1].x, points[1].y)
                p3 = pt2vox(points[2].x, points[2].y)
                p2 = pt2vox(points[3].x, points[3].y)

                poly = plt.Polygon((p0, p1, p3, p2), color='#33ff33')
                self.ax.add_patch(poly)

                pc = pt2vox(line.mean[0], line.mean[3])
                poly = plt.Polygon(pc, color='#33ff33')
                self.ax.add_patch(poly)

        if path:
            origin = path.poses[0]
            pts = np.array([sub_pose2vox(p, origin) for p in path.poses])

            self.ax.plot(pts[:, 0], pts[:, 1], color='r')

        #turtlebot_w = 0.25 # meters across
        #turtlebot_w_px = turtlebot_w * MAX_RANGE / 2 * GRID
        #ul = add(CENTER, v0=(), v1=)
        #turtlebot = plt.Polygon((ul, ur, lr, ll), color='b')
        #self.ax.add_patch(turtlebot)

        self.ax.set_xlim((0, GRID))
        self.ax.set_ylim((GRID, 0))

    def run(self):
        # BEGIN plot stuff
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.mpl_connect('key_press_event', self.die)

        # plot and show empty image
        self.im = self.ax.imshow(ZEROS, cmap=cm.gray)
        self.ax.set_xlim((0, GRID))
        self.ax.set_ylim((GRID, 0))
        self.fig.show()

        # keep updating image with new data from the queue
        self.data = (ZEROS, [], None)
        def update_plot():
            self.update_img()
            self.fig.canvas.draw()
            self.ax.cla()
            self.fig.canvas.flush_events()

        while self.running:
            update_plot()
            sleep(0.02)

    def die(self, _=None):
        self.running = False

    def is_running(self):
        return self.running
