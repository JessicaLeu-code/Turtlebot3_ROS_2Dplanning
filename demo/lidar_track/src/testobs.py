#!/usr/bin/env python
from geometry_msgs.msg import Twist, PoseArray, PoseStamped, PolygonStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
import rospy
import rosnode
import numpy as np

def sim():
    rospy.init_node('optimization')
    o = Optimizer()

    node_names = rosnode.get_node_names()
    o.num_obstacles = sum(1 for i in node_names if i.startswith('/obstacle'))
    for i in range(len(node_names)):
    	print(node_names[i])
        if node_names[i].startswith('/obstacle'):
			rospy.Subscriber(node_names[i], PolygonStamped, o.obstacle_callback)
			print(o.obstacles)

    rate = rospy.Rate(2)

    # path = o.optimize()
    # path_pub.publish(path)

    while not rospy.is_shutdown():
        rate.sleep()


class Optimizer:
    def __init__(self):
        self.x = 0.0  # x coordinates in world frame
        self.y = 0.0  # y coordinates in world frame
        self.t = 0.0  # angle from horizontal in world frame
        self.v = 0.0  # linear x velocity
        self.w = 0.0  # angular z velocity
        self.obstacles = []
        # self.obstacles = np.array([[ 2.3,  4.0 ,  5.2,  1.2],
        # [ 0.4,  0.4, -0.5, -0.5],
        # [ 0. ,  0. ,  0. ,  0. ]])

        self.vision = 3.0
        self.margin = 0.3
        self.nobj = 1
        self.nobjsee = self.nobj
        self.nstep = 21

        ### ======================================
        dt = 0.5
        Vx = 0.1

        dim = 2

        # Set up cost function
        # The weight
        self.c = np.array([1,10,200]) # [penalize for change between old and new path | penalize for change in velocity | penalize for change in acceleration]

        # The distance metric between the original path and the new path
        self.Q1 = np.eye(self.nstep*dim)

        # The velocity
        Vconst = np.concatenate([np.concatenate([-np.eye(2), np.eye(2), np.zeros([2,(self.nstep-2)*2])], axis=1),
                                np.concatenate([np.zeros([(self.nstep-1)*2,2]), np.eye((self.nstep-1)*2)], axis=1) 
                                -np.concatenate([np.eye((self.nstep-1)*2), np.zeros([(self.nstep-1)*2,2])], axis=1)])
        V_ratio = np.array([[5, 0], [0, 1]])
        Rpenalty = np.kron(np.eye(self.nstep),V_ratio)
        self.Q2 = Vconst.T.dot(Rpenalty.T.dot(Rpenalty)).dot(Vconst)
        Vref = np.array([[Vx,0]])*dt    
        self.Vref_1 = self.c[1]*np.kron(np.ones([1, self.nstep]).flatten(),Vref).dot((Rpenalty.T.dot(Rpenalty)).dot(Vconst))

        # The acceleration
        Vdiff = np.eye(self.nstep*dim)-np.diag(np.ones([1,(self.nstep-1)*dim]).flatten(),dim)                                 # 1 along diagonal, -1 at 2nd above diagonal
        Adiff = Vdiff - np.diag(np.ones([1,(self.nstep-1)*dim]).flatten(),dim) + np.diag(np.ones([1,(self.nstep-2)*dim]).flatten(),dim*2)    # [1 0 -2 0 1] repeated along/right of diagonal
        self.Q3 = Adiff[0:(self.nstep-2)*dim,:].T.dot(Adiff[0:(self.nstep-2)*dim,:])    
        

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.v = msg.twist.twist.linear.x 
        self.w = msg.twist.twist.angular.z

        _,_,yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.t = yaw

    def obstacle_callback(self, msg):
        if len(self.obstacles) == self.num_obstacles:
            self.obstacles = []
        self.obstacles += [self.polygonstampednp(msg)]
        # self.obstacles = np.array([[1.6, 3.1, 3.4, 1.2], [0.3, 0.3, -0.5, -0.5], [0, 0, 0, 0]])
        # self.obstacles = np.array([[ 2.3,  4.0 ,  5.2,  1.2],
        # [ 0.4,  0.4, -0.5, -0.5],
        # [ 0. ,  0. ,  0. ,  0. ]])

    def polygonstampednp(self, msg):
        arr = np.array([[],[],[]])
        for i in range(len(msg.polygon.points)):
            arr = np.concatenate([arr, self.vectornp(msg.polygon.points[i])], axis=1)
        return arr 

    def vectornp(self, msg): 
        return np.array([[msg.x, msg.y, msg.z]]).T

    def optimize(self):
        #Generate reference trajectory
        # current_x: [x, y, theta, vx, wz]
            z0 = np.array([[self.x], [self.y]])                                       # current: x0, y0
            zT = np.array([[self.x+self.vision], [0]])                                           # target: x0+3, y0
                   
            path = np.concatenate([[np.linspace(z0[0], zT[0],self.nstep)],
                               [np.linspace(z0[1], zT[1],self.nstep)]])

            ## Cost function update
            dir = np.array([zT[0]+6, zT[1]])
            dir_T = (1/np.linalg.norm(dir))*(np.array([zT[1], -zT[0]-6])).T
            dd = np.kron(np.ones([self.nstep,1]), dir_T.dot(np.array([[-6], [0]])))
            D = np.kron(np.eye(self.nstep),dir_T)

            # Distance to reference line
            Q1 = D.T.dot(D)
            Xdis_1 = 2*self.c[0]*dd.T.dot(D)
            # The total costj
            Qref = Q1*self.c[0] + self.Q2*self.c[1] + self.Q3*self.c[2]

            ## Extended cost
            Mcurv = np.eye(self.nstep)
            Mcurv[-1,-1] = 5
            Vcurv = np.eye(self.nstep)-np.diag(np.ones([1,self.nstep-1]).flatten(),1)
            Qcurv = 5*Mcurv

            ## The boundary constraint
            Aeq = np.zeros([2, self.nstep*2 + self.nstep + self.nobj*self.nstep])
            Aeq[0:2, 0:2] = np.eye(2)      
            beq = path[:,0].reshape(2,1)

            ## The Iteration
            refpath = path.flatten('F')
            oripath = refpath[0:2]
            refinput = np.ones([1, self.nstep])

            for k in range(25):
                Lstack = np.array([[]]*84).T
                Sstack = np.array([[]]*1).T
                for i in range(self.nstep):
                        for j in range(self.nobjsee):                                                 # for each object
                            poly = self.obstacles#+obs{j}.v*np.(ones(1,4)*dt*i                 # position of obstacle at time i (initial position, + velocity*time)
                            L,S,d = self.d2poly(refpath[i*2:(i+1)*2].T, poly.T)
                            Lstack = np.concatenate([Lstack, 
                                                     np.concatenate([np.zeros([1,i*2]), L, np.zeros([1,(self.nstep-(i+1))*2]), np.zeros([1,self.nstep]), 
                                                                     np.zeros([1,i*self.nobj+j]), np.array([[-1]]), np.zeros([1,self.nobj*(self.nstep-i)-(j+1)])], axis=1)])
                            Sstack = np.concatenate([Sstack, S-self.margin]) 
                            # Soft constraint
                            Lstack = np.concatenate([Lstack,
                                                     np.concatenate([np.zeros([1,self.nstep*3]), np.zeros([1,i*self.nobj+j]), np.array([[-1]]), np.zeros([1,self.nobj*(self.nstep-i)-(j+1)])], axis=1)])
                            Sstack = np.concatenate([Sstack, np.array([[0]])])

               ## QP
                Qe = scipy.linalg.block_diag(Qref, Qcurv)

                # enlarge A ,f for soft constraint
                H = scipy.linalg.block_diag(Qe, 50*np.diag(np.ones([1,self.nobj*self.nstep]).flatten()))
                f = np.concatenate([-np.concatenate([Xdis_1.T, np.zeros([self.nstep,1])])-np.concatenate([self.Vref_1.T, np.zeros([self.nstep,1])]), 
                                    np.zeros([self.nobj*self.nstep,1])])

                H = cvxopt.matrix(H)
                f = cvxopt.matrix(f)
                Lstack = cvxopt.matrix(Lstack)
                Sstack = cvxopt.matrix(Sstack)
                Aeq = cvxopt.matrix(Aeq)
                beq = cvxopt.matrix(beq)
                cvxopt.solvers.options['show_progress'] = False
                soln = cvxopt.solvers.qp(H,f,Lstack,Sstack,Aeq,beq)
                soln = soln['x']
                pathnew = soln[0:2*self.nstep]
                refinput = soln[2*self.nstep:]

                z0 = np.array([[pathnew[2], pathnew[3]]])
                zT = np.array([[pathnew[2]+self.vision, 0]])

                refpath = pathnew
        
            poseArray = []
            for i in range(self.nstep):
                poses = PoseStamped()
                poses.pose.position.x = refpath[2*i]
                poses.pose.position.y = refpath[2*i+1]
                poseArray += [poses]
            
            waypoints = Path()
            waypoints.header.frame_id = 'odom'
            waypoints.poses = poseArray
            
            return waypoints

    def d2poly(self, point, poly):
        # point = current position
        # poly = current position of polynomial
        poly = poly[:,0:2]
        d = np.inf
        nside = len(poly)                                                                # vertices of obstacle
        for i in range(nside):                                                               # FOR EACH VERTEX
            x1 = poly[i,0]                                                                  # x coord of i-th vertex
            y1 = poly[i,1]                                                                  # y coord of i-th vertex
            x2 = poly[np.mod(i+1,nside),0]                                                    # x coord of next vertex (loops around to 1st vertex when at the last one)
            y2 = poly[np.mod(i+1,nside),1]
            trid1 = np.linalg.norm([x1-x2,y1-y2])                                                  # distance between i-th and i+1-th vertex
            trid2 = np.linalg.norm([x1-point[0],y1-point[1]])                                      # distance between i-th vertex and current location
            trid3 = np.linalg.norm([x2-point[0],y2-point[1]])                                      # distance between i+1-th vertex and current location
            # default case
            Lr = np.array([[y1-y2,x2-x1]])                                                  # [-dy, dx]  
            Sr = np.array([[-x1*y2+x2*y1]])
            vd = abs(Lr[0,0]*point[0]+Lr[0,1]*point[1]-Sr)/trid1                                # |-dy*x0 + dx*y0 - Sr|/trid1                             
            # case 1
            if pow(trid2,2) > pow(trid1,2)+pow(trid3,2):                                                    # theta > 90deg
                vd = trid3                                                                  # i+1-th vertex ix closer
                Lr = np.array([[point[0]-x2,point[1]-y2]])                                  # [x0 - x-coord of i+1-th vertex, y0 - y-coord of i+1-th vertex]
                Sr = np.array([[point[0]-x2,point[1]-y2]]).dot(np.array([[x2],[y2]]))       # deltaX * x2 + deltaY * y2
            # case 2
            if pow(trid3,2) > pow(trid1,2)+pow(trid2,2):
                vd = trid2
                Lr = np.array([[point[0]-x1,point[1]-y1]])
                Sr = np.array([[point[0]-x1,point[1]-y1]]).dot(np.array([[x1],[y1]]))
            if vd < d:                                                                        # if the closer of the 2 vertices is closer than any previous ones
                d = vd                                                                       # new distance = distance to vertex
                L = Lr                                                                       # new x, y distance to vertex
                S = Sr                                                             
                ii = i                                                                       # index of closest vertex
        # determine sign on L, S, d
        nL = np.linalg.norm(L)
        L = L/nL          # normalized of L
        S = S/nL           # S normalized with L
        if L.dot(poly[np.mod(ii+2,nside)].T) < S:    # vertex 2 positions later
            L = -L         # 
            S = -S
        if d == 0:
            return
        area = 0 
        polyarea = 0
        for i in range(nside):
            area = area + self.triArea(point,poly[i,:], poly[np.mod(i+1,nside),:])
        for i in range(1,nside-1):
            polyarea = polyarea + self.triArea(poly[0,:],poly[i,:],poly[np.mod(i,nside)+1,:])
        if np.linalg.norm(polyarea-area) < 0.01:
            d = -d
        return L, S, d

    def triArea(self, p1, p2, p3):
        a = np.linalg.norm(p1-p2)
        b = np.linalg.norm(p1-p3)
        c = np.linalg.norm(p2-p3)
        half = (a+b+c)/2
        return math.sqrt(half*(half-a)*(half-b)*(half-c))

if __name__ == '__main__':
    try:
        sim()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    try:
        sim()
    except rospy.ROSInterruptException:
        pass
