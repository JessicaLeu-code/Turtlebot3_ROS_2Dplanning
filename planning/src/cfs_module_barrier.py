#!/usr/bin/env python
import numpy as np
import math
import scipy.linalg
import cvxopt

class Cfs:
    def __init__(self, nstep=21, margin=0):
        self.vision = 3.0                   # how far ahead to plan path
        self.margin = margin                # buffer around obstacle
        self.nstep = nstep                  # length of path
        self.c = np.array([0.1,20,5])       # [penalize for change in path | change in velocity | change in acceleration]
        self.Q2 = []                        # velocity change matrix
        self.Q3 = []                        # acceleration change matrix
        self.Vref_1 = []
        self.x = []                         # [x y yaw v w]
        self.z0 = []                        # [x0 y0]
        self.zT = []                        # [xT yT]
        self.z00 = []
        self.zG = []
        self.path = []                      # [[x0 x1 ... xN], [y0 y1 ... yN]]
        self.nobj = 0
        self.dt = 0.5
        self.set_costs()
        self.barrier = 1.5

    def set_costs(self):
        Vx = 0.1

        # velocity
        Vconst = np.concatenate([np.concatenate([-np.eye(2), np.eye(2), np.zeros([2,(self.nstep-2)*2])], axis=1),
                                np.concatenate([np.zeros([(self.nstep-1)*2,2]), np.eye((self.nstep-1)*2)], axis=1)
                                -np.concatenate([np.eye((self.nstep-1)*2), np.zeros([(self.nstep-1)*2,2])], axis=1)])
        V_ratio = np.array([[1, 0], [0, 1]])
        Rpenalty = np.kron(np.eye(self.nstep),V_ratio)
        self.Q2 = Vconst.T.dot(Rpenalty.T.dot(Rpenalty)).dot(Vconst)
        # Vref = np.array([[Vx,0]])*self.dt
        # self.Vref_1 = self.c[1]*np.kron(np.ones([1, self.nstep]).flatten(),Vref).dot((Rpenalty.T.dot(Rpenalty)).dot(Vconst))
        
        # acceleration
        Vdiff = np.eye(self.nstep*2)-np.diag(np.ones([1,(self.nstep-1)*2]).flatten(),2)                                 # 1 along diagonal, -1 at 2nd above diagonal
        Adiff = Vdiff - np.diag(np.ones([1,(self.nstep-1)*2]).flatten(),2) + np.diag(np.ones([1,(self.nstep-2)*2]).flatten(),2*2)    # [1 0 -2 0 1] repeated along/right of diagonal
        self.Q3 = Adiff[0:(self.nstep-2)*2,:].T.dot(Adiff[0:(self.nstep-2)*2,:])

    def set_margin(self, margin):

        self.margin = margin

    def set_barrier(self, barrier):
        self.barrier = barrier

    def update_pos(self, x):
        self.x = x
        self.z0 = np.array([[x[0]], [x[1]]])                 # current: x0, y0
        self.zT = self.z0
        goalVector = self.zG - self.zT
        L = np.linalg.norm(goalVector)
        if L == 0:
            self.zT = self.zT
        elif L > 3:
            self.zT = self.zT + (3/L) * goalVector
        else:
            self.zT = self.zT + goalVector
        self.path = np.concatenate([[np.linspace(self.z0[0], self.zT[0],self.nstep)],
                           [np.linspace(self.z0[1], self.zT[1],self.nstep)]])

    def update_cost(self):
        #direction = np.array([self.zT[0]-self.z00[0], self.zT[1]-self.z00[1]])
        #dir_T = (1/np.linalg.norm(direction))*(np.array([self.zT[1], -self.zT[0]-6])).T
        # dd = np.kron(np.ones([self.nstep,1]), dir_T.dot(np.array([[-6], [0]])))
        dd = np.kron(np.ones([self.nstep,1]), np.array([self.zG[0],self.zG[1]]))
        # D = np.kron(np.eye(self.nstep),dir_T)
        # print(np.shape(D))
        Q1 = np.eye(self.nstep*2)
        Xdis_1 = self.c[0]*dd.T.dot(Q1)
        self.Vref_1 = self.c[1]*dd.T.dot(self.Q2)
        # Q1 = D.T.dot(D)
        Qref = Q1*self.c[0] + self.Q2*self.c[1] + self.Q3*self.c[2]
        Mcurv = np.eye(self.nstep)
        Mcurv[-1,-1] = 5
        Qcurv = 5*Mcurv
        Qe = scipy.linalg.block_diag(Qref, Qcurv)
        H = scipy.linalg.block_diag(Qe, 50*np.diag(np.ones([1,(self.nobj+1)*self.nstep]).flatten()))
        f = np.concatenate([-np.concatenate([Xdis_1.T, np.zeros([self.nstep,1])])-np.concatenate([self.Vref_1.T, np.zeros([self.nstep,1])]),
                            np.zeros([(self.nobj+1)*self.nstep,1])])

        Aeq = np.zeros([2, self.nstep*2 + self.nstep + (self.nobj+1)*self.nstep])
        Aeq[0:2, 0:2] = np.eye(2)
        beq = self.path[:,0].reshape(2,1)
        beq = beq.astype(np.double)

        H = cvxopt.matrix(H)
        f = cvxopt.matrix(f)
        Aeq = cvxopt.matrix(Aeq)
        beq = cvxopt.matrix(beq)

        return H, f, Aeq, beq

    def optimize(self, obstacles, vels, x, goal, human=None):
        self.nobj = len(obstacles)
        self.z00 = self.z0
        if goal == None:
            goal = [0,0]
        self.zG = np.array([[goal[0]], [goal[1]]])  
        self.update_pos(x)

        H, f, Aeq, beq = self.update_cost()

        # The Iteration
        refpath = self.path.flatten('F')

        save1,save2= 0,0

        for k in range(10):
            Lstack = np.array([[]]*((self.nstep*3)+self.nstep*self.nobj+self.nstep)).T
            Sstack = np.array([[]]*1).T
            for i in range(self.nstep):
                for j in range(self.nobj):
                    # for each object
                    poly = obstacles[j]+vels[j]*np.ones((1,4))*self.dt*i                  # position of obstacle at time i (initial position, + velocity*time)
                    L,S,d = self.d2poly(refpath[i*2:(i+1)*2].T, poly.T)
                    
                    Lstack = np.concatenate([Lstack,
                                             np.concatenate([np.zeros([1,i*2]), L, np.zeros([1,(self.nstep-(i+1))*2]), np.zeros([1,self.nstep]),
                                                             np.zeros([1,i*self.nobj+j]), np.array([[-1]]), np.zeros([1,self.nobj*(self.nstep-i)-(j+1)]), np.zeros([1,self.nstep])], axis=1)])
                    Sstack = np.concatenate([Sstack, S-self.margin])
                    # Soft constraint
                    Lstack = np.concatenate([Lstack, np.concatenate([np.zeros([1,self.nstep*3]), np.zeros([1,i*self.nobj+j]), np.array([[-1]]), np.zeros([1,self.nobj*(self.nstep-i)-(j+1)]), np.zeros([1,self.nstep])], axis=1)])
                    Sstack = np.concatenate([Sstack, np.array([[0]])])

            for i in range(self.nstep):
                if human:
                    l1,l2, S = self.gradient(refpath[i*2:(i+1)*2].T, human)
                else:
                    l1,l2, S = self.gradient(refpath[i*2:(i+1)*2].T, goal)
                if not l2 == 0:
                    if i == 0:
                        save1 = -l1/l2
                        save2 = -S/l2
                Lstack = np.concatenate([Lstack,
                                         np.concatenate([np.zeros([1,i*2]), np.array([[l1, l2]]), np.zeros([1,(self.nstep-(i+1))*2]), np.zeros([1,self.nstep+self.nstep*self.nobj]),
                                                         np.zeros([1,i]), np.array([[-1]]), np.zeros([1,self.nstep-(i+1)])], axis=1)])
                Sstack = np.concatenate([Sstack, np.array([[-S]])])
                Lstack = np.concatenate([Lstack, np.concatenate([np.zeros([1,self.nstep*(3+self.nobj)]), np.zeros([1,i]), np.array([[-1]]), np.zeros([1,self.nstep-(i+1)])], axis=1)])
                Sstack = np.concatenate([Sstack, np.array([[0]])])

            # Velocity constraint
            # for i in range(self.nstep - 1):
            #     Lstack = np.concatenate([Lstack,
            #                                  np.concatenate([np.zeros([1,i*2]), np.array([[1, 0, -1, 0]]), np.zeros([1,(self.nstep-(i+2))*2 ]), np.zeros([1,self.nstep+self.nstep*self.nobj]),
            #                                              np.zeros([1,i]), np.array([[-1]]), np.zeros([1,self.nstep-(i+1)])], axis=1)])
            #     Lstack = np.concatenate([Lstack,
            #                                  np.concatenate([np.zeros([1,i*2]), np.array([[-1, 0, 1, 0]]), np.zeros([1,(self.nstep-(i+2))*2 ]), np.zeros([1,self.nstep+self.nstep*self.nobj]),
            #                                              np.zeros([1,i]), np.array([[-1]]), np.zeros([1,self.nstep-(i+1)])], axis=1)])
            #     Lstack = np.concatenate([Lstack,
            #                                  np.concatenate([np.zeros([1,i*2]), np.array([[0, 1, 0, -1]]), np.zeros([1,(self.nstep-(i+2))*2 ]), np.zeros([1,self.nstep+self.nstep*self.nobj]),
            #                                              np.zeros([1,i]), np.array([[-1]]), np.zeros([1,self.nstep-(i+1)])], axis=1)])
            #     Lstack = np.concatenate([Lstack,
            #                                  np.concatenate([np.zeros([1,i*2]), np.array([[0, -1, 0, 1]]), np.zeros([1,(self.nstep-(i+2))*2 ]), np.zeros([1,self.nstep+self.nstep*self.nobj]),
            #                                              np.zeros([1,i]), np.array([[-1]]), np.zeros([1,self.nstep-(i+1)])], axis=1)])
            #     Sstack = np.concatenate([Sstack, np.array([[0.02]])])
            #     Sstack = np.concatenate([Sstack, np.array([[0.02]])])
            #     Sstack = np.concatenate([Sstack, np.array([[0.02]])])
            #     Sstack = np.concatenate([Sstack, np.array([[0.02]])])
            Lstack = cvxopt.matrix(Lstack)
            Sstack = cvxopt.matrix(Sstack)
            cvxopt.solvers.options['show_progress'] = False
            soln = cvxopt.solvers.qp(H,f,Lstack,Sstack,Aeq,beq)
            soln = soln['x']
            refpath = soln[0:2*self.nstep]
        return refpath, (save1, save2)

    def d2poly(self, point, poly):
        # point = current position
        # poly = current position of polynomial
        poly = poly[:,0:2]
        d = np.inf
        nside = len(poly)  
        L = None                                                                 # vertices of obstacle
        for i in range(nside):                                                              # FOR EACH VERTEX
            x1 = poly[i,0]                                                                  # x coord of i-th vertex
            y1 = poly[i,1]                                                                  # y coord of i-th vertex
            x2 = poly[np.mod(i+1,nside),0]                                                  # x coord of next vertex (loops around to 1st vertex when at the last one)
            y2 = poly[np.mod(i+1,nside),1]
            trid1 = np.linalg.norm([x1-x2,y1-y2])                                           # distance between i-th and i+1-th vertex
            trid2 = np.linalg.norm([x1-point[0],y1-point[1]])                               # distance between i-th vertex and current location
            trid3 = np.linalg.norm([x2-point[0],y2-point[1]])                               # distance between i+1-th vertex and current location
            # default case
            Lr = np.array([[y1-y2,x2-x1]])                                                  # [-dy, dx]
            Sr = np.array([[-x1*y2+x2*y1]])
            vd = abs(Lr[0,0]*point[0]+Lr[0,1]*point[1]-Sr)/trid1                            # |-dy*x0 + dx*y0 - Sr|/trid1
            # case 1
            if pow(trid2,2) > pow(trid1,2)+pow(trid3,2):                                    # theta > 90deg
                vd = trid3                                                                  # i+1-th vertex ix closer
                Lr = np.array([[point[0]-x2,point[1]-y2]])                                  # [x0 - x-coord of i+1-th vertex, y0 - y-coord of i+1-th vertex]
                Sr = np.array([[point[0]-x2,point[1]-y2]]).dot(np.array([[x2],[y2]]))       # deltaX * x2 + deltaY * y2
            # case 2
            if pow(trid3,2) > pow(trid1,2)+pow(trid2,2):
                vd = trid2
                Lr = np.array([[point[0]-x1,point[1]-y1]])
                Sr = np.array([[point[0]-x1,point[1]-y1]]).dot(np.array([[x1],[y1]]))
            if vd < d or L is None:                                                                      # if the closer of the 2 vertices is closer than any previous ones
                d = vd                                                                      # new distance = distance to vertex
                L = Lr                                                                      # new x, y distance to vertex
                S = Sr
                ii = i                                                                      # index of closest vertex
        # determine sign on L, S, d
        nL = np.linalg.norm(L)
        L = L/nL                                                                            # normalized of L
        S = S/nL                                                                            # S normalized with L
        if L.dot(poly[np.mod(ii+2,nside)].T) < S:                                           # vertex 2 positions later
            L = -L 
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

    def gradient(self, point, center):
        x0 = center[0]
        y0 = center[1]
        x_r = point[0]
        y_r = point[1]
        margin = self.barrier
        dist = np.sqrt((x_r - x0)**2 + (y_r - y0)**2)
        if dist == 0:
            return 0,0,0
        x_val = -(x_r - x0) / dist
        y_val = -(y_r - y0) / dist
        S_val = margin - dist - x_val*x_r - y_val*y_r
        return x_val, y_val, S_val