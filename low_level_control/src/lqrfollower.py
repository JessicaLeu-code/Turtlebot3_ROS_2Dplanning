#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist

class Lqr:

	def __init__(self, NILQR=21):
		""" initialize LQR controller
		NILQR: length of reference waypoints
		"""

		# constants
		self.n = NILQR											# NILQR, N, n, var.nstep
		self.dt = 0.5											# dt, Dt, ts
		self.nx = 5												# var.nx, nx
		self.nu = 2												# self.nu, nu
		self.thres = 1e-4										# var.thres
		self.line_search_thres = 1e-4							# var.lineSearchThres
		self.l2 = np.diag([70, 70, 0, 0, 0, 10, 10])			# L2

		self.l1 = np.zeros((7, self.n))							# L1
		self.u = np.zeros((2, self.n-1))						# u - control input				
		self.x = np.zeros((5, self.n))							# x - states [x; y; ?; vx; wz]

		# input
		self.lqr_ref = np.zeros((5, self.n))					# reference path - [x; y; 0; 0; 0]

	def set_reference(self, ref, t):
		self.lqr_ref[0:2,:] = ref
		self.lqr_ref[2, 0] = t 
		self.l1 = np.zeros((7, self.n))
		self.u = np.zeros((2, self.n-1))
		self.x = np.zeros((5, self.n))

	def forward_sim_car(self):
		x = np.zeros((self.nx, self.n))
		x[:,0] = self.lqr_ref[:,0]
		cost = 0

		for i in range(0, self.n-1):
			thk = x[2,i]
			vk = x[3,i]
			thdk = x[4,i]
			ak = self.u[0,i]
			thak = self.u[1,i]
			thm = thk + 0.5*self.dt*thdk

			x[0,i+1] = x[0,i] + vk*self.dt*np.cos(thm)
			x[1,i+1] = x[1,i] + vk*self.dt*np.sin(thm)
			x[2,i+1] = x[2,i] + self.dt*thdk
			x[3,i+1] = x[3,i] + self.dt*ak
			x[4,i+1] = x[4,i] + self.dt*thak

			dz = np.concatenate((x[:,i+1]-self.lqr_ref[:,i+1], self.u[:,i]))
			cost += 0.5*dz.T.dot(self.l2).dot(dz)
		self.x = x
		return cost

	def linearize_ab_car(self):
		a = np.zeros((self.n-1, self.nx, self.nx))
		b = np.array([[0,0],[0,0],[0,0],[self.dt,0], [0,self.dt]])

		for i in range(0, self.n-1):
			thk = self.x[2,i]
			vk = self.x[3,i]
			thdk = self.x[4,i]
			thm = thk + 0.5*self.dt*thdk

			a[i] = np.array([[1, 0, -vk*self.dt*np.sin(thm), self.dt*np.cos(thm), -0.5*pow(self.dt,2)*vk*np.sin(thm)],
							 [0, 1, vk*self.dt*np.cos(thm), self.dt*np.sin(thm), 0.5*pow(self.dt, 2)*vk*np.cos(thm)],
							 [0, 0, 1, 0, self.dt],
							 [0, 0, 0, 1, 0],
							 [0, 0, 0, 0, 1]])
		return a,b

	def modify_cost(self):
		for i in range(0, self.n-1):
			self.l1[:,i] = self.l2.dot(np.concatenate((self.x[:,i].T, self.u[:,i].T))) - self.l2.dot(np.concatenate((self.lqr_ref[:,i].T, np.zeros(2))))
		self.l1[0:self.nx,-1] = self.l2[0:self.nx, 0:self.nx].dot(self.x[:,-1]) - self.l2[0:self.nx, 0:self.nx].dot(self.lqr_ref[:,-2])

	def stochastic_lqr(self, a, b):
		vxx = self.l2[0:self.nx, 0:self.nx]
		vx = self.l1[0:self.nx, self.n-1]
		ka = np.zeros((self.nu, self.n-1))
		kb = np.zeros((self.n-1, self.nu, self.nx))
		sigs = np.zeros((self.n-1, self.nu, self.nu))

		for i in range(self.n-2, -1, -1):
			qxx = self.l2[0:self.nx, 0:self.nx] + a[i].T.dot(vxx).dot(a[i])
			quu = self.l2[self.nx:, self.nx:] + b.T.dot(vxx).dot(b)
			qux = self.l2[self.nx:, 0:self.nx] + b.T.dot(vxx).dot(a[i])
			qx = self.l1[0:self.nx, i] + a[i].T.dot(vx)
			qu = self.l1[self.nx:, i] + b.T.dot(vx)
			sig = np.linalg.inv(quu)
			sigs[i] = sig
			vx = qx - qux.T.dot(sig).dot(qu)
			vxx = qxx - qux.T.dot(sig).dot(qux)
			ka[:,i] = -sig.dot(qu)
			kb[i] = -sig.dot(qux)

		return ka, kb, sigs

	def execute_policy(self, ka, kb):
		x = np.zeros((self.nx, self.n))
		x[:,0] = self.lqr_ref[:,0]
		cost = 0
		u = np.zeros((self.nu, self.n-1))

		for i in range(0, self.n-1):
			u[:,i] = ka[:,i] + kb[i].dot(x[:,i])
			thk = x[2,i]
			vk = x[3,i]
			thdk = x[4,i]
			ak = u[0,i]
			thak = u[1,i]
			thm = thk + 0.5*self.dt*thdk

			x[0,i+1] = x[0,i] + vk*self.dt*np.cos(thm)
			x[1,i+1] = x[1,i] + vk*self.dt*np.sin(thm)
			x[2,i+1] = x[2,i] + self.dt*thdk
			x[3,i+1] = x[3,i] + self.dt*ak
			x[4,i+1] = x[4,i] + self.dt*thak

			dz = np.concatenate((x[:,i+1]-self.lqr_ref[:,i+1], u[:,i]))
			cost += 0.5*dz.T.dot(self.l2).dot(dz)
		return x, u, cost

	def get_lqr_reference(self, ref, x):
		""" runs LQR controller
		INPUT
		ref: reference waypoints
		x: current state [x, y, t, v, w]
		OUTPUT
		self.x: new waypoints
		"""
		if len(ref) is 0:
			return
		self.set_reference(ref, x[2])

		max_step = 100

		cost = self.forward_sim_car()
		J = np.zeros(max_step)
		J[0] = cost

		for i in range(0, max_step-1):
			stop = 0
			a, b = self.linearize_ab_car()
			self.modify_cost()
			ka, kb, sigs = self.stochastic_lqr(a,b)
			k1 = ka
			step = 1.0
			while(1):
				for j in range(0, self.n-1):
					k1[:,j] = self.u[:,j] + step*ka[:,j] - kb[j].dot(self.x[:,j])
				x1, u1, cost1 = self.execute_policy(ka, kb)
				if cost1 < cost:
					cost = cost1
					J[i+1] = cost
					self.x = x1
					self.u = u1
					break
				step *= 0.5
				if step < self.line_search_thres:
					stop = 1
					self.x = x1
					self.u = u1
					break
			if stop == 1:
				break
			if J[i+1] is not 0 and (J[i] - J[i+1])/J[i+1] < self.thres:
				break
		return self.x

		