import numpy as np
import math
import time

import RobotUtil as rt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Locobot:

	def __init__(self):
		# Robot descriptor taken from URDF file (rpy xyz for each rigid link transform) - NOTE: don't change
		self.Rdesc=[
			[0, 0, 0, 0.0973, 0, 0.1730625], # From robot base to joint1
			[0, 0, 0, 0, 0, 0.04125],
			[0, 0, 0, 0.05, 0, 0.2],
			[0, 0, 0, 0.2002, 0, 0],
			[0, 0, 0, 0.063, 0.0001, 0],
			[0, 0, 0, 0.106525, 0, 0.0050143] # From joint5 to end-effector center
			]
        
		# TODO: Define the axis of rotation for each joint
		'''Note: you will get this from the URDF (interbotix_locobot_description.urdf).
		For example, in the urdf under each joint you will see: (<axis xyz="0 1 0"/>)
		'''
		self.axis= [
			[0, 0, 1],
			[0, 1, 0],
			[0, 1, 0],
			[0, 1, 0],
			[-1, 0, 0],
			[0, 1, 0]
		] 

		#Set base coordinate frame as identity - NOTE: don't change
		self.Tbase= [[1,0,0,0],
			[0,1,0,0],
			[0,0,1,0],
			[0,0,0,1]]
		
		#Initialize matrices - NOTE: don't change this part
		self.Tlink=[] #Transforms for each link (const)
		self.Tjoint=[] #Transforms for each joint (init eye)
		self.Tcurr=[] #Coordinate frame of current (init eye)
		for i in range(len(self.Rdesc)):
			self.Tlink.append(rt.rpyxyz2H(self.Rdesc[i][0:3],self.Rdesc[i][3:6]))
			self.Tcurr.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])
			self.Tjoint.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])

		self.Tlinkzero=rt.rpyxyz2H(self.Rdesc[0][0:3],self.Rdesc[0][3:6])
		self.Tlink[0]=np.matmul(self.Tbase,self.Tlink[0])

		# initialize Jacobian matrix
		self.J=np.zeros((6,5))
		
		self.q=[0.,0.,0.,0.,0.,0.]
		self.ForwardKin([0.,0.,0.,0.,0.])

	def ForwardKin(self,ang):
		'''
		inputs: joint angles
		outputs: joint transforms for each joint, Jacobian matrix
		'''
		self.q[0:-1]=ang
		# TODO: implement your Forward Kinematics here		
		
		# TODO: Compute current joint and end effector coordinate frames (self.Tjoint). Remember than not all joints rotate about the z axis!
		# slide 12
		for i in range(len(self.q)):
			angle = [dir*self.q[i] for dir in self.axis[i]]
			self.Tjoint[i] = rt.rpyxyz2H(angle, np.zeros(3))

			if i==0:
				self.Tcurr[i] = np.matmul(self.Tlink[i], self.Tjoint[i])
			else:
				prev_eff = np.matmul(self.Tcurr[i-1], self.Tlink[i])
				self.Tcurr[i] = np.matmul(prev_eff, self.Tjoint[i])

		# TODO: Compute Jacobian matrix		
		# Slide 25
		for i in range(len(self.Tcurr) - 1):
			rotate_x , rotate_y, rotate_z = self.axis[i]
			p=self.Tcurr[-1][0:3,3]-self.Tcurr[i][0:3,3]
			if abs(rotate_z):
				a=self.Tcurr[i][0:3,2]
			elif abs(rotate_y):
				a=self.Tcurr[i][0:3,1]
			else:
				a=self.Tcurr[i][0:3,0]
			
			self.J[0:3,i]=np.cross(a,p)
			self.J[3:7,i]=a
				
		return self.Tcurr, self.J


	def IterInvKin(self,ang,TGoal):
		'''
		inputs: starting joint angles (ang), target end effector pose (TGoal)

		outputs: computed joint angles to achieve desired end effector pose, 
		Error in your IK solution compared to the desired target
		'''	
		self.ForwardKin(ang)
		
		# tune hyperparameters
		loops = 10000
		transpose_approach = False
		limit_angle=0.5
		
		# For Jacobian Transpose Approach
		alpha = 0.05

		Err=[0,0,0,0,0,0] # error in position and orientation, initialized to 0
		for s in range(loops):
			#TODO: Compute rotation error
			rErrR = np.matmul(TGoal[0:3, 0:3], np.transpose(self.Tcurr[-1][0:3, 0:3]))
			rErrAxis, rErrAng = rt.R2axisang(rErrR)

			# limit angle
			if rErrAng>limit_angle:
				rErrAng=limit_angle
			if rErrAng<-limit_angle:
				rErrAng=-limit_angle

			rErr = [ErrAxis * rErrAng for ErrAxis in rErrAxis]

			#TODO: Compute position error
			xErr=TGoal[0:3,3] - self.Tcurr[-1][0:3,3]
			if np.linalg.norm(xErr)>0.01:
				xErr=xErr * 0.01 / np.linalg.norm(xErr)

			#TODO: Update joint angles 
			Err[0:3]=xErr
			Err[3:6]=rErr
			
			if transpose_approach:
				# Jacobian Transpose Approach (Slide 33)
				self.q[0:-1] = self.q[0:-1] + alpha * np.matmul(np.transpose(self.J), Err)
			else:
				# Jacobian Pseudo-Inverse Approach (Slide 45)
				self.q[0:-1] = self.q[0:-1] + np.matmul(np.matmul(np.transpose(self.J), np.linalg.pinv(np.matmul(self.J, np.transpose(self.J)))), Err)
			
			#TODO: Recompute forward kinematics for new angles
			self.ForwardKin(self.q[0:-1])
		
		approach_name = "Jacobian Transpose Approach" if transpose_approach else "Jacobian Pseudo-Inverse Approach"
		print ("Approach:{}".format(approach_name))
		print ("Hyperparameters setting: loops {}, limit_angle {}, alpha {}".format(loops, limit_angle, alpha))
		import statistics
		Err = [abs(item) for item in Err]
		print ("Total Abs Error: {}, Abs Median Error: {}".format(sum(Err), statistics.median(Err)))
		return self.q[0:-1], Err


	def PlotSkeleton(self,ang):
		'''
		optional function: this code plots the output of you FK
		'''
		#Compute forward kinematics for ang
		self.ForwardKin(ang)

		#Create figure
		fig =plt.figure()
		ax = fig.add_subplot(111,projection='3d')

		#Draw links along coordinate frames 
		for i in range(len(self.Tcurr)):
			ax.scatter(self.Tcurr[i][0,3], self.Tcurr[i][1,3], self.Tcurr[i][2,3], c='k', marker='.')
			if i is 0:
				ax.plot([0,self.Tcurr[i][0,3]], [0,self.Tcurr[i][1,3]], [0,self.Tcurr[i][2,3]], c='b')
			else:
				ax.plot([self.Tcurr[i-1][0,3],self.Tcurr[i][0,3]], [self.Tcurr[i-1][1,3],self.Tcurr[i][1,3]], [self.Tcurr[i-1][2,3],self.Tcurr[i][2,3]], c='k')

		#Format axes and display
		ax.axis('equal')
		ax.set(xlim=(-1., 1.), ylim=(-1., 1.), zlim=(0,1.))
		ax.set_xlabel('X-axis')
		ax.set_ylabel('Y-axis')
		ax.set_zlabel('Z-axis')

		plt.show()


