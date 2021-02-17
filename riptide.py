#!/usr/bin/env python3

from roblib import *
from AUV import AUV

############################  Class Riptide definition  ##################################

class Riptide(AUV):
    p1,p2,p3,p4 = 1, 1, 1, 1
    
    B= matmul( np.diag([p3,p4,p4]), 
            array([[-1.,       -1.,            -1.],
                   [0.,  sin(2*pi/3), -sin(2*pi/3)],
                   [1., cos(2*pi/3), cos(2*pi/3)]]))
    
    
    def evolX(self, u):
        """Method that returns the derivative of the state vector, given the current state vector and the input commands."""
        
        x=self.X.flatten()    # x,y,z,phi,theta,psi,v
        u=u.flatten()
        u0,u1,u2,u3=list(u)
        phi,theta,psi=list(x[3:6])
        phi,theta,psi = sawtooth(phi), sawtooth(theta), sawtooth(psi)
        v=x[6]
        

        wr = matmul( (v**2)*self.B , array([[sin(u1)],[sin(u2)],[sin(u3)]]) )

        E = eulermat(phi,theta,psi)

        dp = matmul( E , array([[v],[0],[0]]) )

        dAngles = matmul( eulerderivative(phi,theta,psi) , wr )

        #print( "Rotation: \t", wr.T)
        
        dv =self.p1*u0*abs(u0) -self.p2*v*abs(v)
        
        return vstack((dp,dAngles,dv))
    
    

    def control(self, speed, heading, depth, euler_angles, auv_depth):
        """Method that returns the commands on the actuators, given the wanted speed, heading and depth."""

        ## Measures
        phi,theta,psi = euler_angles
        phi,theta,psi = sawtooth(phi), sawtooth(theta), sawtooth(psi)        
        z = auv_depth


        ## Controller parameters
        k_roll = 5
        k_pitch = 5
        k_yaw = 0.5


        ## SPEED CONTROL
        u0 = sqrt(self.p2/self.p1)*speed           # action on thruster


        ## ANGULAR CONTROL
        K = np.diag([k_roll,k_pitch,k_yaw])

        Ed = eulerderivative(phi,theta,psi)

        err = array([[sawtooth(0 - phi)],
                     [max(-0.1, min(-(depth-z)/5,0.1)) - theta],
                     [sawtooth(heading - psi)]])

        w_angles = matmul(matmul(inv(Ed),K), err)   # wanted rotational velocities

        sinU = (1/float(speed))**2 * matmul(inv(self.B), w_angles)
        sinU = np.clip(sinU, -1., 1.)

        u1 = arcsin(sinU[0,0])
        u2 = arcsin(sinU[1,0])
        u3 = arcsin(sinU[2,0])

        U = array([[u0,u1,u2,u3]]).T
        U[1:,:] = U[1:,:].clip(-0.35, 0.35)

        return U

    
###################################################################################






















