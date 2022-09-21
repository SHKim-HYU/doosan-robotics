#!/usr/bin/env python3
import numpy as np

class Trajectory:
    def __init__(self, n_Joint):
        self.m_cof = np.zeros((6,6))
        self.TrajDuration = np.zeros(n_Joint)
        self.TrajInitTime = np.zeros(n_Joint)
        self.TrajTime = np.zeros(n_Joint)
        self.StateVec = np.zeros((n_Joint,6))
        self.Coefficient = np.zeros((n_Joint,6))
        self.m_isReady = np.zeros(n_Joint)
        
    def SetPolynomial5th(self, NumJoint, act, FinalPos, InitTime, Duration):
        self.TrajDuration[NumJoint] = Duration
        self.TrajInitTime[NumJoint] = InitTime
        
        self.m_cof = np.array([[1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,2,0,0,0],
            [1,pow(self.TrajDuration[NumJoint],1),pow(self.TrajDuration[NumJoint],2),pow(self.TrajDuration[NumJoint],3),pow(self.TrajDuration[NumJoint],4),pow(self.TrajDuration[NumJoint],5)],
            [0,1,2*pow(self.TrajDuration[NumJoint],1),3*pow(self.TrajDuration[NumJoint],2),4*pow(self.TrajDuration[NumJoint],3),5*pow(self.TrajDuration[NumJoint],4)],
            [0,0,2,6*pow(self.TrajDuration[NumJoint],1),12*pow(self.TrajDuration[NumJoint],2),20*pow(self.TrajDuration[NumJoint],3)]])
        self.StateVec[NumJoint] = [act,0,0,FinalPos,0,0]
        self.Coefficient[NumJoint] = np.linalg.inv(self.m_cof)@self.StateVec[NumJoint]
        self.m_isReady[NumJoint]=1
        
    
    def Polynomial5th(self,NumJoint, CurrentTime):
        if((CurrentTime - self.TrajInitTime[NumJoint])>= self.TrajDuration[NumJoint]):
            self.m_isReady[NumJoint] = 0
            flag = 0
            return self.StateVec[NumJoint,3:6],flag
        
        if(self.m_isReady[NumJoint]):
            dq=0; dq_dot=0; dq_ddot=0;
            flag = 1
            self.TrajTime[NumJoint] = CurrentTime - self.TrajInitTime[NumJoint]
            for i in range(0,6):
                dq += pow(self.TrajTime[NumJoint],i)*self.Coefficient[NumJoint,i]
                if i>=1:
                    dq_dot += i*pow(self.TrajTime[NumJoint],i-1)*self.Coefficient[NumJoint,i]
                if i>=2:
                    dq_ddot += i*(i-1)*pow(self.TrajTime[NumJoint],i-2)*self.Coefficient[NumJoint,i]
                     
            return np.array([dq,dq_dot,dq_ddot]),flag
            
            
        else:
            return
            
if __name__ == '__main__':
    a=Trajectory(6)
    act=[0,0,0,0,0,0]
    pos_job=[1,2,3,4,5,6]
    a.SetPolynomial5th(0,act[0],pos_job[0],0,2)
    a.Polynomial(0,0.5)
