'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        if effector_name not in ['LLeg', 'RLeg']:
            return joint_angles

        is_left_leg = (effector_name == 'LLeg')

        x,y,z = transform[0,3], transform[1,3], transform[2,3]

        sign = 1.0 if is_left_leg else -1.0
        L_upper= 0.100
        L_lower = 0.103

        distance = (x**2+ y**2 + z**2) **05
        try:  
        hip_pitch = -atan2(x,-z)
        cos_knee = (L_upper**2 + L_lower**2 - distance**2)/ (2*L_upper * L_lower)
        knee_pitch = acos(max(min(cos_knee ,1),-1)
        ankle_pitch = -hip_pitch -knee_pitch

        hip_roll = atan2(y, -z)
        ankle_roll = -hip_roll

        if effector_name == 'LLeg':
            joint_angles = [0.0, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll]
        else: 
            joint_angles = [0.0, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll]

        except: joint_angles = [0.0, 0.0, -0.1, 0.7, -0.5, 0.0]
        
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        joint_angles = self.inverse_kinematics(effector_name, transform)

        if effector_name == 'LLeg':
            joint_names = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll']
        else:
            joint_names = ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']       

        times = []
        keys = []

        for angle in joint_angles:
            times.append([2.0])
            keys.append([[angle, [3, 0, 0], [3,0 ,0]]])
            
        self.keyframes = (joint_names, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
