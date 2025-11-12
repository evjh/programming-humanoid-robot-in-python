'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        if self.start_time is None:
            self.start_time = perception.time
        
        names, times, keys = keyframes

        if not names: 
            return target_joint

        current_time = perception.time - self.start_time

        for i , joint_nme in enumerate(names):
            if joint_name not in self.joint_names:
                continue

        joint_times = times[i]
        joint_keys = keys[i]

        segment_found = False
        for j in range(len(joint_times)-1):
            if joint_times[j] <= current_time <= joint_times[j+1]:

                t0 = joint_times[j]
                t1 = joint_times[j+1]
                t= (current_time -t0)/(t1-t0)
                p0 = joint_keys[j][0]
                p3 = joint_keys[j+1][0]
                p1 = P0+ joint_keys[j][2][2]
                2= P3 + joint_keys[j+1][1][2]
                
                target_joints[joint_name] = ((1-t)**3)*p0 + 3*t*((1-t)**2)*p1+ 3*(t**2)*(1-t)*p2 +(t**3)*p3
                break
                
            else: 
            if current_time < current_times[0]:
                target_joint[joint_name] = joint_keys[0][0]
            else: target_joints[joint_name] = joint_keys[-1][0]

        if 'LHipYawPitch' in target_joints:
            target_joints ['RHipYawPitch'] = target_joints['LHipYawPitch']
        

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
