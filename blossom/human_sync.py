from omni.isaac.urdf import _urdf
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.prims import GeometryPrim
import numpy as np
import omni
import requests
import json
import math

from omni.isaac.examples.blossom.set_env import SetEnv

class Human():
    def __init__(self, prim_path, translation=(0,0,0.5), rotation=(0,0,0,0)):
        self.human_dict = {}
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = True
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.create_physics_scene = True
        import_config.import_inertia_tensor = False
        import_config.default_drive_strength = 1047.19751
        import_config.default_position_drive_damping = 52.35988
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        import_config.distance_scale = 1
        import_config.density = 0.0
        
        for i in range(4):
            human_prim_path = omni.kit.commands.execute( 
            'URDFParseAndImportFile', 
            urdf_path=prim_path, 
            import_config=import_config,)
            self.human_dict[human_prim_path[1]] = None

            
    def get_human_list(self):
        return self.human_dict
    
    def align_human(self):
        for prim_path in self.human_dict.keys():
            # print(self.human_dict.keys())
            omni.kit.commands.execute(
            'IsaacSimTeleportPrim',
            prim_path=prim_path,
            translation=tuple([30 + 2 * np.random.rand(), 30 + 2*np.random.rand(), -3]),
            rotation=(0, 0, 0, 1),
            )

class HumanSync():
    
    RShoulder_pos = 12
    RWrist_pos = 14
    LShoulder_pos = 5
    LWrist_pos = 7
    Neck_pos = 3
    RElbow_pos = 13
    LElbow_pos = 6
    RHip_pos = 22
    RKnee_pos = 23
    RAnkle_pos = 24
    LHip_pos = 18
    LKnee_pos = 19
    LAnkle_pos = 20

    def make_vec(self, arr1, arr2):
        return np.array(arr2) - np.array(arr1)
    def get_angle(self, arr1, arr2):
        dot_product = np.dot(arr1/np.linalg.norm(arr1), arr2/np.linalg.norm(arr2))
        return np.arccos(dot_product)
    def proj(self, arr1, arr2, arr3):
        proj_arr2 = arr2
        proj_arr3 = arr3
        proj_arr2[1] = arr1[1]
        proj_arr3[1] = arr3[1]
        return proj_arr2, proj_arr3
    def proj_roll(self, arr1, arr2, arr3):
        proj_arr1 = [1,0,0]
        proj_arr3 = arr3
        proj_arr1[2] = arr2[2]
        proj_arr3[2] = arr2[2]
        return proj_arr1, proj_arr3
    def get_angle(self, arr1, arr2):
        dot_product = np.dot(arr1/np.linalg.norm(arr1), arr2/np.linalg.norm(arr2))
        return np.arccos(dot_product)
    def get_body_angle(self, arr1, arr2, arr3):
        proj_arr2, proj_arr3 = self.proj(arr1, arr2, arr3)
        vec1 = self.make_vec(arr1, proj_arr2)
        vec2 = self.make_vec(proj_arr2, proj_arr3)
        angle = self.get_angle(vec1, vec2)
        return angle
    def get_body_roll_angle(self, arr1, arr2, arr3):
        arr1, proj_arr3 = self.proj_roll(arr1, arr2, arr3)
        vec1 = [1,0]
        vec1.append(arr2[2])
        vec2 = self.make_vec(proj_arr3, arr2)
        vec2[2] = arr2[2]
        angle = self.get_angle(self, vec1, vec2)
        return angle
    
    def __init__(self, human_list):
        self.url = ''
        self.payload={}
        self.headers = {
            'Accept': 'application/json',
            'X-M2M-RI': '12345',
            'X-M2M-Origin': 'SOrigin'
        }
        self.human_list = human_list
        # self.joints = ['LElbowYaw','RElbowYaw','RElbowRoll', 'RHipPitch', 'RShoulderPitch', 'RKneePitch', 'LElbowRoll', 'LHipPitch', 'LShoulderPitch', 'RShoulderRoll', 'LShoulderRoll', 'LKneePitch', 'RHipRoll', 'LHipRoll']
        self.joints = ['LElbowYaw','RElbowYaw','RElbowRoll', 'RHipPitch', 'RShoulderPitch', 'RKneePitch', 'LElbowRoll', 'LHipPitch', 'LShoulderPitch', 'LKneePitch']
        
    def heading(self,left_shoulder, right_shoulder, theta):
        lx = np.cos(theta) * left_shoulder[0] - np.sin(theta) * left_shoulder[1]
        ly = np.cos(theta) * left_shoulder[1] + np.sin(theta) * left_shoulder[0]
        l = [lx, ly, left_shoulder[2]]
        rx = np.cos(theta) * right_shoulder[0] - np.sin(theta) * right_shoulder[1]
        ry = np.cos(theta) * right_shoulder[1] + np.sin(theta) * right_shoulder[0]
        r = [rx, ry, right_shoulder[2]]
        vec1 = self.make_vec(l, r)
        init_vec = [0, 1, 0]
        angle = self.get_angle(init_vec, vec1)
        return angle
                
    def get_pose_data(self, dc, robot_info):
        response = requests.request('GET', self.url, headers = self.headers, data=self.payload)
        res=response.json()['m2m:cin']['con']['result']
        data = res
        number_of_human = data.__len__()
        
        #여기는 id 조절(증감 반영)
        sim_label_id_list = list(self.human_list.values())
        sim_label_items_list = list(self.human_list.items())
        zed_label_id_list = [e['label_id'] for e in data]
        
        if number_of_human != (4 - sim_label_id_list.count(None)):
            #카메라에서 사람이 나간 경우
            for e in sim_label_id_list:
                if e not in zed_label_id_list:
                    key = [k for k, v in self.human_list.items() if v == e]
                    omni.kit.commands.execute(
                        'IsaacSimTeleportPrim',
                        prim_path=key[0],
                        translation=tuple([30 + 2 * np.random.rand(), 30 + 2*np.random.rand(), -3]),
                        rotation=(0, 0, 0, 1),
                    )
                    self.human_list[key[0]] = None
                    break
            #카메라에 사람이 들어온 경우
            for e in zed_label_id_list:
                if e not in sim_label_id_list:
                    for i in sim_label_items_list:
                        if i[1] == None:
                            self.human_list[i[0]] = e
                            break
        # 식 계산
        sim_label_items_list = self.human_list.items()
        print(sim_label_items_list)
        for e in sim_label_items_list:
            if e[1] != None:
                for i in data:
                    if e[1] == i['label_id']:
                        bodypoint_list = i['skeleton'].replace("'", '"')
                        joint_list = json.loads(bodypoint_list)
                        RArmAngle = self.get_body_angle(joint_list[self.RShoulder_pos], joint_list[self.RElbow_pos], joint_list[self.RWrist_pos])
                        RPelvisAngle = self.get_body_angle(joint_list[self.RShoulder_pos], joint_list[self.RHip_pos], joint_list[self.RKnee_pos])
                        RShoulderAngle = self.get_body_angle(joint_list[self.RHip_pos], joint_list[self.RShoulder_pos], joint_list[self.RElbow_pos])
                        RKneeAngle = self.get_body_angle(joint_list[self.RHip_pos], joint_list[self.RKnee_pos], joint_list[self.RAnkle_pos])
                        LArmAngle = self.get_body_angle(joint_list[self.LShoulder_pos], joint_list[self.LElbow_pos], joint_list[self.LWrist_pos])
                        LPelvisAngle = self.get_body_angle(joint_list[self.LShoulder_pos], joint_list[self.LHip_pos], joint_list[self.LKnee_pos])
                        LShoulderAngle = self.get_body_angle(joint_list[self.LHip_pos], joint_list[self.LShoulder_pos], joint_list[self.LElbow_pos])
                        LKneeAngle = self.get_body_angle(joint_list[self.LHip_pos], joint_list[self.LKnee_pos], joint_list[self.LAnkle_pos])
                        # RShoulderRollAngle = self.get_body_roll_angle(joint_list[self.RHip_pos], joint_list[self.RShoulder_pos], joint_list[self.RElbow_pos])
                        # LShoulderRollAngle = self.get_body_roll_angle(joint_list[self.LHip_pos], joint_list[self.LShoulder_pos], joint_list[self.LElbow_pos])
                        # RPelvisRollAngle = self.get_body_roll_angle(joint_list[self.RShoulder_pos], joint_list[self.RHip_pos], joint_list[self.RKnee_pos])
                        # LPelvisRollAngle = self.get_body_roll_angle(joint_list[self.LShoulder_pos], joint_list[self.LHip_pos], joint_list[self.LKnee_pos])
                        # angle_list = [-1.5,1.5,RArmAngle, -RPelvisAngle-0.2, RShoulderAngle - 1.5, RKneeAngle, (-1)*LArmAngle, -LPelvisAngle-0.2, LShoulderAngle - 1.5, LKneeAngle, RShoulderRollAngle, LShoulderRollAngle, RPelvisRollAngle, LPelvisRollAngle]
                        angle_list = [-1.5, 1.5, RArmAngle, -RPelvisAngle-0.2, RShoulderAngle-1.5, RKneeAngle, (-1)*LArmAngle, -LPelvisAngle-0.2, LShoulderAngle - 1.5, LKneeAngle]
                        for j in angle_list:
                            if j>2:
                                j = j-np.pi
                            elif j<-2:
                                j += np.pi
                        # heading_quat = euler_angles_to_quat(np.array(eval(i['skeleton_head'].replace("'", '"'))))
                        
                        human_pos = eval(i['position'].replace("'", '"'))
                        theta = 0
                        if robot_info[1][3] == 1:
                            theta = 0
                        else: theta = np.pi/2
                        # robot = GeometryPrim(prim_path='/World/scout_v2')
                        heading_data = self.heading(joint_list[self.LShoulder_pos], joint_list[self.RShoulder_pos], theta)
                        heading_quat = euler_angles_to_quat(np.array([heading_data, 3.14, 0]))
                        x = (human_pos[0] * np.cos(theta) - human_pos[1] * np.sin(theta)) + robot_info[0][0]
                        y = (human_pos[1] * np.cos(theta) + human_pos[0] * np.sin(theta)) + robot_info[0][1]
                        translation = [x, y, human_pos[2] + robot_info[0][2]]
                        translation[2] += 0.4
                        # translation = np.array(translation, dtype='float64') + np.array((robot.get_world_pose()[0]))
                        
                        omni.kit.commands.execute(
                            'IsaacSimTeleportPrim',
                            prim_path=e[0],
                            translation=translation,
                            rotation=heading_quat
                        )
                        art = dc.get_articulation(e[0])
                        for item1, item2 in zip(self.joints, angle_list):
                            dc.set_dof_position(dc.find_articulation_dof(art, item1), item2)
                        break
    def get_robot_heading(self):
        point1 = GeometryPrim(prim_path='/World/scout_v2/front_left_wheel_link')
        point2 = GeometryPrim(prim_path='/World/scout_v2/rear_left_wheel_link')
        return np.array(point1.get_world_pose()[0]) - np.array(point2.get_world_pose()[0])
        
    def get_theta(self, direction_vector):
        x_unit = np.array([1, 0])
        robot_vec = direction_vector[0:2]
        return np.arctan(robot_vec[1]-x_unit[1]/robot_vec[0]-x_unit[0])