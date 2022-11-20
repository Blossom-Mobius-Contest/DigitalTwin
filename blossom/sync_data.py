from omni.isaac.core.utils.rotations import euler_angles_to_quat

import requests
import omni
import json
import copy

class SyncData:
    def __init__(self, url) -> None:
        self.url = url
        self.la = None
        self.payload = {}
        self.headers = {
            'Accept': 'application/json',
            'X-M2M-RI': '12345',
            'X-M2M-Origin': 'SOrigin'
        }
        self.room = ["room511", "hall", "room529"]
        self.sensor = ["flameSensor", "gasSensor"]
        self.human_count = {
            "section1": 3,
            "section2": 0,
            "section3": 0,
            }
        self.robot_info=[None, None]
        with open("", 'r') as f:
            self.vl_list = json.load(f)
            
    def check_room_data(self):
        data = list()
        for r in self.room:
            for s in self.sensor:
                url = self.url + "{}/{}la".format(s,r)
                response = requests.request('GET', url, headers = self.headers, data = self.payload)
                data.append(response.json()["m2m:cin"]["con"])
        return data
    
    def robot_sync(self, url):
        response = requests.request('GET', url, headers = self.headers, data = self.payload)
        res = int(response.json()["m2m:cin"]["con"]['location'])
        if res == self.la:
            return self.robot_info
        else:
            self.robot_info[0] = self.vl_list['index'][res]
            # if (0 <= res <= 4) | (res == 9):
            if (0 <= res <= 10):
                rotation = (0, 0, 0.70711, 0.70711)
            else: rotation = (0, 0, 0, 1)
            self.robot_info[1] = rotation
            omni.kit.commands.execute(
                'IsaacSimTeleportPrim',
                prim_path="/World/scout_v2",
                translation=tuple(self.vl_list['index'][res]),
                rotation=rotation,
            )
            translation = copy.deepcopy(self.vl_list['index'][res])
            translation[2] += 4
            omni.kit.commands.execute(
                'IsaacSimTeleportPrim',
                prim_path="/World/scout_v2/scout_marker",
                translation=tuple(translation),
                rotation=rotation,
            )
            self.la = res
        return self.robot_info
    
    def check_human_data(self, url):
        response = requests.request('GET', url, headers = self.headers, data = self.payload)
        res = response.json()["m2m:cin"]["con"]
        self.human_count[res['section']] = res['count']
    
    def get_human_count(self):
        return self.human_count