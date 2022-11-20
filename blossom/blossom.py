from fileinput import isfirstline
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.examples.blossom.set_env import SetEnv
from omni.isaac.examples.blossom.set_interface import SetInterface
from omni.isaac.examples.blossom.visualize_info import VisualizeInfo
from omni.isaac.examples.blossom.human_sync import Human, HumanSync
from omni.isaac.examples.blossom.sync_data import SyncData
from omni.isaac.core.objects import VisualCuboid
from pxr import UsdShade, Sdf, Gf
from itertools import cycle
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Usd

import matplotlib.pyplot as plt
import json
import random
import omni.kit.commands
import numpy as np

class Blossom(BaseSample):
    def __init__(self) -> None:
        self.colorchanged = False
        self.path_list=[21,22,23,24,64,65,66,67,68,69,41,42]
        self.pool = cycle(self.path_list)
        self.path_box_index = 0
        self.ispathhilighted = False
        self.isFirstTime = True
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.fire = {"511": False, "512": False, "529": False}
        super().__init__()
        return
    
    def setup_scene(self):
        world = self.get_world()
        # world.scene.add_default_ground_plane()
        setenv = SetEnv()
        setInterface = SetInterface()
        self.syncData = SyncData("")
        root_path = ''
        map_usd_path, map_prim_path ="", "/World/sejonguniversity_5f"
        scout_usd_path, scout_prim_path = "", "/World/scout_v2"
        octahedron_usd_path, octahedron_prim_path = "", "/World/highlight_"
        extinguisher_usd_path, extinguisher_prim_path = "", "/World/fire_extinguisher"

        omni.kit.commands.execute('CreateMdlMaterialPrimCommand',
            mtl_url='http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Glass/Green_Glass.mdl',
            mtl_name='Green_Glass',
            mtl_path='/World/Looks/Green_Glass')

        omni.kit.commands.execute('CreateMdlMaterialPrimCommand',
            mtl_url='http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Metals/Aluminum_Anodized_Blue.mdl',
            mtl_name='Aluminum_Anodized_Blue',
            mtl_path='/World/Looks/Aluminum_Anodized_Blue')

        humans = Human(root_path)
        human_list = humans.get_human_list()
        humans.align_human()
        self.humansync = HumanSync(human_list)
        setenv.load_asset(map_usd_path, map_prim_path, rotation=(0, 90, 90, 0), scale = (0.01, 0.01, 0.01))
        
        
        setInterface.add_viewport()
        
        setenv.load_asset(scout_usd_path, scout_prim_path, translation=(-12,-17,0.07), rotation =(0, 0, 0.70711, 0.70711))
        setenv.load_asset(octahedron_usd_path, scout_prim_path+'/scout_marker', translation=(0,0,5))
        setenv.load_asset(extinguisher_usd_path, extinguisher_prim_path, translation=(-13.3, -5.5, 0.04), scale = (0.002, 0.002, 0.002), rotation =(0.7071068, 0, 0, 0.7071068))
        omni.kit.commands.execute('BindMaterialCommand',
            prim_path='/World/scout_v2/scout_marker',
            material_path='/World/Looks/Green_Glass',
            strength='strongerThanDescendants')
        
        with open("", 'r') as f:
            self.data = json.load(f)
        for i, e in enumerate(self.data["boxlist"]):
            setenv.load_asset(octahedron_usd_path, octahedron_prim_path+"{}".format(i))
            omni.kit.commands.execute(
                "IsaacSimTeleportPrim",
                prim_path="/World/highlight_{}".format(i),
                rotation=(0,0,0,0),
                translation=(e[0], e[1], 3)
            )

        for e in self.path_list:
            setenv.load_asset(octahedron_usd_path, octahedron_prim_path+"{}".format(e+100))
            omni.kit.commands.execute(
                "IsaacSimTeleportPrim",
                prim_path="/World/highlight_{}".format(e+100),
                rotation=(0,0,0,0),
                translation=(self.data["boxlist"][e][0], self.data["boxlist"][e][1], -1)
            )
            omni.kit.commands.execute('BindMaterialCommand',
                prim_path='/World/highlight_{}'.format(e+100),
                material_path='/World/Looks/Aluminum_Anodized_Blue',
                strength='strongerThanDescendants')
        
        self.count = 0
        self.odd = True
        return
    
    async def setup_post_load(self):
        self.visualizeinfo = VisualizeInfo(self.data, self.path_box_index, self.path_list, self.pool)
        self.humansync.get_robot_heading()
        return
    
    async def _on_show_visualization_effect_async(self, val):
        if not val:
            omni.kit.commands.execute('ChangeSetting',
                path='/rtx/flow/enabled',
                value=True)

        else:
            omni.kit.commands.execute('ChangeSetting',
                path='/rtx/flow/enabled',
                value=False)
        return
    
    async def _on_show_summarize_data_async(self, val):
        if val:
            human_pos = self.syncData.get_human_count()
            self.visualizeinfo.drawPlot(human_pos)
        else:
            self.visualizeinfo.clear_plt()
        return
    
    async def _on_visualize_hazardness_async(self, val):
        if val:
            omni.kit.commands.execute('ToggleVisibilitySelectedPrims',
                selected_paths=[Sdf.Path('/World/sejonguniversity_5f/RectLight')])
            omni.kit.commands.execute('ToggleVisibilitySelectedPrims',
                selected_paths=[Sdf.Path('/World/sejonguniversity_5f/RectLight_01')])
        else:
            omni.kit.commands.execute('ToggleVisibilitySelectedPrims',
                selected_paths=[Sdf.Path('/World/sejonguniversity_5f/RectLight')])   
            omni.kit.commands.execute('ToggleVisibilitySelectedPrims',
                selected_paths=[Sdf.Path('/World/sejonguniversity_5f/RectLight_01')])
        return
    
    async def _on_show_optimized_path_highlight_async(self, val):
        if val:
            self.ispathhilighted = True
        else:
            self.ispathhilighted = False
    
    async def _on_sync_data_event_async(self, val):
        world = self.get_world()
        if val:
            await world.play_async()
            world.add_physics_callback("sim_step", self._on_follow_sync_data_simulation_step)
        else:
            world.remove_physics_callback("sim_step")
        return
    
    def _on_follow_sync_data_simulation_step(self, step_size):
        
        robot_info = self.syncData.robot_sync("")
        # self.humansync.get_pose_data(self.dc, robot_info)
        # self.syncData.check_human_data("")
        
        if self.ispathhilighted == True:
            if self.isFirstTime == True:
                self.visualizeinfo.turn_on_hilighted_path()
                self.isFirstTime = False
            self.visualizeinfo.let_marker_bounce(self.count, self.odd)
            
            if self.odd == True:
                self.count += 16/360
                if self.count>=1:
                    self.odd = False
            else:
                self.count -= 16/360
                if self.count<=0:
                    self.odd = True
        else:
            if self.isFirstTime == False:
                self.visualizeinfo.turn_off_hilighted_path()
                self.isFirstTime = True
                self.count = 0
        return
    
    async def setup_pre_reset(self):
        return
    
    async def setup_post_reset(self):
        return
    
    def world_cleanup(self):
        return




