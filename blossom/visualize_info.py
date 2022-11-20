from omni.isaac.core.utils.rotations import euler_angles_to_quat
from itertools import cycle
from pxr import Sdf
import omni
import numpy as np
import matplotlib.pyplot as plt

class VisualizeInfo():
    def __init__(self, data, path_box_index, path_list, pool):
        self.data = data
        self.path_box_index = path_box_index
        self.path_list = path_list
        self.pool = pool
        omni.kit.commands.execute('ChangeSetting',
        path='rtx/flow/enabled',
        value=False)
        omni.kit.commands.execute('ChangeSetting',
        path='rtx/flow/rayTracedReflectionsEnabled',
        value=True)
        omni.kit.commands.execute('ChangeSetting',
        path='rtx/flow/rayTracedTranslucencyEnabled',
        value=True)
        omni.kit.commands.execute('ChangeSetting',
        path='rtx/flow/pathTracingEnabled',
        value=True)
        omni.kit.commands.execute('ChangeSetting',
            path='/rtx/flow/maxBlocks',
            value=2048,
            prev=0)
        
    def turn_on_hilighted_path(self):
        self.data["boxlist"][self.path_box_index].append(-1)
        for e in self.path_list:
            omni.kit.commands.execute(
            "IsaacSimTeleportPrim",
            prim_path="/World/highlight_{}".format(e),
            rotation=(0,0,0,0),
            translation=self.data["boxlist"][self.path_box_index]
            )
        self.data["boxlist"][self.path_box_index].pop(-1)
        
        self.data["boxlist"][self.path_box_index].append(3)
        for e in self.path_list:
            omni.kit.commands.execute(
            "IsaacSimTeleportPrim",
            prim_path="/World/highlight_{}".format(e+100),
            rotation=(0,0,0,0),
            translation=self.data["boxlist"][self.path_box_index]
            )
        omni.kit.commands.execute('ToggleVisibilitySelectedPrims',
                selected_paths=[Sdf.Path('/World/sejonguniversity_5f/lights/RectLight_21')])
        omni.kit.commands.execute('ToggleVisibilitySelectedPrims',
                selected_paths=[Sdf.Path('/World/sejonguniversity_5f/lights/RectLight_22')])
        omni.kit.commands.execute('ToggleVisibilitySelectedPrims',
                selected_paths=[Sdf.Path('/World/sejonguniversity_5f/lights/RectLight_23')])
        self.data["boxlist"][self.path_box_index].pop(-1)
        
    def let_marker_bounce(self, count, odd):
        
        if (count == 0) | (count<= 0):
            self.path_box_index = next(self.pool)
        
        rotation = euler_angles_to_quat(np.array([1, 0, 0])*np.array([count]))
        self.data["boxlist"][self.path_box_index].append(1.5*count+3)
        omni.kit.commands.execute(
                "IsaacSimTeleportPrim",
                prim_path="/World/highlight_{}".format(self.path_box_index+100),
                rotation=rotation,
                translation=self.data["boxlist"][self.path_box_index]
        )
        self.data["boxlist"][self.path_box_index].pop(-1)

    def turn_off_hilighted_path(self):
        for e in self.path_list:
            self.data["boxlist"][e].append(3)
            omni.kit.commands.execute(
            "IsaacSimTeleportPrim",
            prim_path="/World/highlight_{}".format(e),
            rotation=(0,0,0,0),
            translation=self.data["boxlist"][e]
            )
            self.data["boxlist"][e].pop(-1)
        
        for e in self.path_list:
            self.data["boxlist"][e].append(-1)
            omni.kit.commands.execute(
            "IsaacSimTeleportPrim",
            prim_path="/World/highlight_{}".format(e+100),
            rotation=(0,0,0,0),
            translation=self.data["boxlist"][e]
            )
            self.data["boxlist"][e].pop(-1)
            self.pool = cycle(self.path_list)
    def drawPlot(self, human_pos):
        plt.style.use(['dark_background'])

        threshold = np.ones(100)*40

        fig = plt.figure(figsize=(14,12))

        xData = range(100)
        yData = list()
        yData.append(np.array(range(100))*np.array(range(100)) *1/100 + np.random.rand(100)*10 + np.sin(range(100))*5)
        yData.append(np.log10(np.array(range(100))+1)*10 + np.random.rand(100)*3 + 4)
        yData.append(np.array(range(100))*np.array(range(100)) *1/1000 + np.random.rand(100)*10 + np.sin(range(100))*5)
        yData.append(np.array(np.random.rand(100)))

        title_name = ["Room Number 529, toxic gas concentration", "Room Number 529, Scale of Fire", "Room Number 512, toxic gas concentration", "Room Number 512, Scale of Fire"]
        life_time = ["life time: 2m37s left", "life time: 7m44s left"]
        for i in range(2):
            plt.subplot(3, 2, i*2+1)
            plt.ylim(0, 100)
            plt.plot(xData, yData[i*2], threshold)

            NbData = len(xData)
            MaxBL = [[MaxBL] * NbData for MaxBL in range(100)]
            Max = [np.asarray(MaxBL[x]) for x in range(100)]

            for x in range (0, 40):
                plt.fill_between(xData, yData[i*2], Max[x], where=yData[i*2] <Max[x], facecolor='green', alpha=0.08)

            for x in range (40, 100):
                plt.fill_between(xData, Max[x], yData[i*2], where=yData[i*2] >Max[x], facecolor='red', alpha=0.08)

            plt.fill_between([], [], [], facecolor='green', label="safe")
            plt.fill_between([], [], [], facecolor='red', label="dangerous")
            plt.text(1.0,90, life_time[i])
            plt.title(title_name[2*i])    
            plt.legend(loc=4, fontsize=12)

            plt.subplot(3, 2, i*2+2)
            plt.ylim(0, 30)
            plt.plot(xData, yData[2*i+1])

            for x in range (0, 100):
                plt.fill_between(xData, Max[x], yData[2*i+1], where=yData[2*i+1] >Max[x], facecolor='red', alpha=0.05)
                
            plt.title(title_name[2*i+1])
            plt.legend(loc=4, fontsize=12)
        
        plt.subplot(3,2,5)
        index = np.arange(3)
        plt.bar(index,list(human_pos.values()))
        plt.title('people in sections', fontsize=20)
        plt.xlabel('section', fontsize=18)
        plt.ylabel('count', fontsize=18)
        plt.xticks(index, list(human_pos.values()), fontsize=15)
        plt.show()
    def clear_plt(self):
        plt.clf()