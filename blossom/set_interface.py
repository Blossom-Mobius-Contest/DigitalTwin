from omni.isaac.core.utils.viewports import set_camera_view

import numpy as np
import omni

class SetInterface:
    def add_viewport(self):
    #     try:
    #         viewport_interface = omni.kit.viewport_get_viewport_interface()
    #     except Exception as e:
    #         viewport_interface = omni.kit.viewport_legacy.get_viewport_interface()
    #     viewport_handle_2 = viewport_interface.get_instance('Viewport_2')
    #     viewport_window_2 = viewport_interface.get_viewport_window(viewport_handle_2)
    #     viewport_window_2.set_active_camera("/World/scout_v2/base_link/Camera")
    #     viewport_window_2.set_texture_resolution(2560, 720)
    #     viewport_window_2.set_window_pos(0, 0)
    #     viewport_window_2.set_window_size(2560, 720)    
        
        set_camera_view(eye=np.array([-18, -8, 4]), target=np.array([-4, 2, 0]))

        return