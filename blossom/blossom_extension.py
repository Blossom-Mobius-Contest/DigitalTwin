from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.blossom import Blossom

from omni.isaac.ui.ui_utils import state_btn_builder

import os
import asyncio
import omni.ui as ui

class BlossomExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="Blossom",
            title="Blossom",
            doc_link="",
            overview="This Example shows how to synchronize dofbot from physics to Isaac Sim",
            sample=Blossom(),
            file_path=os.path.abspath(__file__),
            number_of_extra_frames=4,
        )
        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_data_sync_ui(frame)
        frame = self.get_frame(index=1)
        self.build_visulization_mode_ui(frame)
        frame = self.get_frame(index=2)
        self.build_highlight_data_ui(frame)
        frame = self.get_frame(index=3)
        self.build_summarize_data_ui(frame)
        return

    def _on_sync_data_button_event(self, val):
        asyncio.ensure_future(self.sample._on_sync_data_event_async(val))
        return

    def _on_show_visualization_effect_button_event(self, val):
        asyncio.ensure_future(self.sample._on_show_visualization_effect_async(val))
        return

    def _on_show_optimized_path_highlight_button_event(self, val):
        asyncio.ensure_future(self.sample._on_show_optimized_path_highlight_async(val))
        return

    def _on_visualize_hazardness_button_event(self, val):
        asyncio.ensure_future(self.sample._on_visualize_hazardness_async(val))
        return
    
    def _on_show_summarize_data_button_event(self, val):
        asyncio.ensure_future(self.sample._on_show_summarize_data_async(val))
        return

    def post_load_button_event(self):
        self.task_ui_elements["VisualizationEffect"].enabled = True
        self.task_ui_elements["HighlightHazardnessAndShelter"].enabled = True
        self.task_ui_elements["highlightedShelter"].enabled = True
        self.task_ui_elements["SyncData"].enabled = True
        self.task_ui_elements["summarize"].enabled = True
        return

    def post_clear_button_event(self):
        return

    def build_data_sync_ui(self, frame):
            with frame:
                with ui.VStack(spacing=5):
                    frame.title = "Connect to Mobius Server"
                    frame.visible = True

                    dict = {
                        "label": "Sync to real World",
                        "type": "button",
                        "a_text": "START",
                        "b_text": "STOP",
                        "tooltip": "Connection to mobius server",
                        "on_clicked_fn": self._on_sync_data_button_event,
                    }
                    self.task_ui_elements["SyncData"] = state_btn_builder(**dict)
                    self.task_ui_elements["SyncData"].enabled = False

    def build_visulization_mode_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                frame.title = "Turn Visualization Effect On"
                frame.visible = True

                dict = {
                    "label": "Turn fire, smoke effect on",
                    "type": "button",
                    "a_text": "STOP",
                    "b_text": "START",
                    "tooltip": "It may slow your computer",
                    "on_clicked_fn": self._on_show_visualization_effect_button_event,
                }
                self.task_ui_elements["VisualizationEffect"] = state_btn_builder(**dict)
                self.task_ui_elements["VisualizationEffect"].enabled = False

    def build_highlight_data_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                frame.title = "Highlight Target"
                frame.visible = True
                dict = {
                    "label": "show Hazardness & Shelter",
                    "type": "button",
                    "a_text": "START",
                    "b_text": "STOP",
                    "tooltip": "cube color indicates hazardness",
                    "on_clicked_fn": self._on_visualize_hazardness_button_event,
                }
                self.task_ui_elements["HighlightHazardnessAndShelter"] = state_btn_builder(**dict)
                self.task_ui_elements["HighlightHazardnessAndShelter"].enabled = False

                dict = {
                    "label": "show optimized path",
                    "type": "button",
                    "a_text": "START",
                    "b_text": "STOP",
                    "tooltip": "This shows optimized path, shelter",
                    "on_clicked_fn": self._on_show_optimized_path_highlight_button_event,
                }
                self.task_ui_elements["highlightedShelter"] = state_btn_builder(**dict)
                self.task_ui_elements["highlightedShelter"].enabled = False
                
    def build_summarize_data_ui(self, frame):
            with frame:
                with ui.VStack(spacing=5):
                    frame.title = "summarize data"
                    frame.visible = True

                    dict = {
                        "label": "show summarized data",
                        "type": "button",
                        "a_text": "START",
                        "b_text": "STOP",
                        "tooltip": "summarize data of building",
                        "on_clicked_fn": self._on_show_summarize_data_button_event,
                    }
                    self.task_ui_elements["summarize"] = state_btn_builder(**dict)
                    self.task_ui_elements["summarize"].enabled = False
            return