from omni.isaac.urdf import _urdf
import omni.kit.commands

class SetEnv:
    def load_asset(self, usd_path, prim_path, translation=(0,0,0), rotation=(0,0,0,0), scale = (1,1,1)):
        omni.kit.commands.execute(
            "IsaacSimSpawnPrim",
            usd_path=usd_path,
            prim_path=prim_path,
            translation=translation,
            rotation=rotation,
        )
        omni.kit.commands.execute(
            "IsaacSimScalePrim",
            prim_path=prim_path,
            scale=scale,
        )

