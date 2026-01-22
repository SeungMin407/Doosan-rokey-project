from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import random
import sys
import carb
import omni
import omni.usd
from isaacsim.core.api.world import World
from isaacsim.core.utils.nucleus import get_assets_root_path
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema, UsdShade, Sdf, Usd
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.api.objects import DynamicCuboid, DynamicSphere
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.prims import SingleArticulation
import isaacsim.robot_motion.motion_generation as mg
from scipy.spatial.transform import Rotation as R    
from isaacsim.core.prims import RigidPrim
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.sensor import Camera
# ==============================
# 설정 값
# ==============================
APPLE_USD_PATH = "/home/rokey/Desktop/cube.usd"
TEXTURE_PNG = "/home/rokey/Desktop/Scratches/apple_file/apple/apple0.png"
TOTAL_APPLES = 15

CENTER_X, CENTER_Y = -2.5, 0.0
SPREAD_XY = 0.18
START_Z = 2
HEIGHT_JITTER = 0.4

APPLE_RADIUS = 0.05

def create_omni_pbr_material_with_texture(stage, material_path, texture_path):
    material = UsdShade.Material.Define(stage, material_path)

    shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
    shader.CreateIdAttr("OmniPBR")

    shader.CreateInput(
        "diffuse_texture", Sdf.ValueTypeNames.Asset
    ).Set(texture_path)

    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )

    return material

def bind_material_to_apple_root(stage, apple_prim_path, material_path):
    apple_prim = stage.GetPrimAtPath(apple_prim_path)
    material = UsdShade.Material.Get(stage, material_path)

    if not apple_prim.IsValid():
        print("Invalid apple prim:", apple_prim_path)
        return

    UsdShade.MaterialBindingAPI(apple_prim).Bind(material)


# ---------------- RMPFlow Controller ----------------
class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name, robot_articulation, physics_dt=1/60, attach_gripper=True):
        cfg = mg.interface_config_loader.load_supported_motion_policy_config(
            "UR10", "RMPflowSuction" if attach_gripper else "RMPflow"
        )
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**cfg)
        self.articulation_rmp = mg.ArticulationMotionPolicy(
            robot_articulation, self.rmp_flow, physics_dt
        )
        super().__init__(name=name, articulation_motion_policy=self.articulation_rmp)

        self._default_position, self._default_orientation = \
            robot_articulation.get_world_pose()

        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation
        )

    def reset(self):
        super().reset()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation
        )

# ==============================
# World / Stage
# ==============================
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World/AppleDrop")
# background
add_reference_to_stage(
    usd_path="/home/rokey/Desktop/cooperate_3/detect_classification/map/conveyor.usd",
    prim_path="/World/Background"
)
stage = get_current_stage()

# map.usd에서 카메라 prim 가져오기
camera_prim = stage.GetPrimAtPath("/World/TopViewCamera")  # map.usd에 정의된 카메라 경로
rgb_camera = Camera(prim_path="/World/TopViewCamera")

# 사과 생성 
# ==============================
for i in range(13) :
    TEXTURE_PNG = f"/home/rokey/Desktop/Scratches/apple_file/apple/apple{i}.png"

    apple_material = create_omni_pbr_material_with_texture(
        stage,
        f"/World/Materials/ApplePBR{i}",
        TEXTURE_PNG
    )
apple_rating={"A":[0],"B":[1,2,3,4,5,6,7,8],"C":[9,10,11,12]}

apples = []
select_rail=1
rail1=[]
for i in range(TOTAL_APPLES):
    good_apple=random.randint(0,12)
    phys_path = f"/World/AppleDrop/Apple_{i:03d}"
    visual_path = phys_path + "/Visual"

    # Physics
    apple = world.scene.add(
        DynamicSphere(
            prim_path=phys_path,
            name=f"apple_{i}",
            position=np.array([100.0, 0.0, -100.0]),
            radius=APPLE_RADIUS,
        )
    )

    # Visual USD
    add_reference_to_stage(
        usd_path=APPLE_USD_PATH,
        prim_path=visual_path
    )

    # Apple_{i}에 직접 Bind
    bind_material_to_apple_root(
        stage,
        phys_path,
        f"/World/Materials/ApplePBR{good_apple}"
    )

    apples.append([apple,select_rail])
    
    visual_prim = stage.GetPrimAtPath(visual_path)
    xformable = UsdGeom.Xformable(visual_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, -0.05))
    if select_rail==1:
        rail1.append(apple)
    select_rail+=1
    if select_rail>3:
        select_rail=1
# assets
assets_root = get_assets_root_path()
if assets_root is None:
    carb.log_error("Isaac assets not found")
    simulation_app.close()
    sys.exit()
# robot
ur10_path = assets_root + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
robot_prim = add_reference_to_stage(ur10_path, "/World/UR10")
robot_prim.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")

gripper = SurfaceGripper(
    end_effector_prim_path="/World/UR10/ee_link",
    surface_gripper_path="/World/UR10/ee_link/SurfaceGripper"
)

ur10 = world.scene.add(
    SingleManipulator(
        prim_path="/World/UR10",
        name="ur10",
        end_effector_prim_path="/World/UR10/ee_link",
        gripper=gripper
    )
)
ur10.set_joints_default_state(
    positions=np.array([-np.pi/2]*4 + [np.pi/2, 0])
)

world.reset()


# ==============================
# 시뮬레이션 루프
# ==============================
i = 0
drop_delay = 0
task_phase=0
while simulation_app.is_running():
    world.step(render=True)
    # set apple_physics
    # ==============================
    for apple in apples:
        pos, _ = apple[0].get_world_pose()
        if pos[2] < 1.2 and pos[2]>0.5:
            vel = apple[0].get_linear_velocity()
            if apple[1]==1:
                new_vel = np.array([1.0, 0.3*vel[1], vel[2]])
            elif apple[1]==2:
                new_vel = np.array([1.0*0.5, 0.3*vel[1]+1.0*0.33, vel[2]])
            elif apple[1]==3:
                new_vel = np.array([1.0*0.5, 0.3*vel[1]-1.0*0.33, vel[2]])
            apple[0].set_linear_velocity(new_vel)
            
    if task_phase==0:
        ur10.set_world_pose(position=np.array([0.0, 0.6, 0.0]))
        task_phase+=1
        
    elif task_phase==1:
        # apple drop
        drop_delay += 1
        if drop_delay < 50:
            continue

        drop_delay = 0

        if i >= TOTAL_APPLES:
            task_phase+=1
            continue

        px = random.uniform(CENTER_X - SPREAD_XY, CENTER_X + SPREAD_XY)
        py = random.uniform(CENTER_Y - SPREAD_XY, CENTER_Y + SPREAD_XY)
        pz = START_Z + random.uniform(0.0, HEIGHT_JITTER)

        apples[i][0].set_world_pose(
            position=np.array([px, py, pz])
        )
        apples[i][0].set_linear_velocity(
            np.array([0.0, 0.0, 0.0])
        )
        i += 1
simulation_app.close()
