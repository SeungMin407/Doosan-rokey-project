import numpy as np
import random
import carb
import omni
import omni.usd
from isaacsim.core.api.world import World
from isaacsim.core.utils.nucleus import get_assets_root_path
from pxr import UsdGeom, Gf, UsdShade, Sdf
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.core.api.objects import DynamicSphere
from omni.isaac.sensor import Camera

# ==============================
# 설정 값
# ==============================
APPLE_USD_PATH_B = "/home/rokey/Desktop/cooperate_3/detect_classification/map/B/pomegranate01.usd"
APPLE_USD_PATH_C = "/home/rokey/Desktop/cooperate_3/detect_classification/map/C/lychee01.usd"
TOTAL_APPLES = 15
CENTER_X, CENTER_Y = -2.5, 0.0
SPREAD_XY = 0.18
START_Z = 2
HEIGHT_JITTER = 0.4
APPLE_RADIUS = 0.05

# ==============================
# Material 생성
# ==============================
def create_omni_pbr_material_with_texture(stage, material_path, texture_path):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
    shader.CreateIdAttr("OmniPBR")
    shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(texture_path)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material

def bind_material_to_apple_root(stage, apple_prim_path, material_path):
    apple_prim = stage.GetPrimAtPath(apple_prim_path)
    material = UsdShade.Material.Get(stage, material_path)
    if not apple_prim.IsValid():
        carb.log_error(f"Invalid apple prim: {apple_prim_path}")
        return
    UsdShade.MaterialBindingAPI(apple_prim).Bind(material)

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

# 카메라 가져오기
camera_prim = stage.GetPrimAtPath("/World/TopViewCamera")
rgb_camera = Camera(prim_path="/World/TopViewCamera")

# 사과 Material 생성
for i in range(13):
    TEXTURE_PNG = f"/home/rokey/Desktop/Scratches/apple_file/apple/apple{i}.png"
    create_omni_pbr_material_with_texture(stage, f"/World/Materials/ApplePBR{i}", TEXTURE_PNG)

apple_rating = {"A":[0], "B":[1,2,3,4,5,6,7,8], "C":[9,10,11,12]}

# 사과 생성
apples = []
select_rail = 1
rail1 = []

for i in range(TOTAL_APPLES):
    good_apple = random.randint(0, 12)
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
    if good_apple==1:
        APPLE_USD_PATH=""
    elif good_apple<9:
        APPLE_USD_PATH=APPLE_USD_PATH_B
    else:
        APPLE_USD_PATH=APPLE_USD_PATH_C
    # Visual USD
    add_reference_to_stage(usd_path=APPLE_USD_PATH, prim_path=visual_path)

    # Material Bind
    bind_material_to_apple_root(stage, phys_path, f"/World/Materials/ApplePBR{good_apple}")

    apples.append([apple, select_rail])

    visual_prim = stage.GetPrimAtPath(visual_path)
    xformable = UsdGeom.Xformable(visual_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, -0.05))

    if select_rail == 1:
        rail1.append(apple)
    select_rail += 1
    if select_rail > 3:
        select_rail = 1
        
# Robot 불러오기
assets_root = get_assets_root_path()
if assets_root is None:
    carb.log_error("Isaac assets not found")

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
ur10.set_joints_default_state(positions=np.array([-np.pi/2]*4 + [np.pi/2, 0]))
world.reset()
import omni
import numpy as np
import random

# ==============================
# 상태 변수
i = 0
drop_timer = 0.0
task_phase = 0

# ==============================
# 사과 드롭 간격 (초)
DROP_DELAY_SEC = 0.5

# ==============================
# 매 프레임마다 실행되는 함수
def on_update(e):
    global i, drop_timer, task_phase

    dt = e.payload["dt"]  # 이번 프레임 경과 시간 (초)

    # ==============================
    # 사과 물리 처리
    for apple in apples:
        pos, _ = apple[0].get_world_pose()
        if 0.5 < pos[2] < 1.2:
            vel = apple[0].get_linear_velocity()
            if apple[1] == 1:
                new_vel = np.array([1.0, 0.3 * vel[1], vel[2]])
            elif apple[1] == 2:
                new_vel = np.array([0.5, 0.3 * vel[1] + 0.33, vel[2]])
            elif apple[1] == 3:
                new_vel = np.array([0.5, 0.3 * vel[1] - 0.33, vel[2]])
            apple[0].set_linear_velocity(new_vel)

    # ==============================
    # task phase 처리
    global ur10
    if task_phase == 0:
        ur10.set_world_pose(position=np.array([0.0, 0.6, 0.0]))
        task_phase += 1

    elif task_phase == 1:
        # 프레임 단위 타이머 누적
        drop_timer += dt
        if drop_timer < DROP_DELAY_SEC:
            return  # 아직 딜레이 중

        drop_timer = 0.0

        if i >= TOTAL_APPLES:
            task_phase += 1
            return

        # 사과 1개 드롭
        px = random.uniform(CENTER_X - SPREAD_XY, CENTER_X + SPREAD_XY)
        py = random.uniform(CENTER_Y - SPREAD_XY, CENTER_Y + SPREAD_XY)
        pz = START_Z + random.uniform(0.0, HEIGHT_JITTER)

        apples[i][0].set_world_pose(position=np.array([px, py, pz]))
        apples[i][0].set_linear_velocity(np.array([0.0, 0.0, 0.0]))
        i += 1

# ==============================
# Script Editor에 안전하게 등록
app = omni.kit.app.get_app()
sub = app.get_update_event_stream().create_subscription_to_pop(on_update)

print("Apple drop loop converted for Script Editor ✅")
