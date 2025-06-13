# setting.py

import bpy
import json
import math
from .core import get_BONES, get_AXES
from bpy import context

MAX_JOINTS = 8  # iiwa 7+base, UR 6+base

class TcpItem(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(name="TCP Name")

def create_getter(i):
    def getter(self):
        arm_name = bpy.context.scene.ik_motion_props.armature
        arm = bpy.data.objects.get(arm_name)
        bones = get_BONES()
        axes = get_AXES()
        
        if not arm or i >= len(bones):
            return 0.0

        bn = bones[i]
        pb = arm.pose.bones.get(bn)
        if not pb:
            return 0.0
        axis[i] = axes[i]
        return getattr(pb.rotation_euler, axis)
    return getter

def create_setter(i):
    def setter(self, value):
        arm_name = bpy.context.scene.ik_motion_props.armature
        arm = bpy.data.objects.get(arm_name)
        bones = get_BONES()
        axes = get_AXES()
        
        if not arm or i >= len(bones):
            return 0.0

        bn = bones[i]
        pb = arm.pose.bones.get(bn)
        if not pb:
            return
            
        axis[i] = axes[i]
        setattr(pb.rotation_euler, axis, value)
        bpy.context.view_layer.update()
    return setter

class JogProperties(bpy.types.PropertyGroup):
    pass

JogProperties.__annotations__ = {}

def re_register_jog_properties():
    from bpy.utils import unregister_class, register_class
    try:
        unregister_class(JogProperties)
    except Exception as e:
        print(f"[DEBUG] JogProperties unregister failed: {e}")

    try:
        register_class(JogProperties)
        if hasattr(bpy.types.Scene, "jog_props"):
            del bpy.types.Scene.jog_props
        bpy.types.Scene.jog_props = bpy.props.PointerProperty(type=JogProperties)
        print("[DEBUG] JogProperties re-registered with updated properties")
    except Exception as e:
        print(f"[ERROR] Failed to re-register JogProperties: {e}")

class StageJointItem(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(name="Label")
    value: bpy.props.FloatProperty(name="Value", default=0.0, unit='NONE')

class StageJogProperties(bpy.types.PropertyGroup):
    joints: bpy.props.CollectionProperty(type=StageJointItem)

def register_stage_properties(preset):
    items = preset.get("stage_joints", [])
    stage_data = bpy.context.scene.stage_props
    stage_data.joints.clear()

    for item in items:
        joint = stage_data.joints.add()
        joint.name = item.get("label", item.get("name", "Stage"))
        joint.value = 0.0

def re_register_stage_properties():
    from bpy.utils import unregister_class, register_class
    try:
        unregister_class(StageJointItem)
    except:
        pass
    try:
        unregister_class(StageJogProperties)
    except:
        pass

    register_class(StageJointItem)
    register_class(StageJogProperties)

    if hasattr(bpy.types.Scene, "stage_props"):
        del bpy.types.Scene.stage_props
    bpy.types.Scene.stage_props = bpy.props.PointerProperty(type=StageJogProperties)

def in_collections(collection_names):
    def _poll(self, obj):
        return any(
            c in bpy.data.collections and obj.name in bpy.data.collections[c].objects
            for c in collection_names
        )
    return _poll

def in_empty_setup():
    def _poll(self, obj):
        return obj.type == 'EMPTY' and any(
            c in bpy.data.collections and obj.name in bpy.data.collections[c].objects
            for c in ["Setup"]
        )
    return _poll

class IKMotionProperties(bpy.types.PropertyGroup):
    tcp_sorted_list: bpy.props.CollectionProperty(type=TcpItem)
    tcp_list_index: bpy.props.IntProperty(name="Selected TCP Index", default=0)
    stage_props: bpy.props.PointerProperty(type=StageJogProperties)

    selected_teach_point: bpy.props.PointerProperty(
        name="Teach Point",
        type=bpy.types.Object,
        poll=lambda self, obj: obj.name.startswith("P.")
    )

    bake_start_tcp: bpy.props.PointerProperty(
        name="Bake Start",
        type=bpy.types.Object,
        poll=in_collections(["Teach data"])
    )

    bake_end_tcp: bpy.props.PointerProperty(
        name="Bake End",
        type=bpy.types.Object,
        poll=in_collections(["Teach data"])
    )

    fixed_q3_deg: bpy.props.FloatProperty(
        name="Fixed q3",
        description="Fixed q3 value in degrees",
        default=0.0,
        min=math.radians(-170),
        max=math.radians(170),
        subtype='ANGLE',
        unit='ROTATION',
        update=lambda self, ctx: setattr(self, "fixed_q3", self.fixed_q3_deg)
    )

    fixed_q3: bpy.props.FloatProperty(
        name="fixed_q3 (rad)",  
        default=0.0,
        options={'HIDDEN'}
    )

    show_jog: bpy.props.BoolProperty(
        name="Show Jog Mode",
        default=False,
    )
    
    show_plot_joints: bpy.props.BoolVectorProperty(
        name="Plot Joints",
        description="Select joints to export",
        size=14,  # j1~j7 + stage_0~stage_6
        default=(True,) * 14
    )
    show_io: bpy.props.BoolProperty(name="Show Import/Export", default=True)
    
    show_stage: bpy.props.BoolProperty(
        name="Show Stage Jog",
        default=True
    )
    
    motion_speed: bpy.props.FloatProperty(
        name="Speed (mm/s)",
        default=100.0,
        min=1.0,
        soft_max=1000.0
    )

    wait_time_sec: bpy.props.FloatProperty(
        name="Wait Time (sec)",
        default=0.5,
        min=0.0,
        soft_max=10.0
    )

    robot_type: bpy.props.StringProperty(
        name="Robot",
        default=""
    )

    path_percent: bpy.props.FloatProperty(
        name="Path %",
        description="Interpolation value between current and next TCP",
        default=0.0,
        min=0.0,
        max=1.0,
        subtype='FACTOR'
    )

    max_solutions: bpy.props.IntProperty(
        name="Total Solutions",
        description="Total number of IK solutions (set automatically)",
        default=1,
        min=1,
        soft_max=16,
    )

    solution_index_ui: bpy.props.IntProperty(
        name="Pose Index",
        description="Select IK solution to preview or apply",
        default=1,
        min=1,
        soft_max=8,
        update=lambda self, ctx: setattr(self, "current_index", self.solution_index_ui - 1)
    )

    use_last_pose: bpy.props.BoolProperty(
        name="Keep Last Pose",
        description="Use previously selected IK solution when going to goal pose",
        default=True
    )

    goal_object: bpy.props.PointerProperty(
        name="Target",
        type=bpy.types.Object,
        poll=in_collections(["Setup"])
    )

    base_object: bpy.props.PointerProperty(
        name="Robot Base",
        type=bpy.types.Object,
        poll=in_empty_setup()
    )

    ee_object: bpy.props.PointerProperty(
        name="EE",
        type=bpy.types.Object,
        poll=in_empty_setup()
    )

    tcp_object: bpy.props.PointerProperty(
        name="TCP",
        type=bpy.types.Object,
        poll=in_empty_setup()
    )

    linear_target: bpy.props.PointerProperty(
        name="Linear Target",
        type=bpy.types.Object,
        poll=in_empty_setup()
    )

    linear_frames: bpy.props.IntProperty(
        name="ΔFrame",
        description="Number of frames for linear motion",
        default=20,
        min=1
    )

    selected_teach_point: bpy.props.PointerProperty(
        name="Teach Point",
        type=bpy.types.Object,
        poll=in_collections(["Teach data"])
    )

    step_mm: bpy.props.FloatProperty(
        name="Step (mm)",
        description="Cartesian step size for precise linear IK",
        default=5.0,
        min=0.5, soft_max=20.0
    )

    precise_linear: bpy.props.BoolProperty(
        name="High-Precision LIN",
        description="Use Lie-group planner for linear motion (KUKA only)",
        default=False
    )

    armature: bpy.props.EnumProperty(items=lambda s,c:[(o.name,o.name,'') for o in bpy.data.objects if o.type=='ARMATURE'] or [('None','None','')])
    shoulder: bpy.props.EnumProperty(items=[('L','L',''),('R','R','')], default='R')
    elbow:    bpy.props.EnumProperty(items=[('U','U',''),('D','D','')], default='U')
    wrist:    bpy.props.EnumProperty(items=[('I','I',''),('O','O','')], default='I')

    solutions_json:  bpy.props.StringProperty(default="[]")
    current_index:   bpy.props.IntProperty(default=0)
    ik_solution_str: bpy.props.StringProperty(default="Not solved")
    status_text:     bpy.props.StringProperty(default="Idle")

    ik_temp_solutions: bpy.props.StringProperty(default="[]")
    ik_temp_index:     bpy.props.IntProperty(default=0)

    show_setup:   bpy.props.BoolProperty(name="Show Setup",   default=True)
    show_target:  bpy.props.BoolProperty(name="Show Target",  default=True)
    show_teach:   bpy.props.BoolProperty(name="Show Teach",   default=True)
    show_linear:  bpy.props.BoolProperty(name="Show Linear",  default=True)
    show_pose:    bpy.props.BoolProperty(name="Show Pose",    default=True)
    show_step1: bpy.props.BoolProperty(name="Show Step 1", default=True)
    show_step2: bpy.props.BoolProperty(name="Show Step 2", default=True)
    show_step3: bpy.props.BoolProperty(name="Show Step 3", default=True)

    target_rel_x: bpy.props.FloatProperty(name="X (mm)", default=0.0)
    target_rel_y: bpy.props.FloatProperty(name="Y (mm)", default=0.0)
    target_rel_z: bpy.props.FloatProperty(name="Z (mm)", default=0.0)
    target_rel_rx: bpy.props.FloatProperty(name="RX (deg)", default=0.0)
    target_rel_ry: bpy.props.FloatProperty(name="RY (deg)", default=0.0)
    target_rel_rz: bpy.props.FloatProperty(name="RZ (deg)", default=0.0)

    preview_tcp_index: bpy.props.IntProperty(default=0)

    auto_record: bpy.props.BoolProperty(
        name="Auto Record",
        description="Record automatically when teaching",
        default=False
    )

    overwrite_selected: bpy.props.BoolProperty(
        name="Overwrite Selected",
        description="If enabled, record will overwrite the selected TCP point",
        default=False
    )

    @property
    def solutions(self):
        try: return json.loads(self.solutions_json)
        except: return []
    @solutions.setter
    def solutions(self,v): self.solutions_json=json.dumps(v)

    @property
    def temp_solutions(self):
        try: return json.loads(self.ik_temp_solutions)
        except: return []
    @temp_solutions.setter
    def temp_solutions(self, v): self.ik_temp_solutions = json.dumps(v)
