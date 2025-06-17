# setting.py 

import os
import bpy
import json
import math

from .core import get_BONES, get_AXES
from bpy import context

from bpy.utils import unregister_class, register_class

MAX_JOINTS = 8  # iiwa 7+base, UR 6+base

class StageJogProperties(bpy.types.PropertyGroup):

    joint_ev_z: bpy.props.FloatProperty(
        name="joint_ev_z",
        subtype='DISTANCE',
        unit='LENGTH',
        min=-0.58, max=0.22,
        get=lambda self: bpy.data.objects["joint_ev_z"].location[2]
        if "joint_ev_z" in bpy.data.objects else 0.0,
        set=lambda self, v: setattr(bpy.data.objects["joint_ev_z"].location, "z", v)
        if "joint_ev_z" in bpy.data.objects else None
    )

    joint_ev_y: bpy.props.FloatProperty(
        name="joint_ev_y",
        subtype='DISTANCE',
        unit='LENGTH',
        min=-0.4, max=0.0,
        get=lambda self: bpy.data.objects["joint_ev_y"].location[1]
        if "joint_ev_y" in bpy.data.objects else 0.0,
        set=lambda self, v: setattr(bpy.data.objects["joint_ev_y"].location, "y", v)
        if "joint_ev_y" in bpy.data.objects else None
    )

    joint_stage_x: bpy.props.FloatProperty(
        name="joint_stage_x",
        subtype='DISTANCE',
        unit='LENGTH',
        min=0.0, max=0.4,
        get=lambda self: bpy.data.objects["joint_stage_x"].location[0]
        if "joint_stage_x" in bpy.data.objects else 0.0,
        set=lambda self, v: setattr(bpy.data.objects["joint_stage_x"].location, "x", v)
        if "joint_stage_x" in bpy.data.objects else None
    )

    joint_stage_y: bpy.props.FloatProperty(
        name="joint_stage_y",
        subtype='DISTANCE',
        unit='LENGTH',
        min=-0.12, max=0.28,
        get=lambda self: bpy.data.objects["joint_stage_y"].location[1]
        if "joint_stage_y" in bpy.data.objects else 0.0,
        set=lambda self, v: setattr(bpy.data.objects["joint_stage_y"].location, "y", v)
        if "joint_stage_y" in bpy.data.objects else None
    )

    joint_stage_z: bpy.props.FloatProperty(
        name="joint_stage_z",
        subtype='DISTANCE',
        unit='LENGTH',
        min=-0.25, max=0.55,
        get=lambda self: bpy.data.objects["joint_stage_z"].location[2]
        if "joint_stage_z" in bpy.data.objects else 0.0,
        set=lambda self, v: setattr(bpy.data.objects["joint_stage_z"].location, "z", v)
        if "joint_stage_z" in bpy.data.objects else None
    )

    joint_holder_tilt: bpy.props.FloatProperty(
        name="joint_holder_tilt",
        subtype='ANGLE',
        unit='ROTATION',
        min=0.0, max=0.6109,
        get=lambda self: bpy.data.objects["joint_holder_tilt"].rotation_euler[0]
        if "joint_holder_tilt" in bpy.data.objects else 0.0,
        set=lambda self, v: setattr(bpy.data.objects["joint_holder_tilt"].rotation_euler, "x", v)
        if "joint_holder_tilt" in bpy.data.objects else None
    )

    joint_holder_rot: bpy.props.FloatProperty(
        name="joint_holder_rot",
        subtype='ANGLE',
        unit='ROTATION',
        min=0.0, max=2.3562,
        get=lambda self: bpy.data.objects["joint_holder_rot"].rotation_euler[2]
        if "joint_holder_rot" in bpy.data.objects else 0.0,
        set=lambda self, v: setattr(bpy.data.objects["joint_holder_rot"].rotation_euler, "z", v)
        if "joint_holder_rot" in bpy.data.objects else None
    )
    
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

        axis = axes[i]
        return getattr(pb.rotation_euler, axis)
    return getter

def create_setter(i):
    def setter(self, value):
        arm_name = bpy.context.scene.ik_motion_props.armature
        arm = bpy.data.objects.get(arm_name)
        bones = get_BONES()
        axes = get_AXES()

        if not arm or i >= len(bones):
            return

        bn = bones[i]
        pb = arm.pose.bones.get(bn)
        if not pb:
            return

        axis = axes[i]
        setattr(pb.rotation_euler, axis, value)
        bpy.context.view_layer.update()
    return setter

def create_stage_getter(obj_name, axis="z", unit="mm", joint_type="location"):
    def getter(self):
        obj = bpy.data.objects.get(obj_name)
        if not obj:
            return 0.0
        idx = {"x": 0, "y": 1, "z": 2}[axis[-1]]

        if joint_type.startswith("rotation"):
            return obj.rotation_euler[idx]  # ✅ 더 이상 math.degrees() 쓰지 않음
        else:
            return obj.location[idx]
    return getter

def create_stage_setter(obj_name, axis="z", unit="mm", joint_type="location"):
    def setter(self, value):
        obj = bpy.data.objects.get(obj_name)
        if not obj:
            return
        idx = {"x": 0, "y": 1, "z": 2}[axis[-1]]

        if joint_type.startswith("rotation"):
            obj.rotation_mode = 'XYZ'
            rot = list(obj.rotation_euler)
            rot[idx] = value
            obj.rotation_euler = rot
        else:
            loc = list(obj.location)
            loc[idx] = value
            obj.location = loc

        bpy.context.view_layer.update()
    return setter

class JogProperties(bpy.types.PropertyGroup):
    pass

def delayed_update_jog_props():
    from .ops_import_system import update_jog_properties
    update_jog_properties()
    print("[DEBUG] JogProperties updated after short delay")
    return None  

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
        print("[DEBUG] JogProperties re-registered")
    except Exception as e:
        print(f"[ERROR] JogProperties re-registration failed: {e}")

    bpy.app.timers.register(delayed_update_jog_props, first_interval=0.1)

def register_stage_properties(preset):
    from bpy.utils import unregister_class, register_class

    try:
        unregister_class(StageJogProperties)
    except Exception as e:
        print(f"[DEBUG] StageJogProperties unregister failed: {e}")

    if hasattr(bpy.types.Scene, "stage_props"):
        del bpy.types.Scene.stage_props

    StageJogProperties.__annotations__ = {}
    items = preset.get("stage_joints", [])
    for item in items:
        obj_name = item["name"]
        label    = item.get("label", obj_name)
        axis     = item.get("axis", "z")
        unit     = item.get("unit", "mm").lower()
        joint_type = item.get("type", "location").lower()
        min_val  = item.get("min", -1000)
        max_val  = item.get("max", 1000)

        if unit == "deg":
            min_val = math.radians(min_val)
            max_val = math.radians(max_val)
        elif unit == "mm":
            min_val /= 1000.0
            max_val /= 1000.0

        getter = create_stage_getter(obj_name, axis=axis, unit=unit, joint_type=joint_type)
        setter = create_stage_setter(obj_name, axis=axis, unit=unit, joint_type=joint_type)

        StageJogProperties.__annotations__[obj_name] = bpy.props.FloatProperty(
            name=label,
            subtype='ANGLE' if unit == "deg" else 'NONE',
            unit='ROTATION' if unit == "deg" else 'LENGTH',
            min=min_val,
            max=max_val,
            get=getter,
            set=setter
        )

    try:
        register_class(StageJogProperties)
    except Exception as e:
        print(f"[ERROR] StageJogProperties re-registration failed: {e}")
        return

    bpy.types.Scene.stage_props = bpy.props.PointerProperty(type=StageJogProperties)
    print("[DEBUG] StageJogProperties re-registered")
        
def re_register_stage_properties():
    
    cls_joint = globals().get("StageJointItem", None)
    cls_stage = globals().get("StageJogProperties", None)

    if cls_joint:
        try:
            unregister_class(cls_joint)
        except:
            pass

    if cls_stage:
        try:
            unregister_class(cls_stage)
        except:
            pass

    if cls_joint:
        register_class(cls_joint)
    if cls_stage:
        register_class(cls_stage)

    if hasattr(bpy.types.Scene, "stage_props"):
        del bpy.types.Scene.stage_props
    bpy.types.Scene.stage_props = bpy.props.PointerProperty(type=cls_stage)

JogProperties.__annotations__ = {}

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

    goal_object: bpy.props.PointerProperty(
        name="Target",
        type=bpy.types.Object,
        poll=in_collections(["Setup"])
    )

    linear_target: bpy.props.PointerProperty(
        name="Linear Target",
        type=bpy.types.Object,
        poll=in_collections(["Teach data"])
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
        
    show_stage: bpy.props.BoolProperty(
        name="Show Stage Jog",
        default=True
    )
    
    show_plot_joints: bpy.props.BoolVectorProperty(
        name="Plot Joints",
        description="Select joints to export",
        size=14,  # j1~j7 + stage_0~stage_6
        default=(True,) * 14
    )
    
    show_io: bpy.props.BoolProperty(name="Show Import/Export", default=True)

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
    
