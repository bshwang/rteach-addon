# setting.py

import bpy
import json
import math
from .core import get_BONES, get_AXES, get_joint_limits

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
        from bpy import context
        arm_name = context.scene.ik_motion_props.armature
        arm = bpy.data.objects.get(arm_name)
        bones = get_BONES()
        if not arm or i >= len(bones):
            return 0.0
        pb = arm.pose.bones.get(bones[i])
        if pb:
            axis = get_AXES()[i]
            val = getattr(pb.rotation_euler, axis)
            return val
        return 0.0
    return getter

def create_setter(i):
    def setter(self, value):
        from bpy import context
        arm_name = context.scene.ik_motion_props.armature
        arm = bpy.data.objects.get(arm_name)
        bones = get_BONES()
        if not arm or i >= len(bones):
            return
        pb = arm.pose.bones.get(bones[i])
        if pb:
            axis = get_AXES()[i]
            setattr(pb.rotation_euler, axis, value)
            bpy.context.view_layer.update()
    return setter

class JogProperties(bpy.types.PropertyGroup):
    pass

JogProperties.__annotations__ = {}

limits = get_joint_limits()
for i in range(MAX_JOINTS):
    deg_min, deg_max = limits[i] if i < len(limits) else (-180, 180)
    min_r = math.radians(deg_min)
    max_r = math.radians(deg_max)

    JogProperties.__annotations__[f"joint_{i}"] = bpy.props.FloatProperty(
        name=f"Joint {i}",
        subtype='ANGLE',
        unit='ROTATION',
        min=min_r,
        max=max_r,
        get=create_getter(i),
        set=create_setter(i),
    )

def in_collections(collection_names):
    def _poll(self, obj):
        return any(
            c in bpy.data.collections and obj.name in bpy.data.collections[c].objects
            for c in collection_names
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
        poll=in_collections(["Setup"])
    )

    ee_object: bpy.props.PointerProperty(
        name="EE",
        type=bpy.types.Object,
        poll=in_collections(["Setup"])
    )

    tcp_object: bpy.props.PointerProperty(
        name="TCP",
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
