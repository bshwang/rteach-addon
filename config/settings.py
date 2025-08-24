import bpy
import json
import math
from rteach.core.core import apply_solution
from rteach.core.robot_presets import ROBOT_CONFIGS

class TcpItem(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(name="TCP Name")  

class StageTCPItem(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty()

def update_ik_index_preview(self, ctx):
    p = ctx.scene.ik_motion_props
    arm = bpy.data.objects.get(p.armature)

    sols = list(p.solutions) if p.solutions else []
    n = int(getattr(p, "max_solutions", len(sols)))

    if not arm:
        p.status_text = "Armature not found"
        return

    if n <= 0 or not sols:
        p.status_text = "No IK solutions"
        return

    idx_ui = int(self.solution_index_ui)
    if idx_ui < 1:
        idx_ui = 1
    if idx_ui > n:
        idx_ui = n

    if idx_ui != self.solution_index_ui:
        self.solution_index_ui = idx_ui
        p.status_text = f"Clamped IK index to {idx_ui}/{n}"

    q_sel = sols[idx_ui - 1]
    apply_solution(arm, q_sel, ctx.scene.frame_current, insert_keyframe=False)
    p.current_index = idx_ui - 1
    p.status_text = f"Preview IK {idx_ui}/{n}"

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

def delayed_workspace_update():
    result = bpy.ops.object.toggle_workspace_visibility()
    return None

def _on_stage_tcp_index_update(self, ctx):
    p = ctx.scene.ik_motion_props
    items = p.stage_tcp_sorted_list
    i = p.stage_tcp_list_index
    if not items or i < 0 or i >= len(items):
        return None

    name = items[i].name
    src = bpy.data.objects.get(name)
    if not src:
        return None

    jv = src.get("joint_values", None)
    cfg = ROBOT_CONFIGS.get(p.robot_type, {})
    for sj in cfg.get("stage_joints", []):
        key, _, _, _, _, axis, jtype = sj
        tgt = bpy.data.objects.get(key)
        if not tgt:
            continue

        ax = {"x": 0, "y": 1, "z": 2}[axis.lower()]
        val = None
        if isinstance(jv, dict):
            val = jv.get(key, None)
        elif jv is not None:
            try:
                if hasattr(jv, "keys") and key in jv.keys():
                    val = float(jv[key])
            except Exception:
                val = None
        if val is None:
            continue

        if jtype == "location":
            loc = list(tgt.location); loc[ax] = float(val); tgt.location = loc
        else:
            tgt.rotation_mode = 'XYZ'
            rot = list(tgt.rotation_euler); rot[ax] = float(val); tgt.rotation_euler = rot

    ctx.view_layer.update()
    return None

class PathItem(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(name="Goal Name")

class IKMotionProperties(bpy.types.PropertyGroup):

    tcp_sorted_list: bpy.props.CollectionProperty(type=TcpItem)
    tcp_list_index: bpy.props.IntProperty(name="Selected TCP Index", default=0)

    stage_tcp_sorted_list: bpy.props.CollectionProperty(type=TcpItem)
    stage_tcp_list_index: bpy.props.IntProperty(
        name="Selected Stage TCP Index",
        default=0,
        update=_on_stage_tcp_index_update
    )
    selected_stage_tcp: bpy.props.PointerProperty(type=bpy.types.Object)

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
        default=200.0,
        min=0.0,
        soft_max=500.0
    )

    wait_time_sec: bpy.props.FloatProperty(
        name="Wait Time (sec)",
        default=0.0,
        min=0.0,
        soft_max=10.0
    )
    
    sel_tcp_speed: bpy.props.FloatProperty(
        name="Speed (mm/s)",
        description="Selected TCP speed",
        min=0.0,
        max=500.0,
        default=0.0,
        get=lambda self: (
            self.selected_teach_point.get("speed", 0.0)
            if self.selected_teach_point else 0.0
        ),
        set=lambda self, value: (
            self.selected_teach_point.__setitem__("speed", value)
            if self.selected_teach_point else None
        ),
    )

    sel_tcp_wait: bpy.props.FloatProperty(
        name="Wait (sec)",
        description="Selected TCP wait time",
        min=0.0,
        max=10.0,
        default=0.0,
        get=lambda self: (
            self.selected_teach_point.get("wait_time_sec", 0.0)
            if self.selected_teach_point else 0.0
        ),
        set=lambda self, value: (
            self.selected_teach_point.__setitem__("wait_time_sec", value)
            if self.selected_teach_point else None
        ),
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
        default=1,
        min=1,
        soft_max=8,
        update=lambda self, ctx: update_ik_index_preview(self, ctx)
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

    show_overlay: bpy.props.BoolProperty(
        name="Show Overlay",
        description="Display overlay text in 3D View",
        default=True
    )
    
    show_robot_library: bpy.props.BoolProperty(
        name="Show Robot Library",
        default=False,
        description="Show or hide the robot selection grid"
    )
    
    show_workspace: bpy.props.BoolProperty(
        name="Show Workspace",
        default=False,
        description="Toggle workspace mesh visibility",
        update=lambda self, ctx: bpy.app.timers.register(delayed_workspace_update, first_interval=0.01)
    )

    show_robot_motion: bpy.props.BoolProperty(
        name="Show Robot Motion", default=True
    )

    show_stage_motion: bpy.props.BoolProperty(
        name="Show Stage Motion", default=True
    )

    bake_start_frame: bpy.props.IntProperty(
        name="Start Frame",
        default=1,
        min=1,
        description="Start frame for baking motion"
    )

    bake_all_tcp: bpy.props.BoolProperty(
        name="Bake All TCPs",
        default=True
    )

    bake_start_idx: bpy.props.IntProperty(
        name="Start Index",
        default=0,
        min=0
    )

    bake_end_idx: bpy.props.IntProperty(
        name="End Index",
        default=0,
        min=0
    )

    import_teach_filename: bpy.props.StringProperty(name="Import Teach JSON", default="export_teachpoint.json", subtype='FILE_NAME')

    export_teach_filename: bpy.props.StringProperty(
        name="Export Filename",
        default="export_teachpoint.json",
        description="Filename for Teach Data export (include extension, e.g. .json)"
    )
    export_joint_csv_filename: bpy.props.StringProperty(
        name="CSV Filename",
        default="export_timeline.csv",
        description="Filename for Joint CSV export (include extension, e.g. .csv)"
    )

    export_robot_all: bpy.props.BoolProperty(
        name="Robot", description="Include robot joints", default=True)
    
    import_joint_csv_filename: bpy.props.StringProperty(
        name="Import CSV",
        default="parsed_motion.csv",
        description="CSV file to import (time-series)",
        subtype='FILE_NAME'
    )

    show_plot_stage_joints: bpy.props.BoolVectorProperty(
        name="Stage Joints",
        size=8,
        description="Which stage joints to include",
        default=(True,)*8
    )

    csv_import_frame_step: bpy.props.EnumProperty(
        name="CSV Frame Step",
        items=[('10','10',''),('20','20',''),('30','30',''),('40','40',''),('50','50','')],
        default='20'
    )

    xaxis_mode: bpy.props.EnumProperty(
        name="X-Axis",
        items=[
            ('FRAME', "Frame", "Use frame number on X-Axis"),
            ('CYCLE', "Cycle (%)", "Use normalized cycle percent on X-Axis"),
        ],
        default='CYCLE',
    )
    use_cycle: bpy.props.BoolProperty(
        name="Cycle (%)",
        description="Toggle X-axis between Frame and Cycle",
        default=False
    )
    path_goals: bpy.props.CollectionProperty(type=PathItem)
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

    target_rel_x: bpy.props.FloatProperty(name="X (mm)", default=0.0)
    target_rel_y: bpy.props.FloatProperty(name="Y (mm)", default=0.0)
    target_rel_z: bpy.props.FloatProperty(name="Z (mm)", default=0.0)
    target_rel_rx: bpy.props.FloatProperty(name="RX (deg)", default=0.0)
    target_rel_ry: bpy.props.FloatProperty(name="RY (deg)", default=0.0)
    target_rel_rz: bpy.props.FloatProperty(name="RZ (deg)", default=0.0)

    preview_tcp_index: bpy.props.IntProperty(default=0)

    overwrite_selected: bpy.props.BoolProperty(
        name="Overwrite Selected",
        description="If enabled, record will overwrite the selected TCP point",
        default=False
    )
    frame_start: bpy.props.IntProperty(
        name="Start", description="Start frame for graph", default=1, min=1)
    frame_end: bpy.props.IntProperty(
        name="End", description="End frame for graph", default=250, min=1)
    frame_all: bpy.props.BoolProperty(
        name="All", description="Use entire frame range", default=False)

    @property
    def solutions(self):
        try: return json.loads(self.solutions_json)
        except: return []
    @solutions.setter
    def solutions(self,v): self.solutions_json=json.dumps([list(q) for q in v])

    @property
    def temp_solutions(self):
        try: return json.loads(self.ik_temp_solutions)
        except: return []
    @temp_solutions.setter
    def temp_solutions(self, v): self.ik_temp_solutions = json.dumps([list(q) for q in v])
