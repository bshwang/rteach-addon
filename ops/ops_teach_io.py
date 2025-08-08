import bpy
import json
import csv
import tempfile, os, sys

from pathlib import Path
from mathutils import Vector, Quaternion
from rteach.core.robot_presets import ROBOT_CONFIGS
from rteach.core.core import get_BONES, get_AXES
from rteach.config.settings_static  import get_robot_axes, get_joint_limits
from rteach.core.core import get_BONES, get_stage_joint_names, get_robot_config

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_ShowJointGraph(bpy.types.Operator):
    bl_idname = "rteach.show_joint_graph"
    bl_label = "Show Joint Timeline Graph"
    bl_description = "Plot robot & stage joint graphs filtered by UI and X-axis mode"

    def execute(self, context):
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self.report({'ERROR'}, "Matplotlib not installed: plotting unavailable")
            return {'CANCELLED'}
        
        p       = context.scene.ik_motion_props
        scene   = context.scene
        start_f = scene.frame_start if p.frame_all else p.frame_start
        end_f   = scene.frame_end   if p.frame_all else p.frame_end
        span    = max(1, end_f - start_f)
        bones   = get_BONES()
        axes    = get_robot_axes(p.robot_type)
        cfg     = ROBOT_CONFIGS.get(p.robot_type.lower(), {})
        stages  = cfg.get("stage_joints", [])

        # decide axis settings
        def get_x(frames, pct):
            if p.use_cycle:
                return pct, "Cycle (%)", (0, 100)
            else:
                return frames, "Frame", (start_f, end_f)

        # Robot plot
        fig1, ax1 = plt.subplots()
        if p.export_robot_all:
            arm = bpy.data.objects.get(p.armature)
            if arm and arm.animation_data and arm.animation_data.action:
                for i, bn in enumerate(bones):
                    axis = axes[i].lower() if i < len(axes) else 'y'
                    idx  = {'x':0,'y':1,'z':2}[axis]
                    fc   = arm.animation_data.action.fcurves.find(
                        data_path=f'pose.bones["{bn}"].rotation_euler', index=idx)
                    if not fc: continue
                    pts = [(kp.co[0], kp.co[1]) for kp in fc.keyframe_points
                           if start_f <= kp.co[0] <= end_f]
                    if not pts: continue
                    frames, vals = zip(*pts)
                    pct    = [100.0*(f-start_f)/span for f in frames]
                    degs   = [v*180.0/3.141592653589793 for v in vals]
                    x_vals, xlabel, xlim = get_x(frames, pct)
                    ax1.plot(x_vals, degs, label=f"A{i+1}")
        x_vals, xlabel, xlim = get_x([], [])
        ax1.set(xlabel=xlabel, ylabel="angle (deg)", title="Robot Joints", xlim=xlim)
        if p.export_robot_all:
            limits = get_joint_limits(p.robot_type)
            mins = [limits[i][0] for i in range(len(bones))]
            maxs = [limits[i][1] for i in range(len(bones))]
            ax1.set_ylim(min(mins), max(maxs))
        ax1.legend(loc="upper right")
        path1 = os.path.join(tempfile.gettempdir(), "rteach_robot_graph.png")
        fig1.savefig(path1); plt.close(fig1)

        # Stage plot
        fig2, ax2 = plt.subplots(); axR = ax2.twinx()
        for i, sj in enumerate(stages):
            if i >= len(p.show_plot_stage_joints) or not p.show_plot_stage_joints[i]:
                continue
            name, _, _, _, _, axis, jtype = sj
            label = name.replace("joint_", "")
            obj   = bpy.data.objects.get(name)
            idx   = {'x':0,'y':1,'z':2}[axis.lower()]
            path_str = "location" if jtype=="location" else "rotation_euler"
            vals = []
            if obj and obj.animation_data and obj.animation_data.action:
                fc = obj.animation_data.action.fcurves.find(data_path=path_str, index=idx)
                if fc:
                    vals = [(kp.co[0], kp.co[1]) for kp in fc.keyframe_points
                            if start_f <= kp.co[0] <= end_f]
            if not vals:
                v0 = getattr(obj, path_str)[idx] if obj else 0.0
                vals = [(start_f, v0), (end_f, v0)]
            if vals[0][0] > start_f: vals.insert(0, (start_f, vals[0][1]))
            if vals[-1][0] < end_f:  vals.append((end_f, vals[-1][1]))
            frames, raw = zip(*sorted(vals, key=lambda x: x[0]))
            pct = [100.0*(f-start_f)/span for f in frames]
            x_vals, xlabel, xlim = get_x(frames, pct)
            if jtype=="location":
                mm = [v*1000 for v in raw]; ax2.plot(x_vals, mm, '-', label=label)
            else:
                dg = [v*180.0/3.141592653589793 for v in raw]; axR.plot(x_vals, dg, ':', label=label)

        ax2.set(xlabel=xlabel, ylabel="location (mm)", title="Stage Joints", xlim=xlim)
        axR.set_ylabel("angle (deg)")
        lines, labs = ax2.get_legend_handles_labels()
        l2, lab2    = axR.get_legend_handles_labels()
        ax2.legend(lines+l2, labs+lab2, loc="upper right")
        path2 = os.path.join(tempfile.gettempdir(), "rteach_stage_graph.png")
        fig2.savefig(path2); plt.close(fig2)

        # display
        editors = [a for a in context.screen.areas if a.type=='IMAGE_EDITOR']
        for idx, path in enumerate((path1, path2)):
            if idx < len(editors):
                area = editors[idx]; region = next(r for r in area.regions if r.type=='WINDOW')
                ov = context.copy(); ov.update(area=area, region=region)
                bpy.ops.image.open(ov, filepath=path)
            else:
                if sys.platform.startswith('win'): os.startfile(path)
                elif sys.platform=='darwin':      os.system(f'open "{path}"')
                else:                             os.system(f'xdg-open "{path}"')

        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────   
class OBJECT_OT_export_joint_graph_csv(bpy.types.Operator):
    bl_idname = "object.export_joint_graph_csv"
    bl_label = "Export Joint Graph CSV"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature)
        if not arm:
            self.report({'ERROR'}, "No armature found")
            return {'CANCELLED'}

        bones = get_BONES()
        axes  = get_AXES()
        cfg   = ROBOT_CONFIGS.get(p.robot_type.lower(), {})
        stages = cfg.get("stage_joints", [])

        frame_start = ctx.scene.frame_start if p.frame_all else p.frame_start
        frame_end   = ctx.scene.frame_end   if p.frame_all else p.frame_end
        frames = list(range(frame_start, frame_end+1))

        headers = ["frame"]
        if p.export_robot_all:
            headers += [f"j{i+1}" for i in range(len(bones))]
        for i, sj in enumerate(stages):
            if i < len(p.show_plot_stage_joints) and p.show_plot_stage_joints[i]:
                headers.append(sj[1])

        data = []
        for f in frames:
            ctx.scene.frame_set(f)
            row = [f]
            if p.export_robot_all:
                for i, bname in enumerate(bones):
                    b = arm.pose.bones.get(bname)
                    axis = axes[i]
                    val = getattr(b.rotation_euler, axis) if b else 0.0
                    row.append(round(val,4))
            for i, sj in enumerate(stages):
                if i < len(p.show_plot_stage_joints) and p.show_plot_stage_joints[i]:
                    name = sj[0]; axis = sj[5]; jtype = sj[6]
                    ob = bpy.data.objects.get(name)
                    idx = {"x":0,"y":1,"z":2}[axis.lower()]
                    v = (ob.location[idx] if jtype=="location" else ob.rotation_euler[idx]) if ob else 0.0
                    row.append(round(v,4))
            data.append(row)

        filename = p.export_joint_csv_filename
        path = Path(bpy.path.abspath(f"//{filename}"))
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(data)

        self.report({'INFO'}, f"Exported CSV with {len(data)} frames → {path.name}")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_import_teach_data(bpy.types.Operator):
    bl_idname = "object.import_teach_data"
    bl_label = "Import Teach Data (JSON)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, ctx):

        scene = ctx.scene
        p = scene.ik_motion_props
        json_path = Path(bpy.path.abspath(f"//{p.export_teach_filename}"))

        if not json_path.exists():
            self.report({'ERROR'}, f"File not found: {json_path.name}")
            return {'CANCELLED'}

        print(f"[DEBUG] Importing teach data from: {json_path}")
        with open(json_path, "r", encoding="utf-8") as f:
            raw = json.load(f)

        if isinstance(raw, dict):
            robot_mapping = raw.get("RobotMapping", {})
            segments = raw.get("Task", [])
            
            if robot_mapping:
                ctx.scene["robot_mapping"] = robot_mapping 
                print(f"[IMPORT] Robot mapping loaded: {robot_mapping}")

            else:
                ctx.scene["robot_mapping"] = {}
                print("[IMPORT] No robot mapping found")

        elif isinstance(raw, list):
            robot_mapping = {}
            segments = raw
        else:
            self.report({'ERROR'}, "Unrecognized JSON format")
            return {'CANCELLED'}

        teach_coll = bpy.data.collections.get("Teach data")
        if not teach_coll:
            teach_coll = bpy.data.collections.new("Teach data")
            ctx.scene.collection.children.link(teach_coll)

        src = bpy.data.objects.get("Target_Gizmo") or bpy.data.objects.get("TCP_Gizmo")
        if not src:
            self.report({'ERROR'}, "Reference gizmo (Target_Gizmo or TCP_Gizmo) not found")
            return {'CANCELLED'}

        arm_name_to_base_location = {}
        rob_key_to_config_key = {}
        rob_key_to_stage_info = {}

        for rob_key, target in robot_mapping.items():
            if isinstance(target, dict):
                preset_key = target.get("config")
                joints_map = target.get("joints", {})
                rob_key_to_config_key[rob_key] = preset_key
                rob_key_to_stage_info[rob_key] = joints_map
                print(f"[IMPORT] Stage robot key mapped: {rob_key} → preset={preset_key}, joints={list(joints_map.keys())}")
            else:
                if "::" in target:
                    preset_key, _ = target.split("::", 1)
                else:
                    preset_key = target
                config = get_robot_config()
                if config:
                    rob_key_to_config_key[rob_key] = preset_key

                arm = bpy.data.objects.get(target)
                if arm:
                    arm_name_to_base_location[rob_key] = arm.matrix_world.translation.copy()
                    print(f"[IMPORT] Found robot armature in scene: {rob_key} → {target}")
                else:
                    print(f"[WARN] Robot base not found in Blender: {rob_key} → {target}")

        def side_from_key(key: str, is_stage=False) -> str:
            if is_stage:
                return "S"
            key = key.lower()
            if "left" in key:
                return "L"
            elif "right" in key:
                return "R"
            return "X"

        label_counter = {}
        generated_tcp = set()
        goal_history = set()

        manipulator_counter = 0
        stage_counter = 0

        def add_tcp_point(pt_data, label, rob_key, is_goal):
            side = side_from_key(rob_key)
            key = (label, side)

            if is_goal or key not in goal_history:
                goal_history.add(key)
                label_counter[key] = label_counter.get(key, 0) + 1
                seq = label_counter[key]
                name = f"{label}_{side}_{seq:02}"

                loc = Vector((pt_data["X"], pt_data["Y"], pt_data["Z"])) + arm_name_to_base_location[rob_key]
                rot = Quaternion((pt_data["QW"], pt_data["QX"], pt_data["QY"], pt_data["QZ"]))
                move_type = pt_data.get("MoveType", "PTP").upper()
                motion_type = "JOINT" if move_type == "PTP" else "LINEAR"

                print(f"[DEBUG] Creating {name} from robot={rob_key}")
                print(f"  Goal     = {label}")
                print(f"  Side     = {side}")
                print(f"  Raw loc  = {pt_data['X'], pt_data['Y'], pt_data['Z']}")
                print(f"  Offset   = {arm_name_to_base_location[rob_key][:]}")
                print(f"  Final loc= {loc[:]}")
                print(f"  Raw rot (quat)  = {rot[:]}")
                print(f"  Motion   = {motion_type}")

                obj = src.copy()
                obj.data = src.data.copy() if src.data else None
                obj.name = name
                obj.location = loc
                obj.rotation_mode = 'QUATERNION'
                obj.rotation_quaternion = rot
                obj.show_name = True
                obj.empty_display_size = 0.1

                obj["goal"] = label
                obj["motion_type"] = motion_type
                obj["speed"] = pt_data.get("speed", 200.0)
                obj["wait_time_sec"] = pt_data.get("wait_time_sec", 0.0)
                obj["bake_enabled"] = True
                obj["robot_key"] = rob_key
                joint_pose = []
                for i in range(1, 8):
                    key = f"A{i}"
                    if key in pt_data:
                        joint_pose.append(pt_data[key])
                obj["joint_pose"] = joint_pose

                nonlocal manipulator_counter
                obj["index"] = manipulator_counter
                manipulator_counter += 1
                print(f"[DEBUG] [MANIPULATOR] index={manipulator_counter}  → {obj.name}")

                teach_coll.objects.link(obj)
                
        def add_stage_tcp_point(pt_data, label, rob_key, is_goal):
            joint_map = robot_mapping.get(rob_key, {}).get("joints", {})
            config_key = rob_key_to_config_key.get(rob_key)
            stage_joint_defs = ROBOT_CONFIGS.get(config_key, {}).get("stage_joints", [])

            joint_values = {}
            for key, joint_name in joint_map.items():
                val = pt_data.get(key)
                if val is not None:
                    try:
                        joint_values[joint_name] = float(val)
                        print(f"[DEBUG] Stage Joint → {joint_name} = {val}")
                    except (ValueError, TypeError):
                        print(f"[WARN] Cannot convert stage joint value: {joint_name} = {val}")
                        joint_values[joint_name] = 0.0  # or fallback

            side = side_from_key(rob_key, is_stage=True)
            key = (label, side)
            if is_goal or key not in goal_history:
                goal_history.add(key)
                label_counter[key] = label_counter.get(key, 0) + 1
                seq = label_counter[key]
                name = f"{label}_{side}_{seq:02}"

                obj = bpy.data.objects.new(name, None)
                obj.empty_display_type = 'SPHERE'
                obj.empty_display_size = 0.08
                obj.show_name = True

                obj["goal"] = label
                obj["robot_key"] = rob_key
                obj["stage"] = True
                obj["joint_values"] = joint_values
                obj["bake_enabled"] = True

                nonlocal stage_counter
                obj["index"] = stage_counter
                stage_counter += 1
                obj.hide_viewport = True

                teach_coll.objects.link(obj)

        for seg in segments:
            pts = seg.get("Points", [])
            rob_key = seg.get("Robot", "")
            path = seg.get("Path", "")

            stage_joint_names = get_stage_joint_names(rob_key, rob_key_to_config_key)
            is_stage = bool(stage_joint_names)
            if not rob_key or (not is_stage and rob_key not in arm_name_to_base_location):
                print(f"[WARN] Skipping segment with unmapped robot: {rob_key}")
                print(f"[DEBUG] Known robot keys: {list(arm_name_to_base_location.keys())}")
                continue

            if not isinstance(pts, list):
                continue

            path_parts = path.replace("FROM_", "").split("_TO_")
            if len(path_parts) != 2:
                continue

            label_start, label_goal = path_parts

            for pt_pair in pts:
                start = pt_pair.get("StartPoint")
                goal = pt_pair.get("GoalPoint")

                if start:
                    if is_stage:
                        add_stage_tcp_point(start, label_start, rob_key, is_goal=False)
                    else:
                        add_tcp_point(start, label_start, rob_key, is_goal=False)
                if goal:
                    if is_stage:
                        add_stage_tcp_point(goal, label_goal, rob_key, is_goal=True)
                    else:
                        add_tcp_point(goal, label_goal, rob_key, is_goal=True)

        self.report({'INFO'}, f"Imported teach data from {json_path.name}")
        bpy.ops.object.refresh_tcp_list()
        bpy.ops.object.update_all_tcp_poses()
        bpy.ops.object.refresh_stage_tcp_list()
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────     
class OBJECT_OT_export_teach_data(bpy.types.Operator):
    bl_idname = "object.export_teach_data"
    bl_label  = "Export Teach Data (.json)"

    def execute(self, ctx):

        p       = ctx.scene.ik_motion_props
        arm_obj = bpy.data.objects.get(p.armature)
        coll    = bpy.data.collections.get("Teach data")

        if not arm_obj or not coll:
            self.report({'ERROR'}, "Armature or Teach data missing")
            return {'CANCELLED'}

        inv = arm_obj.matrix_world.inverted()
        tps = sorted(coll.objects, key=lambda o: o.get("seq", 0))

        robot_mapping = {}
        print("[EXPORT] Building RobotMapping...")

        for obj in tps:
            rob_key = obj.get("robot_key", "") or obj.get("robot", "")
            if not rob_key:
                print(f"[WARN] TCP '{obj.name}' has no 'robot_key'")
                continue
            if rob_key in robot_mapping:
                continue

            base = obj.find_armature()
            if not base:
                guess = rob_key.lower()
                base = next(
                    (obj for obj in bpy.data.objects
                     if obj.type == 'ARMATURE' and obj.name.lower().startswith(guess)),
                    None
                )
                if base:
                    print(f"[INFO] Found armature '{guess}' for robot key '{rob_key}'")
                else:
                    print(f"[FAIL] No armature found for robot '{rob_key}' → Tried: '{guess}'")

            if base:
                robot_mapping[rob_key] = base.name
            else:
                print(f"[WARN] TCP '{obj.name}' has robot '{rob_key}' but no armature resolved")

        task_dict = {}

        def make_point(obj):
            mat  = inv @ obj.matrix_world
            pos  = mat.to_translation()
            quat = mat.to_quaternion()
            qp   = obj.get("joint_pose", [])
            pt   = {}

            if not qp:
                print(f"[WARN] TCP '{obj.name}' has no joint_pose data")

            for i, v in enumerate(qp):
                pt[f"A{i+1}"] = round(v, 6)

            mt = obj.get("motion_type", "JOINT")
            pt["MoveType"] = "PTP" if mt == "JOINT" else "LIN"
            pt["QW"] = round(quat.w, 6)
            pt["QX"] = round(quat.x, 6)
            pt["QY"] = round(quat.y, 6)
            pt["QZ"] = round(quat.z, 6)
            pt["X"]  = round(pos.x, 6)
            pt["Y"]  = round(pos.y, 6)
            pt["Z"]  = round(pos.z, 6)
            return pt

        for i in range(1, len(tps)):
            prev_tp = tps[i - 1]
            tp      = tps[i]

            start_pt  = make_point(prev_tp)
            goal_pt   = make_point(tp)

            path_name = f"FROM_{prev_tp.get('goal', '')}_TO_{tp.get('goal', '')}"
            rob_key   = tp.get("robot_key", "") or tp.get("robot", "")

            if not rob_key:
                print(f"[WARN] TCP '{tp.name}' has no robot key — fallback to armature")

            task_dict.setdefault(path_name, {
                "Path": path_name,
                "Points": [],
                "Robot": rob_key or arm_obj.name
            })

            task_dict[path_name]["Points"].append({
                "StartPoint": start_pt,
                "GoalPoint":  goal_pt
            })

        full_data = {
            "RobotMapping": robot_mapping,
            "Task": list(task_dict.values())
        }

        filename = p.export_teach_filename
        filepath = bpy.path.abspath(f"//{filename}")
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(full_data, f, indent=2)

        print(f"[EXPORT] Exported Teach Data to: {filepath}")
        print(f"[EXPORT] RobotMapping = {robot_mapping}")
        print(f"[EXPORT] Task count = {len(full_data['Task'])}")

        self.report({'INFO'}, f"Exported Teach Data → {filename}")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────   
classes = (
    OBJECT_OT_export_teach_data,
    OBJECT_OT_export_joint_graph_csv,
    OBJECT_OT_ShowJointGraph,
    OBJECT_OT_import_teach_data,
)
