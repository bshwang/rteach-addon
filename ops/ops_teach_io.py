import bpy
import json
import csv
import tempfile, os, sys
import re, itertools

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

        def get_x(frames, pct):
            if p.use_cycle:
                return pct, "Cycle (%)", (0, 100)
            else:
                return frames, "Frame", (start_f, end_f)

        def _to_py(x):
            try:
                return {k: _to_py(x[k]) for k in x.keys()}
            except Exception:
                if isinstance(x, dict):
                    return {k: _to_py(v) for k, v in x.items()}
                return x

        rm = _to_py(context.scene.get("robot_mapping", {})) or {}
        mapping_arms = {v for v in rm.values() if isinstance(v, str)}

        def _bone_sig(arm):
            try:
                return tuple(sorted(b.name for b in arm.pose.bones))
            except Exception:
                return ()

        sig_arms = set()
        sel_arm = bpy.data.objects.get(p.armature) if p.armature else None
        if sel_arm and sel_arm.type == 'ARMATURE':
            sig = _bone_sig(sel_arm)
            for ob in bpy.data.objects:
                if ob.type == 'ARMATURE' and _bone_sig(ob) == sig:
                    sig_arms.add(ob.name)

        allowed_arm_names = mapping_arms | sig_arms

        if not allowed_arm_names:
            coll = bpy.data.collections.get("Teach data")
            if coll:
                for o in coll.objects:
                    if o.get("stage"):
                        continue
                    base = o.find_armature()
                    if base:
                        allowed_arm_names.add(base.name)

        def _valid_arm(a):
            return a and a.type == 'ARMATURE' and all(a.pose.bones.get(bn) for bn in bones)

        # ── Robot plot ──
        paths = []
        if p.export_robot_all:
            arms = [bpy.data.objects.get(n) for n in sorted(allowed_arm_names)]
            arms = [a for a in arms if _valid_arm(a)]
        else:
            arm = bpy.data.objects.get(p.armature)
            arms = [arm] if _valid_arm(arm) else []

        print(f"[GRAPH] Allowed arms: {sorted(list(allowed_arm_names))}")
        print(f"[GRAPH] Selected arms: {[a.name for a in arms]}")

        for arm in arms:
            if not (arm and arm.animation_data and arm.animation_data.action):
                continue
            fig, ax = plt.subplots()
            for i, bn in enumerate(bones):
                axis = (axes[i] if i < len(axes) else 'y').lower()
                idx  = {'x':0,'y':1,'z':2}[axis]
                fc = arm.animation_data.action.fcurves.find(data_path=f'pose.bones["{bn}"].rotation_euler', index=idx)
                if not fc:
                    continue
                pts = [(kp.co[0], kp.co[1]) for kp in fc.keyframe_points if start_f <= kp.co[0] <= end_f]
                if not pts:
                    continue
                frames, vals = zip(*pts)
                pct  = [100.0*(f-start_f)/span for f in frames]
                degs = [v*180.0/3.141592653589793 for v in vals]
                x_vals, xlabel, xlim = get_x(frames, pct)
                ax.plot(x_vals, degs, label=f"A{i+1}")
            x_vals, xlabel, xlim = get_x([], [])
            ax.set(xlabel=xlabel, ylabel="angle (deg)", title=f"Robot Joints: {arm.name}", xlim=xlim)
            limits = get_joint_limits(p.robot_type)
            mins = [limits[i][0] for i in range(len(bones))]
            maxs = [limits[i][1] for i in range(len(bones))]
            ax.set_ylim(min(mins), max(maxs))
            ax.legend(loc="upper right")
            safe = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in arm.name)
            path = os.path.join(tempfile.gettempdir(), f"rteach_robot_graph_{safe}.png")
            fig.savefig(path); plt.close(fig)
            paths.append(path)
        print(f"[GRAPH] Saved robot plots: {paths}")

        # ── Stage plot ──
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
                    vals = [(kp.co[0], kp.co[1]) for kp in fc.keyframe_points if start_f <= kp.co[0] <= end_f]
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

        img_paths = list(paths)
        if 'path2' in locals() and path2:
            img_paths.append(path2)

        editors = [a for a in context.screen.areas if a.type == 'IMAGE_EDITOR']
        for idx, path in enumerate(img_paths):
            if not path:
                continue
            if idx < len(editors):
                area = editors[idx]
                region = next((r for r in area.regions if r.type == 'WINDOW'), None)
                if not region:
                    continue
                ov = context.copy()
                ov.update(area=area, region=region)
                bpy.ops.image.open(ov, filepath=path)
            else:
                if sys.platform.startswith('win'):
                    os.startfile(path)
                elif sys.platform == 'darwin':
                    os.system(f'open "{path}"')
                else:
                    os.system(f'xdg-open "{path}"')

        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────   
class OBJECT_OT_export_joint_graph_csv(bpy.types.Operator):
    bl_idname = "object.export_joint_graph_csv"
    bl_label = "Export Joint Graph CSV"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        bones = get_BONES()
        axes  = get_AXES()
        cfg   = ROBOT_CONFIGS.get(p.robot_type.lower(), {})
        stages = cfg.get("stage_joints", [])

        def _to_py(x):
            try:
                return {k: _to_py(x[k]) for k in x.keys()}
            except Exception:
                if isinstance(x, dict):
                    return {k: _to_py(v) for k, v in x.items()}
                return x

        rm = _to_py(ctx.scene.get("robot_mapping", {})) or {}
        mapping_arms = {v for v in rm.values() if isinstance(v, str)}

        def _bone_sig(arm):
            try:
                return tuple(sorted(b.name for b in arm.pose.bones))
            except Exception:
                return ()

        sig_arms = set()
        sel_arm = bpy.data.objects.get(p.armature) if p.armature else None
        if sel_arm and sel_arm.type == 'ARMATURE':
            sig = _bone_sig(sel_arm)
            for ob in bpy.data.objects:
                if ob.type == 'ARMATURE' and _bone_sig(ob) == sig:
                    sig_arms.add(ob.name)

        allowed_arm_names = mapping_arms | sig_arms

        if not allowed_arm_names:
            coll = bpy.data.collections.get("Teach data")
            if coll:
                for o in coll.objects:
                    if o.get("stage"):
                        continue
                    base = o.find_armature()
                    if base:
                        allowed_arm_names.add(base.name)

        arms = [bpy.data.objects.get(n) for n in sorted(allowed_arm_names)]
        arms = [a for a in arms if a and a.type == 'ARMATURE' and all(a.pose.bones.get(bn) for bn in bones)]

        print(f"[CSV EXPORT] Allowed arms: {sorted(list(allowed_arm_names))}")
        print(f"[CSV EXPORT] Selected arms: {[a.name for a in arms]}")

        if not arms:
            self.report({'ERROR'}, "No eligible robot armatures")
            return {'CANCELLED'}

        frame_start = ctx.scene.frame_start if p.frame_all else p.frame_start
        frame_end   = ctx.scene.frame_end   if p.frame_all else p.frame_end
        frames = list(range(frame_start, frame_end+1))

        headers = ["frame"]
        if p.export_robot_all:
            for a in arms:
                headers += [f"{a.name}:j{i+1}" for i in range(len(bones))]

        stage_headers = []
        for i, sj in enumerate(stages):
            if i < len(p.show_plot_stage_joints) and p.show_plot_stage_joints[i]:
                stage_headers.append(sj[0])

        headers += stage_headers

        print(f"[CSV EXPORT] Headers: {headers}")

        data = []
        for f in frames:
            ctx.scene.frame_set(f)
            row = [f]
            if p.export_robot_all:
                for a in arms:
                    for i, bname in enumerate(bones):
                        b = a.pose.bones.get(bname)
                        axis = axes[i]
                        val = getattr(b.rotation_euler, axis) if b else 0.0
                        row.append(round(val, 4))
            for i, sj in enumerate(stages):
                if i < len(p.show_plot_stage_joints) and p.show_plot_stage_joints[i]:
                    name = sj[0]; axis = sj[5]; jtype = sj[6]
                    ob = bpy.data.objects.get(name)
                    idx = {"x":0,"y":1,"z":2}[axis.lower()]
                    v = (ob.location[idx] if jtype == "location" else ob.rotation_euler[idx]) if ob else 0.0
                    row.append(round(v, 4))
            data.append(row)

        filename = p.export_joint_csv_filename
        path = Path(bpy.path.abspath(f"//{filename}"))
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(data)

        print(f"[CSV EXPORT] Frames: {len(data)}, Stage headers: {stage_headers}")
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
        json_path = Path(bpy.path.abspath(f"//{p.import_teach_filename}"))

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
        import json, re, itertools

        def _to_py(x):
            try:
                if isinstance(x, dict):
                    return {k: _to_py(v) for k, v in x.items()}
                if isinstance(x, (list, tuple)):
                    return [_to_py(v) for v in x]
                if hasattr(x, "keys"):
                    return {k: _to_py(x[k]) for k in x.keys()}
            except Exception:
                pass
            return x

        def _jsonify(x):
            if isinstance(x, dict):
                return {k: _jsonify(v) for k, v in x.items()}
            if isinstance(x, (list, tuple)):
                return [_jsonify(v) for v in x]
            try:
                json.dumps(x)
                return x
            except TypeError:
                return str(x)

        p       = ctx.scene.ik_motion_props
        arm_obj = bpy.data.objects.get(p.armature)
        coll    = bpy.data.collections.get("Teach data")

        if not arm_obj or not coll:
            self.report({'ERROR'}, "Armature or Teach data missing")
            return {'CANCELLED'}

        manip_tps = [o for o in coll.objects if "joint_pose" in o.keys() and not o.get("stage")]

        def _idkeys(o):
            try: return list(o.keys())
            except: return []

        stage_by_flag  = [o for o in bpy.data.objects if bool(o.get("stage"))]
        stage_by_jvals = [o for o in bpy.data.objects if "joint_values" in _idkeys(o)]
        stage_by_coll  = []
        for c in bpy.data.collections:
            if "stage" in c.name.lower():
                stage_by_coll.extend(list(c.objects))
        stage_by_coll = list({o.name: o for o in stage_by_coll}.values())
        stage_all = {}
        for o in itertools.chain(stage_by_flag, stage_by_jvals, stage_by_coll):
            stage_all[o.name] = o
        stage_tps = sorted(stage_all.values(), key=lambda o: o.get("index", 999999))

        print(f"[EXPORT] Manip TPs: {len(manip_tps)}, Stage TPs: {len(stage_tps)}")

        robot_mapping = {}
        scene_map = _to_py(ctx.scene.get("robot_mapping", {}))
        if isinstance(scene_map, dict) and scene_map:
            robot_mapping.update(scene_map)
            print(f"[EXPORT] Loaded RobotMapping from scene: {len(scene_map)}")

        print("[EXPORT] Building RobotMapping...")
        for obj in manip_tps:
            rob_key = obj.get("robot_key", "") or obj.get("robot", "")
            if not rob_key or rob_key in robot_mapping:
                continue
            base = obj.find_armature()
            if not base:
                guess = rob_key.lower()
                base = next((o for o in bpy.data.objects if o.type == 'ARMATURE' and o.name.lower().startswith(guess)), None)
                if base:
                    print(f"[INFO] Found armature '{guess}' for robot key '{rob_key}'")
                else:
                    print(f"[FAIL] No armature found for robot '{rob_key}' → Tried: '{guess}'")
            if base:
                robot_mapping[rob_key] = base.name
            else:
                print(f"[WARN] TCP '{obj.name}' has robot '{rob_key}' but no armature resolved")
        print(f"[EXPORT] RobotMapping built: {len(robot_mapping)} entries")

        base_locs = {}
        for rk in {o.get("robot_key","") or o.get("robot","") for o in manip_tps}:
            if not rk: continue
            nm = robot_mapping.get(rk)
            base = bpy.data.objects.get(nm) if isinstance(nm, str) else None
            base_locs[rk] = (base.matrix_world.translation.copy() if base else arm_obj.matrix_world.translation.copy())
        default_base_loc = arm_obj.matrix_world.translation.copy()

        def make_manip_point(obj, base_loc):
            mw = obj.matrix_world
            pos = mw.translation - base_loc
            quat = mw.to_quaternion()
            qp = obj.get("joint_pose", [])
            pt = {}
            for i, v in enumerate(qp):
                pt[f"A{i+1}"] = round(float(v), 6)
            mt = obj.get("motion_type", "JOINT")
            pt["MoveType"] = "PTP" if mt == "JOINT" else "LIN"
            pt["QW"] = round(float(quat.w), 6)
            pt["QX"] = round(float(quat.x), 6)
            pt["QY"] = round(float(quat.y), 6)
            pt["QZ"] = round(float(quat.z), 6)
            pt["X"]  = round(float(pos.x), 6)
            pt["Y"]  = round(float(pos.y), 6)
            pt["Z"]  = round(float(pos.z), 6)
            return pt

        stage_maps = {}
        for k, v in robot_mapping.items():
            if isinstance(v, dict) and isinstance(v.get("joints"), dict) and v.get("joints"):
                stage_maps[k] = v["joints"]
        print(f"[EXPORT][STAGE] Available stage robots: {list(stage_maps.keys())}")

        def applicable_stage_robots(obj):
            r = set()
            jvals = dict(obj.get("joint_values", {}))
            rk_obj = str(obj.get("robot_key","") or obj.get("robot",""))
            for rk, jm in stage_maps.items():
                names = set(jm.values())
                if any(n in jvals for n in names):
                    r.add(rk)
                    continue
                mv = robot_mapping.get(rk, {})
                cfg = mv.get("config") if isinstance(mv, dict) else None
                if rk_obj and (rk_obj == rk or rk_obj == cfg):
                    r.add(rk)
            return r

        def resolve_stage_map(rk):
            v = robot_mapping.get(rk, None)
            if isinstance(v, dict) and v.get("joints"):
                return rk, v["joints"]
            return "", {}

        def make_stage_point(obj, joints_map):
            jvals = dict(obj.get("joint_values", {}))
            pt = {}
            axes = list(joints_map.keys())
            for ax in axes:
                name = joints_map.get(ax)
                try:
                    v = float(jvals.get(name, 0.0))
                except Exception:
                    v = 0.0
                pt[ax] = round(v, 6)
            mv = obj.get("move_order", None)
            if mv is not None:
                pt["MoveOrder"] = mv
            print(f"[EXPORT][STAGE] make_stage_point {obj.name} axes={axes} -> {pt}")
            return pt

        task_dict = {}

        manip_by_robot = {}
        for o in manip_tps:
            key = o.get("robot_key", "") or o.get("robot", "")
            if not key:
                name = o.name.upper()
                if "_L_" in name: key = "MANIPULATOR_LEFT"
                elif "_R_" in name: key = "MANIPULATOR_RIGHT"
            manip_by_robot.setdefault(key, []).append(o)
        for key in manip_by_robot:
            manip_by_robot[key].sort(key=lambda o: o.get("index", 1e9))
            print(f"[EXPORT] Group {key or '(no key)'}: {len(manip_by_robot[key])} points")

        for rob_key, items in manip_by_robot.items():
            base_loc = base_locs.get(rob_key, default_base_loc)
            for i in range(1, len(items)):
                prev_tp, tp = items[i-1], items[i]
                goal_pt  = make_manip_point(tp, base_loc)
                start_pt = make_manip_point(prev_tp, base_loc)
                path_name = f"FROM_{prev_tp.get('goal','')}_TO_{tp.get('goal','')}"
                rk_out = rob_key or arm_obj.name
                key = (path_name, rk_out)
                task = task_dict.setdefault(key, {"Path": path_name, "Points": [], "Robot": rk_out})
                task["Points"].append({"GoalPoint": goal_pt, "StartPoint": start_pt})
                print(f"[EXPORT]   + Pair {i-1}->{i}: {prev_tp.name} -> {tp.name} [{rk_out}]")

        if stage_tps:
            print(f"[EXPORT] Stage sequence: {len(stage_tps)} points")
            match_cache = {}
            for o in stage_tps:
                match_cache[o.name] = sorted(list(applicable_stage_robots(o)))
                print(f"[EXPORT][STAGE] detect {o.name}: -> {match_cache[o.name] or 'None'}")
            for i in range(1, len(stage_tps)):
                prev_tp, tp = stage_tps[i - 1], stage_tps[i]
                robots = set(match_cache.get(prev_tp.name, [])) | set(match_cache.get(tp.name, []))
                if not robots:
                    print(f"[EXPORT][STAGE] Skip pair {prev_tp.name}->{tp.name}: no matching stage robots")
                    continue
                for rk in sorted(list(robots)):
                    rk_out, joints_map = resolve_stage_map(rk)
                    if not joints_map:
                        print(f"[WARN] Skip stage pair {prev_tp.name}->{tp.name}: no joints map for '{rk}'")
                        continue
                    goal_pt  = make_stage_point(tp, joints_map)
                    start_pt = make_stage_point(prev_tp, joints_map)
                    path_name = f"FROM_{prev_tp.get('goal','')}_TO_{tp.get('goal','')}"
                    key = (path_name, rk_out)
                    task = task_dict.setdefault(key, {"Path": path_name, "Points": [], "Robot": rk_out})
                    task["Points"].append({"GoalPoint": goal_pt, "StartPoint": start_pt})
                    print(f"[EXPORT]   + [STAGE:{rk_out}] {prev_tp.name}->{tp.name} | Goal {goal_pt} | Start {start_pt}")
        else:
            print("[EXPORT] No Stage TPs found")

        full_data = {
            "RobotMapping": _to_py(robot_mapping),
            "Task": list(task_dict.values())
        }

        filename = p.export_teach_filename
        filepath = bpy.path.abspath(f"//{filename}")
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(_jsonify(full_data), f, indent=2)

        print(f"[EXPORT] Exported Teach Data to: {filepath}")
        print(f"[EXPORT] RobotMapping = {_to_py(robot_mapping)}")
        print(f"[EXPORT] Task count = {len(full_data['Task'])}")
        self.report({'INFO'}, f"Exported Teach Data → {filename}")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_import_joint_csv(bpy.types.Operator):
    bl_idname = "object.import_joint_csv"
    bl_label  = "Import Joint CSV"

    def execute(self, ctx):

        p = ctx.scene.ik_motion_props
        path = Path(bpy.path.abspath(f"//{p.import_joint_csv_filename}"))
        if not path.exists():
            self.report({'ERROR'}, f"CSV not found: {path.name}")
            return {'CANCELLED'}

        bones = get_BONES()
        axes  = get_AXES()
        fps   = ctx.scene.render.fps or 24

        cfg = ROBOT_CONFIGS.get(p.robot_type.lower(), {})
        stage_defs = {}
        for sj in cfg.get("stage_joints", []):
            name = sj[0]
            axis = str(sj[5]).lower()
            jtyp = str(sj[6]).lower()
            unit = str(sj[2]).lower()
            stage_defs[name] = (axis, jtyp, unit)

        arms_by_name = {a.name: a for a in bpy.data.objects if a.type == 'ARMATURE'}

        def ensure_pose(arm):
            if arm and arm.mode != 'POSE':
                ctx.view_layer.objects.active = arm
                try: bpy.ops.object.mode_set(mode='POSE')
                except: pass

        def arm_by_casefold(name_guess):
            arm = arms_by_name.get(name_guess)
            if arm: return arm
            low = name_guess.lower()
            for nm, ob in arms_by_name.items():
                if nm.lower() == low:
                    return ob
            return None

        PI = 3.141592653589793
        EPS_M = 1e-6
        EPS_RAD = 1e-6

        def changed(prev, cur, is_rot):
            if prev is None:
                return True
            e = EPS_RAD if is_rot else EPS_M
            return abs(cur - prev) > e

        def key_pb(pb, idx, val, frame):
            e = list(pb.rotation_euler)
            e[idx] = val
            pb.rotation_euler = e
            pb.keyframe_insert(data_path="rotation_euler", index=idx, frame=frame)

        def key_loc(ob, idx, val, frame):
            v = list(ob.location)
            v[idx] = val
            ob.location = v
            ob.keyframe_insert(data_path="location", index=idx, frame=frame)

        def key_rot(ob, idx, val, frame):
            if ob.rotation_mode != "XYZ":
                ob.rotation_mode = "XYZ"
            v = list(ob.rotation_euler)
            v[idx] = val
            ob.rotation_euler = v
            ob.keyframe_insert(data_path="rotation_euler", index=idx, frame=frame)

        last_robot_val = {}
        last_robot_key = {}
        last_stage_val = {}
        last_stage_key = {}

        pat_colon = re.compile(r'^(?P<arm>.+?):j(?P<idx>\d+)$', re.I)
        pat_under = re.compile(r'^(?P<arm>.+?)_j(?P<idx>\d+)$', re.I)

        total_robot_keys = 0
        total_stage_keys = 0
        first_frame = None
        last_frame = None

        with open(path, encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            headers = reader.fieldnames or []
            frame_key = next((h for h in headers if h and h.lower() in ("frame","frames")), None)
            time_key  = next((h for h in headers if h and h.lower() in ("time","t")), None)

            stage_cols = [h for h in headers if h in stage_defs]

            for i, row in enumerate(reader, start=1):
                if frame_key:
                    try: frame = int(float(row.get(frame_key, i)))
                    except: frame = i
                elif time_key:
                    try: frame = int(round(float(row.get(time_key, 0.0)) * fps))
                    except: frame = i
                else:
                    frame = i

                if first_frame is None:
                    first_frame = frame
                last_frame = frame

                for h in headers:
                    if not h: continue
                    m = pat_colon.match(h) or pat_under.match(h)
                    if not m: continue
                    arm_name = m.group("arm")
                    j_idx = int(m.group("idx")) - 1
                    if j_idx < 0 or j_idx >= len(bones): continue
                    txt = row.get(h, "")
                    if txt in ("", None): continue
                    try: val = float(txt)
                    except: continue

                    arm = arm_by_casefold(arm_name)
                    if not arm: continue
                    pb = arm.pose.bones.get(bones[j_idx])
                    if not pb: continue
                    ensure_pose(arm)

                    ax = (axes[j_idx] if j_idx < len(axes) else 'y').lower()
                    idx = {'x':0,'y':1,'z':2}[ax]
                    if pb.rotation_mode != 'XYZ':
                        pb.rotation_mode = 'XYZ'

                    key_id = (arm.name, bones[j_idx], idx)
                    prev = last_robot_val.get(key_id)
                    if changed(prev, val, True):
                        prev_key = last_robot_key.get(key_id)
                        if prev is not None and prev_key is not None and frame-1 > prev_key:
                            key_pb(pb, idx, prev, frame-1); total_robot_keys += 1
                        key_pb(pb, idx, val, frame); total_robot_keys += 1
                        last_robot_val[key_id] = val
                        last_robot_key[key_id] = frame

                for col in stage_cols:
                    txt = row.get(col, "")
                    if txt in ("", None): continue
                    try: v_csv = float(txt)
                    except: continue

                    axis, jtyp, unit = stage_defs.get(col, ('z','location','m'))
                    idx = {'x':0,'y':1,'z':2}[axis]
                    ob = bpy.data.objects.get(col)
                    if not ob: continue

                    if jtyp == "location":
                        v_mm = v_csv * 1000.0
                        v_obj = v_mm / 1000.0
                        key_id = (ob.name, 'loc', idx)
                        prev = last_stage_val.get(key_id)
                        if changed(prev, v_mm, False):
                            prev_key = last_stage_key.get(key_id)
                            if prev is not None and prev_key is not None and frame-1 > prev_key:
                                key_loc(ob, idx, prev/1000.0, frame-1); total_stage_keys += 1
                            key_loc(ob, idx, v_obj, frame); total_stage_keys += 1
                            last_stage_val[key_id] = v_mm
                            last_stage_key[key_id] = frame
                    else:
                        v_deg = v_csv * 180.0 / PI
                        v_obj = v_deg * PI / 180.0
                        if ob.rotation_mode != "XYZ":
                            ob.rotation_mode = "XYZ"
                        key_id = (ob.name, 'rot', idx)
                        prev = last_stage_val.get(key_id)
                        if changed(prev, v_deg, True):
                            prev_key = last_stage_key.get(key_id)
                            if prev is not None and prev_key is not None and frame-1 > prev_key:
                                key_rot(ob, idx, prev*PI/180.0, frame-1); total_stage_keys += 1
                            key_rot(ob, idx, v_obj, frame); total_stage_keys += 1
                            last_stage_val[key_id] = v_deg
                            last_stage_key[key_id] = frame

        if last_frame is not None:
            for (arm_name, bname, idx), prev in last_robot_val.items():
                kf = last_robot_key.get((arm_name, bname, idx))
                if kf is not None and kf != last_frame:
                    arm = arms_by_name.get(arm_name)
                    if arm:
                        pb = arm.pose.bones.get(bname)
                        if pb:
                            key_pb(pb, idx, prev, last_frame); total_robot_keys += 1

            for (obname, kind, idx), prev in last_stage_val.items():
                kf = last_stage_key.get((obname, kind, idx))
                if kf is not None and kf != last_frame:
                    ob = bpy.data.objects.get(obname)
                    if ob:
                        if kind == 'loc':
                            key_loc(ob, idx, prev/1000.0, last_frame); total_stage_keys += 1
                        else:
                            key_rot(ob, idx, prev*PI/180.0, last_frame); total_stage_keys += 1

        print(f"[CSV IMPORT] {path.name}  robot_keys={total_robot_keys}  stage_keys={total_stage_keys}")
        self.report({'INFO'}, f"Imported CSV → {path.name}")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────   
classes = (
    OBJECT_OT_export_teach_data,
    OBJECT_OT_export_joint_graph_csv,
    OBJECT_OT_ShowJointGraph,
    OBJECT_OT_import_teach_data,
    OBJECT_OT_import_joint_csv,
)
