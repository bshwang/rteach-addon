import bpy
import json
import csv
import tempfile, os, sys
import json, re, itertools

import xml.etree.ElementTree as ET

from pathlib import Path
from mathutils import Vector, Quaternion, Matrix
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
    bl_label = "Export Teach CSV"
    bl_options = {'REGISTER', 'UNDO'}

    export_mode: bpy.props.EnumProperty(
        name="Mode",
        items=[
            ('STD_TCP', "Standard (Waypoints→CSV)", ""),
            ('KEYFRAME', "Keyframe Snapshot (Non-std)", ""),
        ],
        default='STD_TCP'
    )
    frame_step: bpy.props.EnumProperty(
        name="Frame Step",
        items=[('10','10',''),('20','20',''),('30','30',''),('40','40',''),('50','50','')],
        default='20'
    )

    filepath: bpy.props.StringProperty(subtype='FILE_PATH')
    directory: bpy.props.StringProperty(subtype='DIR_PATH')
    filename: bpy.props.StringProperty(name="File Name", default="")
    filter_glob: bpy.props.StringProperty(default="*.csv", options={'HIDDEN'})

    def invoke(self, context, event):
        import os
        import bpy
        base = os.path.splitext(os.path.basename(bpy.data.filepath))[0] or "rteach_export"
        self.filename = f"{base}_rteach.csv"
        self.filepath = bpy.path.abspath(f"//{self.filename}")
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, ctx):
        import csv, re, os
        from pathlib import Path
        from rteach.core.robot_presets import ROBOT_CONFIGS

        def sort_key(o):
            m = re.match(r'^[Tt](\d+)_([0-9]+)_', o.name)
            if m: return (int(m.group(1)), int(m.group(2)), 0 if o.get("stage") else 1)
            return (9999, int(o.get("index", 1e9)), 0 if o.get("stage") else 1)

        def dof_guess(rk: str, sample_len: int) -> int:
            if sample_len and sample_len > 0:
                return min(max(sample_len,1), 10)
            r = rk.lower()
            if "ur" in r: return 6
            if "iiwa" in r or "kuka" in r: return 7
            for _, cfg in ROBOT_CONFIGS.items():
                sets = cfg.get("armature_sets", {}) or {}
                if rk in sets or (rk + "_Arm") in sets:
                    ax = cfg.get("axes") or []
                    if isinstance(ax, (list,tuple)) and len(ax)>0:
                        return min(len(ax),10)
            return 6

        def axes_for_arm(arm_name: str, bone_count: int):
            for cfg_key, cfg in ROBOT_CONFIGS.items():
                sets = cfg.get("armature_sets", {}) or {}
                if arm_name in sets.keys():
                    axes = cfg.get("axes") or []
                    if axes:
                        ax = [str(a).lower() for a in axes]
                        if len(ax) >= bone_count: return ax[:bone_count]
                        return (ax * ((bone_count + len(ax) - 1)//len(ax)))[:bone_count]
            nm = arm_name.lower()
            if "ur" in nm: return (["z","x","x","z","x","z"] * 2)[:bone_count]
            return ["y"] * bone_count

        def bones_for_arm(arm):
            n = 0
            for i in range(1, 11):
                if arm.pose.bones.get(f"j{i}"): n = i
            if n == 0: n = 6
            return [f"j{i}" for i in range(1, n+1)]

        def axis_index(c):
            return {"x":0,"y":1,"z":2}.get((c or "y").lower(), 1)

        def stage_defs_from_scene():
            names = set()
            for cfg in ROBOT_CONFIGS.values():
                for (nm, _, _, _, _, axis, jtype) in (cfg.get("stage_joints") or []):
                    if bpy.data.objects.get(nm):
                        names.add(nm)
            out = []
            for cfg in ROBOT_CONFIGS.values():
                for (nm, _, _, _, _, axis, jtype) in (cfg.get("stage_joints") or []):
                    if nm in names:
                        out.append((nm, str(axis).lower(), str(jtype).lower()))
            return out

        if self.filepath:
            out = Path(self.filepath)
        else:
            d = Path(self.directory) if self.directory else Path(bpy.path.abspath("//"))
            fn = self.filename or "rteach_export.csv"
            out = d / fn
        if out.suffix.lower() != ".csv":
            out = out.with_suffix(".csv")
        out.parent.mkdir(parents=True, exist_ok=True)

        if self.export_mode == 'STD_TCP':
            coll = bpy.data.collections.get("Teach data")
            if not coll:
                self.report({'ERROR'}, "Teach data collection not found")
                return {'CANCELLED'}

            manip_tps = [o for o in coll.objects if "joint_pose" in o.keys() and not o.get("stage")]
            stage_flag = [o for o in coll.objects if bool(o.get("stage"))]
            stage_jvals = [o for o in coll.objects if "joint_values" in o.keys()]
            stage_all = {o.name:o for o in (stage_flag + stage_jvals)}
            stage_tps = list(stage_all.values())

            all_pts = manip_tps + stage_tps
            if not all_pts:
                self.report({'ERROR'}, "No waypoints to export")
                return {'CANCELLED'}
            all_pts.sort(key=sort_key)

            robot_keys = []
            rk_seen = set()
            rk_dof = {}
            stage_keys = []
            sk_seen = set()

            for o in all_pts:
                if o.get("stage"):
                    for k in dict(o.get("joint_values", {})).keys():
                        if k not in sk_seen:
                            sk_seen.add(k)
                            stage_keys.append(k)
                else:
                    rk = str(o.get("robot_key",""))
                    if rk and rk not in rk_seen:
                        rk_seen.add(rk)
                        n = dof_guess(rk, len(list(o.get("joint_pose", []))))
                        rk_dof[rk] = n
                        robot_keys.append(rk)
                    else:
                        if rk:
                            rk_dof[rk] = max(rk_dof.get(rk,0), len(list(o.get("joint_pose", []))) or rk_dof.get(rk,0))

            headers = ["frame"]
            for rk in sorted(robot_keys):
                for i in range(1, rk_dof.get(rk,6)+1):
                    headers.append(f"{rk}:j{i}")
            for sk in sorted(stage_keys):
                headers.append(sk)
            headers.append("goal")

            rows = []
            step = int(self.frame_step)
            frame = 1
            for o in all_pts:
                row = [""] * len(headers)
                row[0] = frame
                goal = str(o.get("goal",""))
                if o.get("stage"):
                    sv = dict(o.get("joint_values", {}))
                    for k,v in sv.items():
                        try:
                            col = headers.index(k)
                            row[col] = round(float(v), 6)
                        except ValueError:
                            pass
                else:
                    rk = str(o.get("robot_key",""))
                    q = list(map(float, o.get("joint_pose", [])))
                    dof = rk_dof.get(rk, len(q) or 6)
                    for j in range(min(dof, len(q))):
                        key = f"{rk}:j{j+1}"
                        if key in headers:
                            col = headers.index(key)
                            row[col] = round(float(q[j]), 6)
                row[-1] = goal
                rows.append(row)
                frame += step

            with open(out, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(headers)
                w.writerows(rows)

            self.report({'INFO'}, f"Exported STD CSV → {out.name}")
            return {'FINISHED'}

        else:
            arms = [a for a in bpy.data.objects if a.type=='ARMATURE']
            arms = [a for a in arms if any(a.pose.bones.get(f"j{i}") for i in range(1,11))]
            arms.sort(key=lambda x: x.name.lower())
            if not arms and not stage_defs_from_scene():
                self.report({'ERROR'}, "No armatures or stage joints found")
                return {'CANCELLED'}

            arm_bones = {}
            arm_axes = {}
            arm_fc = {}
            frame_set = set()

            for arm in arms:
                bones = bones_for_arm(arm)
                axes = axes_for_arm(arm.name, len(bones))
                arm_bones[arm.name] = bones
                arm_axes[arm.name] = axes
                arm_fc[arm.name] = {}
                act = getattr(getattr(arm.animation_data, "action", None), "fcurves", []) or []
                for i, bname in enumerate(bones):
                    idxa = axis_index(axes[i])
                    dp = f'pose.bones["{bname}"].rotation_euler'
                    fc = next((fc for fc in act if fc.data_path==dp and fc.array_index==idxa), None)
                    if fc:
                        arm_fc[arm.name][i] = fc
                        for kp in fc.keyframe_points:
                            frame_set.add(int(round(kp.co[0])))

            stage_defs = stage_defs_from_scene()
            stage_fc = {}
            for (nm, ax, jt) in stage_defs:
                ob = bpy.data.objects.get(nm)
                if not ob: continue
                idxa = axis_index(ax)
                act = getattr(getattr(ob, "animation_data", None), "action", None)
                fcurves = getattr(act, "fcurves", []) or []
                if jt == "location":
                    dp = "location"
                else:
                    dp = "rotation_euler"
                fc = next((fc for fc in fcurves if fc.data_path==dp and fc.array_index==idxa), None)
                if fc:
                    stage_fc[nm] = (fc, jt, idxa)
                    for kp in fc.keyframe_points:
                        frame_set.add(int(round(kp.co[0])))

            frames = sorted(frame_set)
            if not frames:
                self.report({'ERROR'}, "No keyframes found")
                return {'CANCELLED'}

            headers = ["frame"]
            for arm in arms:
                bones = arm_bones[arm.name]
                for j in range(len(bones)):
                    headers.append(f"{arm.name}:j{j+1}")
            for (nm, _, _) in stage_defs:
                if nm in bpy.data.objects: headers.append(nm)

            rows = []
            for f in frames:
                row = [""] * len(headers)
                row[0] = int(f)
                for arm in arms:
                    bones = arm_bones[arm.name]
                    axes = arm_axes[arm.name]
                    for j in range(len(bones)):
                        key = f"{arm.name}:j{j+1}"
                        col = headers.index(key)
                        fc = arm_fc[arm.name].get(j)
                        if fc:
                            row[col] = round(float(fc.evaluate(f)), 6)
                for (nm, jt_ax, _) in stage_defs:
                    if nm not in stage_fc: continue
                    col = headers.index(nm)
                    fc, jt, idxa = stage_fc[nm]
                    row[col] = round(float(fc.evaluate(f)), 6)
                rows.append(row)

            with open(out, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(headers)
                w.writerows(rows)

            self.report({'INFO'}, f"Exported Keyframe CSV → {out.name}")
            return {'FINISHED'}

# ──────────────────────────────────────────────────────────────     
class OBJECT_OT_export_teach_data(bpy.types.Operator):
    bl_idname = "object.export_teach_data"
    bl_label  = "Export Teach Data (.json)"
    bl_options = {'REGISTER', 'UNDO'}

    filepath: bpy.props.StringProperty(subtype='FILE_PATH')
    directory: bpy.props.StringProperty(subtype='DIR_PATH')
    filename: bpy.props.StringProperty(name="File Name", default="")
    filter_glob: bpy.props.StringProperty(default="*.json", options={'HIDDEN'})

    def invoke(self, context, event):
        import os
        import bpy
        base = os.path.splitext(os.path.basename(bpy.data.filepath))[0] or "rteach_export"
        self.filename = f"{base}_rteach.json"
        self.filepath = bpy.path.abspath(f"//{self.filename}")
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, ctx):
        import json, re, os
        from pathlib import Path

        def sort_key(o):
            m = re.match(r'^[Tt](\d+)_([0-9]+)_', o.name)
            if m: return (int(m.group(1)), int(m.group(2)), 0 if o.get("stage") else 1)
            return (9999, int(o.get("index", 1e9)), 0 if o.get("stage") else 1)

        coll = bpy.data.collections.get("Teach data")
        if not coll:
            self.report({'ERROR'}, "Teach data collection not found")
            return {'CANCELLED'}

        manip_tps = [o for o in coll.objects if "joint_pose" in o.keys() and not o.get("stage")]
        stage_flag = [o for o in coll.objects if bool(o.get("stage"))]
        stage_jvals = [o for o in coll.objects if "joint_values" in o.keys()]
        stage_all = {o.name:o for o in (stage_flag + stage_jvals)}
        stage_tps = list(stage_all.values())

        all_pts = manip_tps + stage_tps
        if not all_pts:
            self.report({'ERROR'}, "No waypoints to export")
            return {'CANCELLED'}
        all_pts.sort(key=sort_key)

        std = {"version":"1.1","units":{"length":"m","angle":"rad"},"meta":{},"waypoints":[],"paths":[]}

        idx = 0
        group = {}
        for o in all_pts:
            name = o.name
            rk = str(o.get("robot_key",""))
            goal = str(o.get("goal",""))
            if o.get("stage"):
                sv = dict(o.get("joint_values", {}))
                rec = {
                    "name": name,
                    "idx": idx,
                    "robot_key": rk,
                    "goal_label": goal,
                    "kind": "stage",
                    "stage_values": {k: float(v) for k,v in sv.items()}
                }
            else:
                mw = o.matrix_world
                loc = mw.translation
                quat = (o.rotation_quaternion if o.rotation_mode == 'QUATERNION' else mw.to_quaternion())
                q = list(map(float, o.get("joint_pose", [])))
                mt = str(o.get("motion_type","JOINT")).upper()
                sp = float(o.get("speed", 0.0))
                wt = float(o.get("wait_time_sec", 0.0))
                rec = {
                    "name": name,
                    "idx": idx,
                    "robot_key": rk,
                    "goal_label": goal,
                    "kind": "manipulator",
                    "motion": {"type": ("JOINT" if mt=="JOINT" else "LINEAR"), "speed_mm_s": sp, "wait_sec": wt},
                    "solver_hints": {"solution_index": None, "fixed_q3": None},
                    "pose": {
                        "joint": q,
                        "world": {
                            "loc": [float(loc.x), float(loc.y), float(loc.z)],
                            "quat": [float(quat.w), float(quat.x), float(quat.y), float(quat.z)]
                        }
                    }
                }
            std["waypoints"].append(rec)
            m = re.match(r'^[Tt](\d+)_', name)
            if m:
                g = int(m.group(1))
                group.setdefault(g, []).append(name)
            idx += 1

        for g in sorted(group.keys()):
            std["paths"].append({"name": f"T{g:02d}", "robot_key": "*", "sequence": group[g]})

        if self.filepath:
            out = Path(self.filepath)
        else:
            d = Path(self.directory) if self.directory else Path(bpy.path.abspath("//"))
            fn = self.filename or "rteach_export.json"
            out = d / fn
        if out.suffix.lower() != ".json":
            out = out.with_suffix(".json")
        out.parent.mkdir(parents=True, exist_ok=True)

        with open(out, "w", encoding="utf-8") as f:
            json.dump(std, f, ensure_ascii=False, indent=2)

        self.report({'INFO'}, f"Exported STD JSON → {out.name}")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────  
class OBJECT_OT_import_teach_data(bpy.types.Operator):
    bl_idname = "object.import_teach_data"
    bl_label = "Import Teach Data (STD JSON)"
    bl_options = {'REGISTER', 'UNDO'}

    filepath: bpy.props.StringProperty(subtype='FILE_PATH')
    filter_glob: bpy.props.StringProperty(default="*.json", options={'HIDDEN'})

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, ctx):
        from pathlib import Path
        import json
        from mathutils import Vector, Quaternion

        scene = ctx.scene
        p = scene.ik_motion_props

        path = None
        if self.filepath:
            path = Path(self.filepath)
        else:
            try:
                path = Path(bpy.path.abspath(f"//{p.import_teach_filename}"))
            except Exception:
                path = None

        if not path or not path.exists():
            self.report({'ERROR'}, "Select a valid JSON file")
            return {'CANCELLED'}

        print(f"[STD JSON IMPORT] file={path}")

        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        if not isinstance(data, dict) or "waypoints" not in data:
            self.report({'ERROR'}, "Unrecognized STD JSON")
            return {'CANCELLED'}

        coll = bpy.data.collections.get("Teach data")
        if not coll:
            coll = bpy.data.collections.new("Teach data")
            scene.collection.children.link(coll)

        src = bpy.data.objects.get("Target_Gizmo") or bpy.data.objects.get("TCP_Gizmo")
        if not src:
            self.report({'ERROR'}, "Reference gizmo (Target_Gizmo or TCP_Gizmo) not found")
            return {'CANCELLED'}

        manip_idx = 0
        stage_idx = 0

        waypoints = sorted(list(data.get("waypoints", [])), key=lambda w: w.get("idx", 0))

        for wp in waypoints:
            kind = (wp.get("kind") or "").lower()
            name = str(wp.get("name") or f"P{int(wp.get('idx',0)):04d}")
            robot_key = str(wp.get("robot_key") or "")
            goal_label = str(wp.get("goal_label") or "")

            if kind == "manipulator":
                pose = wp.get("pose") or {}
                world = pose.get("world") or {}
                loc = world.get("loc") or [0.0, 0.0, 0.0]
                quat = world.get("quat") or [1.0, 0.0, 0.0, 0.0]
                q = pose.get("joint") or []

                obj = src.copy()
                obj.data = src.data.copy() if src.data else None
                obj.name = name
                obj.location = Vector((float(loc[0]), float(loc[1]), float(loc[2])))
                obj.rotation_mode = 'QUATERNION'
                obj.rotation_quaternion = Quaternion((float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))
                obj.show_name = True
                obj["goal"] = goal_label
                m = wp.get("motion") or {}
                mt = str(m.get("type") or "JOINT").upper()
                obj["motion_type"] = "JOINT" if mt == "JOINT" else "LINEAR"
                obj["speed"] = float(m.get("speed_mm_s") or 0.0)
                obj["wait_time_sec"] = float(m.get("wait_sec") or 0.0)
                obj["bake_enabled"] = True
                obj["robot_key"] = robot_key
                obj["joint_pose"] = [float(v) for v in q]
                obj["index"] = manip_idx
                obj["name_preserve"] = True
                manip_idx += 1
                coll.objects.link(obj)

            elif kind == "stage":
                sv = dict(wp.get("stage_values") or {})
                obj = bpy.data.objects.new(name, None)
                obj.empty_display_type = 'SPHERE'
                obj.empty_display_size = 0.08
                obj.show_name = True
                obj["goal"] = goal_label
                obj["robot_key"] = robot_key
                obj["stage"] = True
                obj["bake_enabled"] = True
                obj["name_preserve"] = True
                obj["joint_values"] = {k: float(v) for k, v in sv.items()}
                obj["index"] = stage_idx
                stage_idx += 1
                obj.hide_viewport = True
                coll.objects.link(obj)

        try:
            bpy.ops.object.refresh_tcp_list()
        except Exception:
            pass
        try:
            bpy.ops.object.refresh_stage_tcp_list()
        except Exception:
            pass

        self.report({'INFO'}, f"Imported STD JSON: {path.name}")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────   
class OBJECT_OT_import_joint_csv(bpy.types.Operator):
    bl_idname = "object.import_joint_csv"
    bl_label  = "Import Joint CSV (STD)"
    bl_options = {'REGISTER', 'UNDO'}

    filepath: bpy.props.StringProperty(subtype='FILE_PATH')
    filter_glob: bpy.props.StringProperty(default="*.csv", options={'HIDDEN'})

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, ctx):
        from pathlib import Path
        import csv, re
        from rteach.core.robot_presets import ROBOT_CONFIGS

        p = ctx.scene.ik_motion_props

        path = None
        if self.filepath:
            path = Path(self.filepath)
        else:
            try:
                path = Path(bpy.path.abspath(f"//{p.import_joint_csv_filename}"))
            except Exception:
                path = None

        if not path or not path.exists():
            self.report({'ERROR'}, "Select a valid CSV file")
            return {'CANCELLED'}

        try:
            step_ui = int(getattr(p, "csv_import_frame_step", "20"))
        except Exception:
            step_ui = 20
        start_frame = int(ctx.scene.frame_start) if hasattr(ctx.scene, "frame_start") else 1

        print(f"[CSV IMPORT] file={path}  frame_step={step_ui}  start={start_frame}")

        arms_by_name = {a.name: a for a in bpy.data.objects if a.type == 'ARMATURE'}

        def resolve_armature(name_guess: str):
            ng = re.sub(r':+$', '', name_guess.strip())
            if ng in arms_by_name: return arms_by_name[ng]
            if ng + "_Arm" in arms_by_name: return arms_by_name[ng + "_Arm"]
            low = ng.lower()
            for nm, ob in arms_by_name.items():
                if nm.lower() == low:
                    return ob
            for nm, ob in arms_by_name.items():
                if low in nm.lower():
                    return ob
            return None

        def bones_for_arm(arm):
            n = 0
            for i in range(1, 11):
                if arm.pose.bones.get(f"j{i}"):
                    n = i
            if n == 0:
                n = 6
            return [f"j{i}" for i in range(1, n+1)]

        def axes_for_arm(arm_name: str, bone_count: int):
            for cfg_key, cfg in ROBOT_CONFIGS.items():
                sets = cfg.get("armature_sets", {}) or {}
                if arm_name in sets.keys():
                    axes = cfg.get("axes") or []
                    if isinstance(axes, (list, tuple)) and axes:
                        ax = [str(a).lower() for a in axes]
                        if len(ax) >= bone_count:
                            return ax[:bone_count]
                        return (ax * ((bone_count + len(ax) - 1)//len(ax)))[:bone_count]
            nm = arm_name.lower()
            if "ur" in nm:
                ax = ["z","x","x","z","x","z"]
                return (ax * ((bone_count + len(ax) - 1)//len(ax)))[:bone_count]
            if "kuka" in nm or "iiwa" in nm:
                return ["y"] * bone_count
            return ["y"] * bone_count

        def ensure_pose(arm):
            if arm and arm.mode != 'POSE':
                ctx.view_layer.objects.active = arm
                try: bpy.ops.object.mode_set(mode='POSE')
                except: pass

        def axis_index(c):
            return {"x":0,"y":1,"z":2}.get((c or "y").lower(), 1)

        def _ensure_linear_interp(id_data, data_path: str, index: int | None):
            ad = getattr(id_data, "animation_data", None)
            act = getattr(ad, "action", None)
            if not act: return
            for fc in act.fcurves:
                if fc.data_path != data_path: continue
                if index is not None and fc.array_index != index: continue
                for kp in fc.keyframe_points:
                    kp.interpolation = 'LINEAR'

        def key_pb(arm, pb, idx, val, frame, bone_name):
            e = list(pb.rotation_euler)
            if len(e) < 3: e = [0.0,0.0,0.0]
            e[idx] = float(val)
            pb.rotation_mode = 'XYZ'
            pb.rotation_euler = e
            dp = f'pose.bones["{bone_name}"].rotation_euler'
            pb.keyframe_insert(data_path="rotation_euler", index=idx, frame=frame)
            _ensure_linear_interp(arm, dp, idx)

        def key_loc(ob, idx, val, frame):
            v = list(ob.location)
            if len(v) < 3: v = [0.0,0.0,0.0]
            v[idx] = float(val)
            ob.location = v
            ob.keyframe_insert(data_path="location", index=idx, frame=frame)
            _ensure_linear_interp(ob, "location", idx)

        def key_rot(ob, idx, val, frame):
            if ob.rotation_mode != "XYZ":
                ob.rotation_mode = "XYZ"
            v = list(ob.rotation_euler)
            if len(v) < 3: v = [0.0,0.0,0.0]
            v[idx] = float(val)
            ob.rotation_euler = v
            ob.keyframe_insert(data_path="rotation_euler", index=idx, frame=frame)
            _ensure_linear_interp(ob, "rotation_euler", idx)

        def detect_stage_preset(stage_cols: list[str]):
            best_key, best_cov, best_map = None, 0, {}
            for cfg_key, cfg in ROBOT_CONFIGS.items():
                sj = cfg.get("stage_joints", []) or []
                m = {}
                cov = 0
                for (name, _, _, _, _, axis, jtype) in sj:
                    m[name] = (str(axis).lower(), str(jtype).lower())
                    if name in stage_cols: cov += 1
                if cov > best_cov:
                    best_key, best_cov, best_map = cfg_key, cov, m
            return best_key, best_cov, best_map

        pat_dcolon = re.compile(r'^(?P<arm>.+?)::j(?P<idx>\d+)$', re.I)
        pat_colon  = re.compile(r'^(?P<arm>.+?):j(?P<idx>\d+)$', re.I)
        pat_under  = re.compile(r'^(?P<arm>.+?)_j(?P<idx>\d+)$', re.I)

        total_robot_keys = 0
        total_stage_keys = 0

        with open(path, encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            headers = reader.fieldnames or []
            print(f"[CSV IMPORT] headers={headers}")

            stage_cols = [h for h in headers if h and h.startswith("joint_")]
            stage_cfg_key, stage_cov, stage_defs = detect_stage_preset(stage_cols)
            print(f"[CSV IMPORT] stage_cols={stage_cols}")
            print(f"[CSV IMPORT] stage_preset={stage_cfg_key} coverage={stage_cov}")
            for k in sorted(stage_cols):
                if k in stage_defs:
                    ax, jt = stage_defs[k]
                    print(f"[CSV IMPORT][STAGE MAP] {k} -> axis={ax} type={jt}")
                else:
                    print(f"[CSV IMPORT][STAGE MAP] {k} -> heuristic")

            arm_cache = {}
            axes_cache = {}
            bones_cache = {}
            robot_map_logged = set()
            stage_map_logged = set()

            row_idx = 0
            for row in reader:
                frame = start_frame + row_idx * step_ui
                row_idx += 1

                for h in headers:
                    if not h:
                        continue
                    m = pat_dcolon.match(h) or pat_colon.match(h) or pat_under.match(h)
                    if not m:
                        continue
                    arm_name_raw = m.group("arm")
                    j_idx = int(m.group("idx")) - 1
                    txt = row.get(h, "")
                    if txt in ("", None):
                        continue
                    try:
                        val = float(txt)
                    except:
                        continue

                    arm = arm_cache.get(arm_name_raw)
                    if arm is None:
                        arm = resolve_armature(arm_name_raw)
                        arm_cache[arm_name_raw] = arm
                        if arm:
                            bones_cache[arm.name] = bones_for_arm(arm)
                            axes_cache[arm.name]  = axes_for_arm(arm.name, len(bones_cache[arm.name]))
                            print(f"[CSV IMPORT][ARM] '{arm_name_raw}' -> '{arm.name}' bones={bones_cache[arm.name]} axes={axes_cache[arm.name]}")
                        else:
                            print(f"[CSV IMPORT][ARM][MISS] '{arm_name_raw}'")
                    if not arm:
                        continue

                    bones = bones_cache[arm.name]
                    if j_idx < 0 or j_idx >= len(bones):
                        print(f"[CSV IMPORT][ARM][BONE OOB] {arm.name} j{j_idx+1} > {len(bones)}")
                        continue
                    pb = arm.pose.bones.get(bones[j_idx])
                    if not pb:
                        print(f"[CSV IMPORT][ARM][MISS BONE] {arm.name} '{bones[j_idx]}'")
                        continue
                    ensure_pose(arm)
                    axes = axes_cache[arm.name]
                    idxa = axis_index(axes[j_idx])

                    if (arm.name, bones[j_idx]) not in robot_map_logged:
                        print(f"[CSV IMPORT][ARM MAP] {h} -> arm={arm.name} bone={bones[j_idx]} axis={axes[j_idx]} idx={idxa}")
                        robot_map_logged.add((arm.name, bones[j_idx]))

                    key_pb(arm, pb, idxa, val, frame, bones[j_idx])
                    total_robot_keys += 1
                    print(f"[CSV IMPORT][KEY][ARM] f={frame} arm={arm.name} bone={bones[j_idx]} axis={axes[j_idx]} val={val}")

                for col in stage_cols:
                    txt = row.get(col, "")
                    if txt in ("", None):
                        continue
                    try:
                        v = float(txt)
                    except:
                        continue

                    if col in stage_defs:
                        axis, jtyp = stage_defs[col]
                    else:
                        if col.endswith("_x"): axis, jtyp = "x", "location"
                        elif col.endswith("_y"): axis, jtyp = "y", "location"
                        elif col.endswith("_z"): axis, jtyp = "z", "location"
                        elif col.endswith("_rot"): axis, jtyp = "z", "rotation"
                        elif col.endswith("_tilt"): axis, jtyp = "x", "rotation"
                        else: axis, jtyp = "z", "location"

                    ob = bpy.data.objects.get(col)
                    if not ob:
                        print(f"[CSV IMPORT][STAGE][MISS OBJ] {col}")
                        continue

                    idxa = axis_index(axis)
                    if col not in stage_map_logged:
                        print(f"[CSV IMPORT][STAGE MAP] {col} -> axis={axis} type={jtyp} idx={idxa}")
                        stage_map_logged.add(col)

                    if jtyp == "location":
                        key_loc(ob, idxa, v, frame)
                    else:
                        key_rot(ob, idxa, v, frame)
                    total_stage_keys += 1
                    print(f"[CSV IMPORT][KEY][STAGE] f={frame} obj={col} type={jtyp} axis={axis} val={v}")

        print(f"[CSV IMPORT] done  robot_keys={total_robot_keys}  stage_keys={total_stage_keys}")
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
