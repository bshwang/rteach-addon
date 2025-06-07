import bpy
import math
import json

import numpy as np
from mathutils import Matrix, Vector, Quaternion
from scipy.spatial.transform import Rotation as R, Slerp
from .settings import IKMotionProperties
from .robot_state import get_active_robot, set_active_robot
from .core import (
    solve_and_apply, apply_solution, get_inverse_kinematics, compute_base_matrix, 
    compute_tcp_offset_matrix, get_forward_kinematics, 
    sort_solutions, get_BONES, get_best_ik_solution,
)
from .core_iiwa import linear_move
from pathlib import Path

# ──────────────────────────────────────────────────────────────
class OBJECT_OT_clear_path_visuals(bpy.types.Operator):
	bl_idname = "object.clear_path_visuals"
	bl_label = "Clear Teach Path"

	def execute(self, ctx):	
	    pv = bpy.data.collections.get("PathVisuals")	
	    if not pv:	
	        self.report({'INFO'}, "No PathVisuals")	
	        return {'CANCELLED'}	
	    for ob in pv.objects:	
	        ob.hide_viewport = True	
	        ob.hide_set(True)
	    self.report({'INFO'}, "Path hidden")	
	    return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_draw_teach_path(bpy.types.Operator):
    bl_idname = "object.draw_teach_path"
    bl_label  = "Draw Teach Path (Coloured)"

    samples: bpy.props.IntProperty(default=20, min=4, max=100)

    def execute(self, ctx):

        p = ctx.scene.ik_motion_props
        coll = bpy.data.collections.get("Teach data")
        if not coll or len(coll.objects) < 2:
            self.report({'WARNING'}, "Need ≥2 Teach Points")
            return {'CANCELLED'}

        tps = sorted(coll.objects, key=lambda o: o.get("index", 9999))

        # 컬렉션 생성 또는 재활용
        col_name = "PathVisuals"
        pv = bpy.data.collections.get(col_name) or bpy.data.collections.new(col_name)
        if pv.name not in {c.name for c in ctx.scene.collection.children}:
            ctx.scene.collection.children.link(pv)
        for ob in list(pv.objects):
            bpy.data.objects.remove(ob, do_unlink=True)

        curve = bpy.data.curves.new("TeachPath_Curve", 'CURVE')
        curve.dimensions = '3D'
        curve.bevel_depth = 0.001

        mat_lin = bpy.data.materials.get("PathLinear") or bpy.data.materials.new("PathLinear")
        mat_lin.use_nodes     = False
        mat_lin.diffuse_color = (1, 1, 0, 0.8)
        mat_lin.blend_method  = 'BLEND'

        mat_jnt = bpy.data.materials.get("PathJoint") or bpy.data.materials.new("PathJoint")
        mat_jnt.use_nodes     = False
        mat_jnt.diffuse_color = (0.6, 0, 0.9, 0.8)
        mat_jnt.blend_method  = 'BLEND'

        curve.materials.clear()
        curve.materials.append(mat_lin)
        curve.materials.append(mat_jnt)

        fk = get_forward_kinematics()
        N = max(2, self.samples)

        for i in range(len(tps) - 1):
            A, B = tps[i], tps[i + 1]
            motion = A.get("motion_type", "LINEAR").upper()
            spline = curve.splines.new('POLY')

            if motion == "LINEAR":
                spline.material_index = 0
                pts = [A.matrix_world.translation, B.matrix_world.translation]

            elif motion == "JOINT":
                spline.material_index = 1
                q0_raw = A.get("joint_pose")
                q1_raw = B.get("joint_pose")
                if not q0_raw or not q1_raw:
                    continue
                q0 = np.asarray(q0_raw, float)
                q1 = np.asarray(q1_raw, float)
                pts = []
                for j in range(N):
                    t = j / (N - 1)
                    q = (1 - t) * q0 + t * q1
                    T_fk = fk(q)
                    pts.append(Vector(T_fk[:3, 3]))

            else:
                continue

            spline.points.add(len(pts) - 1)
            for k, v in enumerate(pts):
                spline.points[k].co = (*v, 1)

        obj = bpy.data.objects.new("TeachPath", curve)
        obj.show_in_front = False
        pv.objects.link(obj)
        self.report({'INFO'}, "Path shown")
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_tcp_move_up(bpy.types.Operator):
    """Move Teach Point up in order"""
    bl_idname = "object.tcp_move_up"
    bl_label = "Move Up"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        coll = bpy.data.collections.get("Teach data")
        if not coll:
            self.report({'ERROR'}, "Teach data collection not found")
            return {'CANCELLED'}

        sorted_tps = sorted(coll.objects, key=lambda o: o.get("index", 9999))

        i = p.tcp_list_index
        if not (0 <= i < len(sorted_tps)):
            self.report({'WARNING'}, "Invalid index")
            return {'CANCELLED'}

        if i == 0:
            self.report({'INFO'}, "Already at top")
            return {'CANCELLED'}

        obj_a = sorted_tps[i]
        obj_b = sorted_tps[i - 1]
        idx_a = obj_a.get("index", 9999)
        idx_b = obj_b.get("index", 9999)

        obj_a["index"], obj_b["index"] = idx_b, idx_a
        p.tcp_list_index = i - 1

        bpy.ops.object.draw_teach_path()
        ctx.area.tag_redraw()
        update_tcp_sorted_list()
        p.status_text = f"Moved {obj_a.name} up"
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_tcp_move_down(bpy.types.Operator):
    """Move Teach Point down in order"""
    bl_idname = "object.tcp_move_down"
    bl_label = "Move Down"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        coll = bpy.data.collections.get("Teach data")
        if not coll:
            self.report({'ERROR'}, "Teach data collection not found")
            return {'CANCELLED'}

        sorted_tps = sorted(coll.objects, key=lambda o: o.get("index", 9999))

        i = p.tcp_list_index
        if not (0 <= i < len(sorted_tps)):
            self.report({'WARNING'}, "Invalid index")
            return {'CANCELLED'}

        if i == len(sorted_tps) - 1:
            self.report({'INFO'}, "Already at bottom")
            return {'CANCELLED'}

        obj_a = sorted_tps[i]
        obj_b = sorted_tps[i + 1]
        idx_a = obj_a.get("index", 9999)
        idx_b = obj_b.get("index", 9999)

        obj_a["index"], obj_b["index"] = idx_b, idx_a
        p.tcp_list_index = i + 1

        bpy.ops.object.draw_teach_path()
        ctx.area.tag_redraw()
        update_tcp_sorted_list()
        p.status_text = f"Moved {obj_a.name} down"
        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_tcp_delete(bpy.types.Operator):
    """Delete selected Teach Point"""
    bl_idname = "object.tcp_delete"
    bl_label = "Delete TCP Point"

    name: bpy.props.StringProperty()

    def execute(self, ctx):
        obj = bpy.data.objects.get(self.name)
        if not obj:
            self.report({'ERROR'}, f"Object '{self.name}' not found")
            return {'CANCELLED'}

        # 컬렉션에서 언링크
        for coll in obj.users_collection:
            coll.objects.unlink(obj)

        # 데이터 삭제
        bpy.data.objects.remove(obj, do_unlink=True)
        ctx.scene.ik_motion_props.status_text = f"Deleted {self.name}"
        update_tcp_sorted_list()
        return {'FINISHED'}
    
# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_reindex_tcp_points(bpy.types.Operator):
    """Reassign sequential index values to all Teach Points"""
    bl_idname = "object.reindex_tcp_points"
    bl_label = "Reindex TCP"

    def execute(self, ctx):
        objs = bpy.data.collections.get("Teach data", {}).objects
        if not objs:
            self.report({'INFO'}, "No Teach Points found")
            return {'CANCELLED'}

        # Teach Point들을 index 기준으로 정렬
        sorted_objs = sorted(objs, key=lambda o: o.get("index", 9999))

        # index 재지정
        for i, obj in enumerate(sorted_objs):
            obj["index"] = i

        # ✅ selected_teach_point에 첫 번째 TCP 오브젝트를 Object 타입으로 설정
        props = ctx.scene.ik_motion_props
        props.selected_teach_point = sorted_objs[0] if sorted_objs else None
        props.status_text = "Teach Points reindexed"

        return {'FINISHED'}

# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_clear_all_tcp_points(bpy.types.Operator):
    bl_idname = "object.clear_all_tcp_points"
    bl_label  = "Clear All TCPs"

    def execute(self, ctx):
        coll = bpy.data.collections.get("Teach data")
        if coll:
            for ob in list(coll.objects):
                bpy.data.objects.remove(ob, do_unlink=True)

        path_coll = bpy.data.collections.get("Teach path")
        if path_coll:
            for ob in list(path_coll.objects):
                if ob.name.startswith("TeachPath"):
                    bpy.data.objects.remove(ob, do_unlink=True)

        props = ctx.scene.ik_motion_props
        props.selected_teach_point = None        

        if props.goal_object and props.goal_object.name not in bpy.data.objects:
            props.goal_object = None               

        props.status_text = "All Teach Points and Path cleared"
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────    
class OBJECT_OT_snap_goal_to_active(bpy.types.Operator):
    """Snap Target Gizmo to selected object"""
    bl_idname = "object.snap_goal_to_active"
    bl_label = "Snap Target to Active"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        goal = p.goal_object
        active = ctx.active_object

        if not goal:
            self.report({'ERROR'}, "Target (goal_object) not set")
            return {'CANCELLED'}
        if not active:
            self.report({'ERROR'}, "No active object selected")
            return {'CANCELLED'}

        goal.matrix_world = active.matrix_world.copy()
        goal.scale = (1, 1, 1)  # 🔧 크기 초기화 (선택적)
        goal.select_set(True)

        p.status_text = f"Snapped target to {active.name}"
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────     
class OBJECT_OT_sync_robot_type(bpy.types.Operator):
    bl_idname = "object.sync_robot_type"
    bl_label = "Sync Robot Type"

    def execute(self, ctx):
        p   = ctx.scene.ik_motion_props
        arm = bpy.data.objects.get(p.armature) or ctx.view_layer.objects.active
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Select a valid armature in Setup")
            return {'CANCELLED'}

        bone_cnt = len(arm.pose.bones)
        if p.robot_type == 'UR' and bone_cnt != 7:
            self.report({'ERROR'}, f"Armature has {bone_cnt} bones, need 7 for UR")
            return {'CANCELLED'}
        if p.robot_type == 'KUKA' and bone_cnt != 8:
            self.report({'ERROR'}, f"Armature has {bone_cnt} bones, need 8 for KUKA")
            return {'CANCELLED'}

        set_active_robot(p.robot_type)
        self.report({'INFO'}, f"Robot synced to {p.robot_type}")
        return {'FINISHED'}   
        
# ────────────────────────────────────────────────────────────── 
class OBJECT_OT_focus_on_target(bpy.types.Operator):
    bl_idname = "object.focus_on_target"
    bl_label = "Focus on Target"

    def execute(self, ctx):
        p = ctx.scene.ik_motion_props
        tgt = p.goal_object

        if not tgt:
            self.report({'ERROR'}, "No target set")
            return {'CANCELLED'}

        bpy.ops.object.select_all(action='DESELECT')
        tgt.select_set(True)
        ctx.view_layer.objects.active = tgt

        self.report({'INFO'}, f"Target '{tgt.name}' is now selected")
        return {'FINISHED'}
    
# ──────────────────────────────────────────────────────────────         
def update_tcp_sorted_list():
    p = bpy.context.scene.ik_motion_props
    coll = bpy.data.collections.get("Teach data")
    if not coll:
        return

    p.tcp_sorted_list.clear()

    for obj in sorted(coll.objects, key=lambda o: o.get("index", 9999)):
        item = p.tcp_sorted_list.add()
        item.name = obj.name

    p.tcp_list_index = min(p.tcp_list_index, len(p.tcp_sorted_list) - 1)
    bpy.context.area.tag_redraw()
    
# ──────────────────────────────────────────────────────────────     
class OBJECT_OT_export_teach_data(bpy.types.Operator):
    """Export Teach Data (TCP pose + joint angles)"""
    bl_idname = "object.export_teach_data"
    bl_label = "Export Teach Data (.json)"

    def execute(self, ctx):

        p = ctx.scene.ik_motion_props
        base = p.base_object
        coll = bpy.data.collections.get("Teach data")
        if not base or not coll:
            self.report({'ERROR'}, "Base or Teach data not found")
            return {'CANCELLED'}

        base_inv = base.matrix_world.inverted()
        tps = sorted(coll.objects, key=lambda o: o.get("index", 9999))

        data = []
        for tp in tps:
            mat_rel = base_inv @ tp.matrix_world
            pos = mat_rel.to_translation()
            quat = mat_rel.to_quaternion()

            q = tp.get("joint_pose")
            if not q:
                continue

            data.append({
                "name": tp.name,
                "position_m": [round(pos.x, 6), round(pos.y, 6), round(pos.z, 6)],
                "quaternion": [round(quat.x, 6), round(quat.y, 6), round(quat.z, 6), round(quat.w, 6)],
                "joint_pose_rad": [round(v, 6) for v in q]
            })

        if not data:
            self.report({'WARNING'}, "No teach points with joint data")
            return {'CANCELLED'}

        path = Path(bpy.path.abspath("//export_teach_data.json"))
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        self.report({'INFO'}, f"Exported {len(data)} TCPs → {path.name}")
        return {'FINISHED'}

# ──────────────────────────────────────────────────────────────   
classes = (
    OBJECT_OT_clear_path_visuals,
    OBJECT_OT_draw_teach_path,
    OBJECT_OT_tcp_move_up,
    OBJECT_OT_tcp_move_down,
    OBJECT_OT_tcp_delete,
    OBJECT_OT_reindex_tcp_points,
    OBJECT_OT_clear_all_tcp_points,
    OBJECT_OT_snap_goal_to_active,
    OBJECT_OT_sync_robot_type,
    OBJECT_OT_focus_on_target,
    OBJECT_OT_export_teach_data,
)