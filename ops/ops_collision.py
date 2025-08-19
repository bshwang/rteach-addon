import bpy
import math
import time
import re
from mathutils import Vector
from mathutils.bvhtree import BVHTree
import gpu
from gpu_extras.batch import batch_for_shader
from rteach.core.core import get_BONES

EDGE_IDX = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
COL_NEAR  = (1.0, 0.85, 0.1, 1.0)
COL_COLL  = (1.0, 0.2, 0.15, 1.0)
COL_CLEAR = (0.65, 0.65, 0.65, 1.0)
COL_DIM   = (0.25, 0.25, 0.25, 0.30)
COL_GRAB  = (0.30, 0.60, 1.00, 1.0)

_state = {
    "running": False,
    "timer": None,
    "h_view": None,
    "obj_status": {},
    "last_ms_total": 0.0,
    "last_ms_comp": 0.0,
    "last_status": "Ready",
    "dirty": True,
    "last_time": 0.0,
    "shader3d": None,
    "orig_colors": {},
    "dim_applied": False,
    "idx_cache": {},
}

def _bb_world_corners(obj):
    return [obj.matrix_world @ Vector(c) for c in obj.bound_box]

def _world_aabb(obj, depsgraph=None):
    dg = depsgraph or bpy.context.evaluated_depsgraph_get()
    e = obj.evaluated_get(dg)
    me = e.to_mesh()
    try:
        if not me or not me.vertices:
            cs = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
            xs = [c.x for c in cs]; ys = [c.y for c in cs]; zs = [c.z for c in cs]
            return Vector((min(xs), min(ys), min(zs))), Vector((max(xs), max(ys), max(zs)))
        mn = Vector((1e20, 1e20, 1e20))
        mx = Vector((-1e20, -1e20, -1e20))
        M = obj.matrix_world
        for v in me.vertices:
            p = M @ v.co
            if p.x < mn.x: mn.x = p.x
            if p.y < mn.y: mn.y = p.y
            if p.z < mn.z: mn.z = p.z
            if p.x > mx.x: mx.x = p.x
            if p.y > mx.y: mx.y = p.y
            if p.z > mx.z: mx.z = p.z
        return mn, mx
    finally:
        e.to_mesh_clear()

def _world_aabb_raw(obj):
    cs = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    xs = [c.x for c in cs]; ys = [c.y for c in cs]; zs = [c.z for c in cs]
    return Vector((min(xs), min(ys), min(zs))), Vector((max(xs), max(ys), max(zs)))

def _aabb_intersect(a_min, a_max, b_min, b_max):
    return not (a_max.x < b_min.x or a_min.x > b_max.x or
                a_max.y < b_min.y or a_min.y > b_max.y or
                a_max.z < b_min.z or a_min.z > b_max.z)

def _aabb_gap(a_min, a_max, b_min, b_max):
    dx = max(0.0, max(a_min.x - b_max.x, b_min.x - a_max.x))
    dy = max(0.0, max(a_min.y - b_max.y, b_min.y - a_max.y))
    dz = max(0.0, max(a_min.z - b_max.z, b_min.z - a_max.z))
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def _aabb_gap_components(a_min, a_max, b_min, b_max):
    dx = max(0.0, max(a_min.x - b_max.x, b_min.x - a_max.x))
    dy = max(0.0, max(a_min.y - b_max.y, b_min.y - a_max.y))
    dz = max(0.0, max(a_min.z - b_max.z, b_min.z - a_max.z))
    return dx, dy, dz

def _aabb_dims(mn, mx):
    return (mx.x - mn.x, mx.y - mn.y, mx.z - mn.z)

def _aabb_center(mn, mx):
    return Vector(((mn.x + mx.x) * 0.5, (mn.y + mx.y) * 0.5, (mn.z + mx.z) * 0.5))

def _shader3d():
    if _state["shader3d"] is not None:
        return _state["shader3d"]
    try:
        _state["shader3d"] = gpu.shader.from_builtin('3D_UNIFORM_COLOR')
    except:
        _state["shader3d"] = gpu.shader.from_builtin('UNIFORM_COLOR')
    return _state["shader3d"]

def _draw_bbox_lines(obj, color):
    corners = _bb_world_corners(obj)
    if not corners: return
    pts = []
    for a, b in EDGE_IDX:
        pts.append(corners[a]); pts.append(corners[b])
    sh = _shader3d()
    batch = batch_for_shader(sh, 'LINES', {"pos": pts})
    gpu.state.blend_set('ALPHA')
    gpu.state.depth_test_set('LESS_EQUAL')
    try: gpu.state.line_width_set(2.0)
    except: pass
    sh.bind(); sh.uniform_float("color", color)
    batch.draw(sh)
    gpu.state.blend_set('NONE')
    gpu.state.depth_test_set('LESS_EQUAL')

def _status_color(s):
    if s == "COLLIDE": return COL_COLL
    if s == "NEAR": return COL_NEAR
    return COL_CLEAR

def _set_view_color_type(color_type):
    wm = bpy.context.window_manager
    if not wm: return
    for win in wm.windows:
        for area in win.screen.areas:
            if area.type != 'VIEW_3D': continue
            for sp in area.spaces:
                if sp.type == 'VIEW_3D':
                    sp.shading.color_type = color_type

def _store_orig_color(obj):
    if obj and obj.name not in _state["orig_colors"]:
        _state["orig_colors"][obj.name] = tuple(obj.color)

def _restore_all_colors():
    for name, col in list(_state["orig_colors"].items()):
        o = bpy.data.objects.get(name)
        if o: o.color = col
    _state["orig_colors"].clear()
    _state["dim_applied"] = False

def _find_active_armature():
    sc = bpy.context.scene
    if getattr(sc, "collision_armature_name", ""):
        a = bpy.data.objects.get(sc.collision_armature_name)
        if a and a.type == 'ARMATURE': return a
    p = getattr(sc, "ik_motion_props", None)
    nm = getattr(p, "armature", "") if p else ""
    if nm and nm in bpy.data.objects and bpy.data.objects[nm].type == 'ARMATURE':
        return bpy.data.objects[nm]
    for o in bpy.data.objects:
        if o.type == 'ARMATURE' and ("UR" in o.name or "iiwa" in o.name or "KUKA" in o.name):
            return o
    return None

def _belongs_to_armature(obj, arm):
    if not arm or not obj: return False
    if obj.find_armature() == arm or obj.parent == arm: return True
    for m in obj.modifiers:
        if m.type == 'ARMATURE' and getattr(m, "object", None) == arm:
            return True
    return False

def _gather_robot_meshes():
    arm = _find_active_armature()
    if not arm: return []
    out = []
    for o in bpy.data.objects:
        if o.type == 'MESH' and _belongs_to_armature(o, arm):
            out.append(o)
    return list({*out})

def _iter_collection_meshes(coll):
    if not coll: return []
    return [o for o in coll.all_objects if o.type == 'MESH']

def _is_in_obstacle_collection(o):
    sc = bpy.context.scene
    col = getattr(sc, "collision_obstacle_collection", None)
    if not col: return False
    try:
        return o in col.all_objects
    except:
        return False

def _is_obstacle_obj(o, sc=None):
    if sc is None:
        sc = bpy.context.scene
    mode = getattr(sc, "collision_obstacle_mode", 'BOTH')
    in_coll = _is_in_obstacle_collection(o)
    tagged = bool(getattr(o, "collision_is_obstacle", False))
    if mode == 'TAG_ONLY':
        return (tagged, 'tag' if tagged else 'none')
    if mode == 'COLLECTION_ONLY':
        return (in_coll, 'collection' if in_coll else 'none')
    if tagged and in_coll: return (True, 'both')
    if tagged: return (True, 'tag')
    if in_coll: return (True, 'collection')
    return (False, 'none')

def _gather_obstacle_meshes(robot_set):
    rnames = {r.name for r in robot_set}
    obs = []
    for o in bpy.data.objects:
        if o.type != 'MESH' or o.name in rnames:
            continue
        is_obs, _src = _is_obstacle_obj(o, bpy.context.scene)
        if is_obs:
            obs.append(o)
    return list({*obs})

def _build_bvh_world(obj, depsgraph, epsilon=1e-5):
    e = obj.evaluated_get(depsgraph)
    me = e.to_mesh()
    try:
        me.calc_loop_triangles()
        if not me.loop_triangles or not me.vertices:
            return None
        verts = [obj.matrix_world @ v.co for v in me.vertices]
        tris = [tuple(t.vertices) for t in me.loop_triangles]
        return BVHTree.FromPolygons(verts, tris, all_triangles=True, epsilon=epsilon)
    finally:
        e.to_mesh_clear()

def _get_bone_index_map():
    try: bones = get_BONES() or []
    except: bones = []
    return {bn: i for i, bn in enumerate(bones)}

def _name_to_index(name, eef_hint=None):
    n = name.lower()
    if "base" in n: return 0
    m = re.search(r'link[_\s\-]?(\d+)', n)
    if m: return int(m.group(1))
    m = re.search(r'wrist[_\s\-]?(\d+)', n)
    if m:
        try: return 3 + int(m.group(1))
        except: return 6
    if any(k in n for k in ("flange","tool","tcp","ee","gripper")):
        return 100 if eef_hint is None else eef_hint
    return None

def _build_index_cache(robot, bone_index_map):
    cache = {}
    eef_idx = max(bone_index_map.values()) if bone_index_map else 100
    for r in robot:
        idx = None
        if bone_index_map:
            if r.parent_type == 'BONE' and r.parent_bone in bone_index_map:
                idx = bone_index_map[r.parent_bone]
            if idx is None and getattr(r, "vertex_groups", None):
                for vg in r.vertex_groups:
                    if vg.name in bone_index_map: idx = bone_index_map[vg.name]; break
        if idx is None:
            idx = _name_to_index(r.name, eef_hint=eef_idx)
        if idx is not None:
            cache[r.name] = idx
    return cache

def _get_link_index(obj, bone_index_map):
    if obj and obj.name in _state.get("idx_cache", {}):
        return _state["idx_cache"][obj.name]
    if bone_index_map:
        if obj.parent_type == 'BONE' and obj.parent_bone in bone_index_map:
            return bone_index_map[obj.parent_bone]
        if getattr(obj, "vertex_groups", None):
            for vg in obj.vertex_groups:
                if vg.name in bone_index_map: return bone_index_map[vg.name]
    return _name_to_index(obj.name)

def _exclude_pair(a, b, bone_index_map):
    sc = bpy.context.scene
    if not getattr(sc, "collision_exclude_adjacent", True): return False
    ai = _get_link_index(a, bone_index_map)
    bi = _get_link_index(b, bone_index_map)
    if ai is None or bi is None: return False
    if getattr(sc, "collision_exclude_same_link", True) and ai == bi: return True
    span = max(0, getattr(sc, "collision_exclude_span", 1))
    return abs(ai - bi) <= span

def _pair_hit(a_min, a_max, b_min, b_max, br, bo, mode):
    if not _aabb_intersect(a_min, a_max, b_min, b_max):
        return False
    if mode == 'AABB': return True
    if br is None or bo is None: return True
    return bool(br.overlap(bo))

def _eef_index(bone_index_map):
    if not bone_index_map: return None
    return max(bone_index_map.values())

def _is_object_grabbed(obj, arm, bone_index_map):
    if getattr(obj, "collision_is_grabbed", False): return True
    sc = bpy.context.scene
    if not sc.collision_grab_auto or not arm: return False
    eef_bn = None
    if bone_index_map:
        inv = {i: bn for bn, i in bone_index_map.items()}
        eef_bn = inv.get(_eef_index(bone_index_map))
    if obj.parent_type == 'BONE' and obj.parent == arm:
        pb = obj.parent_bone or ""
        if pb == eef_bn or any(k in pb.lower() for k in ("tool","tcp","ee")):
            return True
    for c in obj.constraints:
        if c.type == 'CHILD_OF' and c.target == arm and c.subtarget:
            sub = c.subtarget
            if sub == eef_bn or any(k in sub.lower() for k in ("tool","tcp","ee")):
                return True
    return False

def _grab_owner(obj):
    sc = bpy.context.scene
    o = getattr(obj, "collision_grab_by_obj", None)
    if o: return o, "prop"
    if getattr(sc, "collision_grab_auto_object", True):
        if obj.parent and getattr(obj.parent, "type", "") == 'MESH':
            return obj.parent, "parent"
        for c in obj.constraints:
            if c.type == 'CHILD_OF' and c.target and getattr(c.target, "type", "") == 'MESH':
                return c.target, "childof"
    return None, None

def _is_grabbed_by(a, b):
    owner, _ = _grab_owner(a)
    return owner == b

def _expand_aabb(mn, mx, pad):
    return Vector((mn.x - pad, mn.y - pad, mn.z - pad)), Vector((mx.x + pad, mx.y + pad, mx.z + pad))

def _apply_tint_map(status_map):
    _restore_all_colors()
    sc = bpy.context.scene
    if not sc.collision_tint_enable:
        _set_view_color_type('MATERIAL')
        return
    _set_view_color_type('OBJECT')
    robot = _gather_robot_meshes()
    obs = _gather_obstacle_meshes(robot)
    rset = {o.name for o in robot}
    oset = {o.name for o in obs}
    for name in (rset | oset):
        s = status_map.get(name, "CLEAR")
        obj = bpy.data.objects.get(name)
        if not obj: continue
        _store_orig_color(obj)
        col = _status_color(s)
        if s == "CLEAR" and getattr(obj, "collision_is_grabbed", False):
            col = COL_GRAB
        obj.color = col
    for o in bpy.data.objects:
        if o.type != 'MESH': continue
        if o.name in rset or o.name in oset or getattr(o, "collision_is_grabbed", False):
            continue
        _store_orig_color(o)
        o.color = COL_DIM
    _state["dim_applied"] = True

def _refresh_tint_now(recompute=False):
    if recompute or not _state["obj_status"]:
        _compute_status()
    else:
        _apply_tint_map(_state["obj_status"])
    for win in bpy.context.window_manager.windows:
        for area in win.screen.areas:
            if area.type == 'VIEW_3D': area.tag_redraw()

def _skip_obs_obs_pair(a, b):
    sc = bpy.context.scene
    if _is_grabbed_by(a, b) or _is_grabbed_by(b, a):
        return True
    if getattr(sc, "collision_obsobs_grab_exclude", False):
        if getattr(a, "collision_is_grabbed", False) or getattr(b, "collision_is_grabbed", False):
            return True
    return getattr(a, "collision_is_grabbed", False) and getattr(b, "collision_is_grabbed", False)

def _role_with_src(obj, robot_set):
    sc = bpy.context.scene
    rnames = {r.name for r in robot_set}
    if obj.name in rnames:
        return "ROBOT", "RobotSet"
    srcs = []
    coll = getattr(sc, "collision_obstacle_collection", None)
    try:
        if coll and obj in coll.all_objects:
            srcs.append("Collection")
    except:
        pass
    if getattr(obj, "collision_is_obstacle", False):
        srcs.append("Tag")
    if srcs:
        return "OBSTACLE", "+".join(srcs)
    return "SCENE", "-"

def _eval_mesh_counts(obj, depsgraph):
    e = obj.evaluated_get(depsgraph)
    me = e.to_mesh()
    try:
        me.calc_loop_triangles()
        return len(me.vertices), len(me.loop_triangles)
    finally:
        e.to_mesh_clear()

def _debug_pair_details(a, b, mode, dg, eps):
    ra_min, ra_max = _world_aabb_raw(a)
    rb_min, rb_max = _world_aabb_raw(b)
    ea_min, ea_max = _world_aabb(a, dg)
    eb_min, eb_max = _world_aabb(b, dg)
    g_raw = _aabb_gap(ra_min, ra_max, rb_min, rb_max)
    g_eval = _aabb_gap(ea_min, ea_max, eb_min, eb_max)
    dx, dy, dz = _aabb_gap_components(ea_min, ea_max, eb_min, eb_max)
    da = _aabb_dims(ea_min, ea_max); db = _aabb_dims(eb_min, eb_max)
    ca = _aabb_center(ea_min, ea_max); cb = _aabb_center(eb_min, eb_max)
    cd = (cb - ca).length
    va, ta = _eval_mesh_counts(a, dg); vb, tb = _eval_mesh_counts(b, dg)
    print(f"    AABB(raw)  gap={g_raw*1000:.2f} mm intersect={'yes' if _aabb_intersect(ra_min, ra_max, rb_min, rb_max) else 'no'}")
    print(f"    AABB(eval) gap={g_eval*1000:.2f} mm intersect={'yes' if _aabb_intersect(ea_min, ea_max, eb_min, eb_max) else 'no'} | dx={dx*1000:.2f} dy={dy*1000:.2f} dz={dz*1000:.2f} mm")
    print(f"    A dims={da[0]*1000:.2f}/{da[1]*1000:.2f}/{da[2]*1000:.2f} mm  B dims={db[0]*1000:.2f}/{db[1]*1000:.2f}/{db[2]*1000:.2f} mm")
    print(f"    Centers A={ca.x:.3f},{ca.y:.3f},{ca.z:.3f}  B={cb.x:.3f},{cb.y:.3f},{cb.z:.3f} | center_dist={cd*1000:.2f} mm")
    print(f"    Mesh eval: {a.name} v={va} t={ta} | {b.name} v={vb} t={tb} | eps={eps*1e6:.2f} µm mode={mode}")

def _compute_status():
    sc = bpy.context.scene
    t0 = time.perf_counter()
    robot = _gather_robot_meshes()
    obs = _gather_obstacle_meshes(robot)
    dg = bpy.context.evaluated_depsgraph_get()
    thr = max(0.0, float(sc.collision_safe_margin_mm)) * 0.001
    mode = sc.collision_detection_mode
    eps = float(sc.collision_mesh_eps_um) * 1e-6

    per_obj = {}
    for r in robot: per_obj[r.name] = "CLEAR"
    for o in obs:   per_obj[o.name] = "CLEAR"

    bone_index_map = _get_bone_index_map()
    _state["idx_cache"] = _build_index_cache(robot, bone_index_map)
    arm = _find_active_armature()
    eef_idx = _eef_index(bone_index_map)
    grab_span = max(0, sc.collision_grab_span)

    aabb_r = {r: _world_aabb(r, dg) for r in robot}
    aabb_o = {o: _world_aabb(o, dg) for o in obs}
    t1 = time.perf_counter()

    if robot:
        rmin = Vector(( 1e20,  1e20,  1e20))
        rmax = Vector((-1e20, -1e20, -1e20))
        for r in robot:
            mn, mx = aabb_r[r]
            rmin.x = min(rmin.x, mn.x); rmin.y = min(rmin.y, mn.y); rmin.z = min(rmin.z, mn.z)
            rmax.x = max(rmax.x, mx.x); rmax.y = max(rmax.y, mx.y); rmax.z = max(rmax.z, mx.z)
        region_min, region_max = _expand_aabb(rmin, rmax, thr)
    else:
        region_min, region_max = Vector((0,0,0)), Vector((0,0,0))

    def in_region(o):
        if not robot: return True
        mn, mx = aabb_o[o]
        return _aabb_intersect(region_min, region_max, mn, mx)

    bvh_r = {}
    bvh_o = {}

    def get_bvh_r(x):
        if mode == 'AABB': return None
        b = bvh_r.get(x)
        if b is None:
            b = _build_bvh_world(x, dg, epsilon=eps); bvh_r[x] = b
        return b

    def get_bvh_o(x):
        if mode == 'AABB': return None
        b = bvh_o.get(x)
        if b is None:
            b = _build_bvh_world(x, dg, epsilon=eps); bvh_o[x] = b
        return b

    for i in range(len(robot)):
        for j in range(i+1, len(robot)):
            a = robot[i]; b = robot[j]
            if _exclude_pair(a, b, bone_index_map): continue
            a_min, a_max = aabb_r[a]; b_min, b_max = aabb_r[b]
            hit = _pair_hit(a_min, a_max, b_min, b_max, get_bvh_r(a), get_bvh_r(b), mode)
            if hit:
                per_obj[a.name] = "COLLIDE"; per_obj[b.name] = "COLLIDE"

    for r in robot:
        for o in obs:
            if not in_region(o): continue
            a_min, a_max = aabb_r[r]; b_min, b_max = aabb_o[o]
            if _is_object_grabbed(o, arm, bone_index_map):
                ri = _get_link_index(r, bone_index_map)
                if (ri is not None) and (eef_idx is not None) and abs(ri - eef_idx) <= grab_span:
                    continue
            hit = _pair_hit(a_min, a_max, b_min, b_max, get_bvh_r(r), get_bvh_o(o), mode)
            if hit:
                per_obj[r.name] = "COLLIDE"; per_obj[o.name] = "COLLIDE"
            else:
                gap = _aabb_gap(a_min, a_max, b_min, b_max)
                if gap <= thr and per_obj[r.name] != "COLLIDE" and per_obj[o.name] != "COLLIDE":
                    if per_obj[r.name] == "CLEAR": per_obj[r.name] = "NEAR"
                    if per_obj[o.name] == "CLEAR": per_obj[o.name] = "NEAR"

    if sc.collision_check_obstacles_pairs and len(obs) >= 2:
        for i in range(len(obs)):
            if not in_region(obs[i]): continue
            for j in range(i+1, len(obs)):
                if not in_region(obs[j]): continue
                a = obs[i]; b = obs[j]
                if _skip_obs_obs_pair(a, b): continue
                a_min, a_max = aabb_o[a]; b_min, b_max = aabb_o[b]
                hit = _pair_hit(a_min, a_max, b_min, b_max, get_bvh_o(a), get_bvh_o(b), mode)
                if hit:
                    per_obj[a.name] = "COLLIDE"; per_obj[b.name] = "COLLIDE"
                else:
                    gap = _aabb_gap(a_min, a_max, b_min, b_max)
                    if gap <= thr and per_obj[a.name] != "COLLIDE" and per_obj[b.name] != "COLLIDE":
                        if per_obj[a.name] == "CLEAR": per_obj[a.name] = "NEAR"
                        if per_obj[b.name] == "CLEAR": per_obj[b.name] = "NEAR"

    _state["obj_status"] = per_obj
    _apply_tint_map(per_obj)

    t2 = time.perf_counter()
    _state["last_ms_total"] = (t2 - t0) * 1000.0
    _state["last_ms_comp"]  = (t2 - t1) * 1000.0
    st = "COLLIDE" if any(v == "COLLIDE" for v in per_obj.values()) else ("NEAR" if any(v == "NEAR" for v in per_obj.values()) else "CLEAR")
    _state["last_status"] = f"Status: {st} | Robot={len(robot)} Obstacles={len(obs)} Collide={sum(1 for v in per_obj.values() if v=='COLLIDE')} Near={sum(1 for v in per_obj.values() if v=='NEAR')}"

def _draw_callback():
    sc = bpy.context.scene
    if not sc.collision_outline_enable: return
    if not _state["obj_status"]: return
    for name, s in _state["obj_status"].items():
        o = bpy.data.objects.get(name)
        if o: _draw_bbox_lines(o, _status_color(s))

def _depsgraph_listener(_dg):
    if _state["running"]:
        _state["dirty"] = True

class RTEACH_OT_collision_start(bpy.types.Operator):
    bl_idname = "rteach.collision_start"
    bl_label = "Start Live"
    bl_options = {'REGISTER'}
    def modal(self, context, event):
        if not _state["running"]:
            self.cancel(context); return {'CANCELLED'}
        if event.type == 'TIMER':
            now = time.perf_counter()
            interval = max(5, context.scene.collision_interval_ms) / 1000.0
            if _state["dirty"] or (now - _state["last_time"] >= interval):
                _compute_status()
                _state["dirty"] = False
                _state["last_time"] = now
                for win in context.window_manager.windows:
                    for area in win.screen.areas:
                        if area.type == 'VIEW_3D': area.tag_redraw()
        return {'PASS_THROUGH'}
    def execute(self, context):
        if _state["running"]: return {'CANCELLED'}
        _state["running"] = True
        _state["dirty"] = True
        _state["last_time"] = 0.0
        _refresh_tint_now(recompute=True)
        _set_view_color_type('OBJECT' if context.scene.collision_tint_enable else 'MATERIAL')
        wm = context.window_manager
        _state["timer"] = wm.event_timer_add(0.02, window=context.window)
        if _state["h_view"] is None:
            _state["h_view"] = bpy.types.SpaceView3D.draw_handler_add(_draw_callback, (), 'WINDOW', 'POST_VIEW')
        if _depsgraph_listener not in bpy.app.handlers.depsgraph_update_post:
            bpy.app.handlers.depsgraph_update_post.append(_depsgraph_listener)
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}
    def cancel(self, context):
        wm = context.window_manager
        if _state["timer"]: wm.event_timer_remove(_state["timer"]); _state["timer"] = None

class RTEACH_OT_collision_stop(bpy.types.Operator):
    bl_idname = "rteach.collision_stop"
    bl_label = "Stop Live"
    bl_options = {'REGISTER'}
    def execute(self, context):
        _state["running"] = False
        if _state["timer"]:
            try: context.window_manager.event_timer_remove(_state["timer"])
            except: pass
            _state["timer"] = None
        if _state["h_view"]:
            try: bpy.types.SpaceView3D.draw_handler_remove(_state["h_view"], 'WINDOW')
            except: pass
            _state["h_view"] = None
        try: bpy.app.handlers.depsgraph_update_post.remove(_depsgraph_listener)
        except: pass
        _restore_all_colors()
        _set_view_color_type('MATERIAL')
        return {'FINISHED'}

class RTEACH_OT_collision_scan_once(bpy.types.Operator):
    bl_idname = "rteach.collision_scan_once"
    bl_label = "Quick Check"
    bl_options = {'REGISTER'}
    def execute(self, context):
        _compute_status()
        if _state["h_view"] is None:
            _state["h_view"] = bpy.types.SpaceView3D.draw_handler_add(_draw_callback, (), 'WINDOW', 'POST_VIEW')
            bpy.app.timers.register(lambda: (_remove_once_draw(), None)[1], first_interval=max(0.2, float(context.scene.collision_scan_draw_sec)))
        for win in context.window_manager.windows:
            for area in win.screen.areas:
                if area.type == 'VIEW_3D': area.tag_redraw()
        return {'FINISHED'}

def _remove_once_draw():
    if _state["h_view"]:
        try: bpy.types.SpaceView3D.draw_handler_remove(_state["h_view"], 'WINDOW')
        except: pass
        _state["h_view"] = None

class RTEACH_OT_collision_scan_motion(bpy.types.Operator):
    bl_idname = "rteach.collision_scan_motion"
    bl_label = "Scan Keyframes"
    bl_options = {'REGISTER'}
    def execute(self, context):
        scn = context.scene
        arm = _find_active_armature()
        frames = set()
        if arm and arm.animation_data and arm.animation_data.action:
            act = arm.animation_data.action
            for fc in act.fcurves:
                for kp in fc.keyframe_points:
                    frames.add(int(round(kp.co[0])))
        if not frames:
            fs, fe = scn.frame_start, scn.frame_end
            frames = set(range(int(fs), int(fe)+1, 1))
        frames = sorted(frames)
        sub = max(0, int(scn.collision_scan_substeps))
        if sub > 0 and len(frames) >= 2:
            ext = []
            for i in range(len(frames)-1):
                a, b = frames[i], frames[i+1]
                ext.append(a)
                step = max(1, (b - a) // (sub + 1))
                for t in range(a + step, b, step):
                    ext.append(t)
            ext.append(frames[-1])
            frames = sorted(set(ext))
        for m in list(scn.timeline_markers):
            if m.name.startswith("COL_") or m.name.startswith("NEAR_"):
                scn.timeline_markers.remove(m)
        was_col = False; col_start = None
        was_near = False; near_start = None
        cur = scn.frame_current
        for f in frames:
            scn.frame_set(f)
            _compute_status()
            vals = _state["obj_status"].values()
            is_col = any(v == "COLLIDE" for v in vals)
            is_near = (not is_col) and any(v == "NEAR" for v in vals)
            if is_col and not was_col: col_start = f; was_col = True
            if not is_col and was_col:
                scn.timeline_markers.new(f"COL_START_{col_start}", frame=col_start)
                scn.timeline_markers.new(f"COL_END_{f-1}", frame=f-1)
                was_col = False; col_start = None
            if is_near and not was_near: near_start = f; was_near = True
            if not is_near and was_near:
                scn.timeline_markers.new(f"NEAR_START_{near_start}", frame=near_start)
                scn.timeline_markers.new(f"NEAR_END_{f-1}", frame=f-1)
                was_near = False; near_start = None
        if was_col and col_start is not None:
            scn.timeline_markers.new(f"COL_START_{col_start}", frame=col_start)
            scn.timeline_markers.new(f"COL_END_{frames[-1]}", frame=frames[-1])
        if was_near and near_start is not None:
            scn.timeline_markers.new(f"NEAR_START_{near_start}", frame=near_start)
            scn.timeline_markers.new(f"NEAR_END_{frames[-1]}", frame=frames[-1])
        scn.frame_set(cur)
        self.report({'INFO'}, f"Scanned {len(frames)} frames")
        return {'FINISHED'}

class RTEACH_OT_collision_clear_markers(bpy.types.Operator):
    bl_idname = "rteach.collision_clear_markers"
    bl_label = "Clear Marks"
    bl_options = {'REGISTER'}
    def execute(self, context):
        scn = context.scene
        removed = 0
        for m in list(scn.timeline_markers):
            if m.name.startswith("COL_") or m.name.startswith("NEAR_"):
                scn.timeline_markers.remove(m); removed += 1
        self.report({'INFO'}, f"Removed {removed} markers")
        return {'FINISHED'}

class RTEACH_OT_collision_probe_selected(bpy.types.Operator):
    bl_idname = "rteach.collision_probe_selected"
    bl_label = "Inspect Selected"
    bl_options = {'REGISTER'}
    def execute(self, context):
        sel = [o for o in context.selected_objects if o and o.type == 'MESH']
        if not sel:
            self.report({'WARNING'}, "Select one or more meshes"); return {'CANCELLED'}
        sc = context.scene
        mode = sc.collision_detection_mode
        eps_um = float(sc.collision_mesh_eps_um)
        eps = eps_um * 1e-6
        thr = max(0.0, float(sc.collision_safe_margin_mm)) * 0.001
        thr_mm = thr * 1000.0
        dg = bpy.context.evaluated_depsgraph_get()
        robot = _gather_robot_meshes()
        obs = _gather_obstacle_meshes(robot)
        robot_set = set(robot)
        bone_index_map = _get_bone_index_map()
        _state["idx_cache"] = _build_index_cache(robot, bone_index_map)
        arm = _find_active_armature()
        eef_idx = _eef_index(bone_index_map)
        grab_span = max(0, sc.collision_grab_span)

        aabb_cache = {}
        bvh_cache  = {}

        def aabb(o):
            v = aabb_cache.get(o.name)
            if v is None:
                v = _world_aabb(o, dg)
                aabb_cache[o.name] = v
            return v

        def bvh(o):
            v = bvh_cache.get(o.name)
            if v is None:
                v = None if mode == 'AABB' else _build_bvh_world(o, dg, eps)
                bvh_cache[o.name] = v
            return v

        def _pair_compute(a, b):
            a_min, a_max = aabb(a); b_min, b_max = aabb(b)
            gap = _aabb_gap(a_min, a_max, b_min, b_max) * 1000.0
            aabb_hit = _aabb_intersect(a_min, a_max, b_min, b_max)
            mhit = False
            if aabb_hit and mode != 'AABB':
                br = bvh(a); bo = bvh(b)
                mhit = False if (br is None or bo is None) else bool(br.overlap(bo))
            res = "COLLIDE" if (aabb_hit and (mode == 'AABB' or mhit)) else ("NEAR" if gap <= thr_mm else "CLEAR")
            return res, gap, aabb_hit, (mhit if mode != 'AABB' else None)

        def _pair_exclude(a, b):
            ra = a in robot_set; rb = b in robot_set
            if ra and rb:
                if _exclude_pair(a, b, bone_index_map):
                    ai = _get_link_index(a, bone_index_map); bi = _get_link_index(b, bone_index_map)
                    if getattr(sc, "collision_exclude_same_link", True) and ai is not None and bi is not None and ai == bi:
                        return True, "same_link"
                    return True, "adjacent"
                return False, "-"
            if ra and not rb:
                if b not in obs:
                    return True, "not_obstacle"
                if _is_object_grabbed(b, arm, bone_index_map):
                    ri = _get_link_index(a, bone_index_map)
                    if (ri is not None) and (eef_idx is not None) and abs(ri - eef_idx) <= grab_span:
                        return True, "grab_robot_span"
                return False, "-"
            if rb and not ra:
                if a not in obs:
                    return True, "not_obstacle"
                if _is_object_grabbed(a, arm, bone_index_map):
                    ri = _get_link_index(b, bone_index_map)
                    if (ri is not None) and (eef_idx is not None) and abs(ri - eef_idx) <= grab_span:
                        return True, "grab_robot_span"
                return False, "-"
            # obs/scene vs obs/scene
            if _is_grabbed_by(a, b) or _is_grabbed_by(b, a):
                return True, "grab_owner"
            if _skip_obs_obs_pair(a, b):
                return True, "grab_exclude"
            return False, "-"

        def pair_report(a, b, idx=None):
            res, gap, aabb_hit, mhit = _pair_compute(a, b)
            ex, why = _pair_exclude(a, b)
            tag_a = ("ROBOT" if a in robot_set else ("OBSTACLE" if a in obs else "SCENE"))
            tag_b = ("ROBOT" if b in robot_set else ("OBSTACLE" if b in obs else "SCENE"))
            head = f"[{idx}] " if idx is not None else ""
            mh = '-' if mode == 'AABB' else ('yes' if mhit else 'no')
            print(f"  {head}{a.name}({tag_a}) vs {b.name}({tag_b}) -> result={res} | gap={gap:.2f} mm | thr={thr_mm:.2f} mm | exclude={'yes' if ex else 'no'} | reason={why} | aabb_hit={'yes' if aabb_hit else 'no'} | mesh_hit={mh}")
            return res, ex

        # 다중 선택: 모든 pair 리포트
        if len(sel) >= 2:
            names = ", ".join([o.name for o in sel])
            print(f"[InspectPairs] count={len(sel)} | mode={mode} | thr={thr_mm:.2f} mm | mesh_eps={eps_um:.2f} µm | obstacle_mode={getattr(sc,'collision_obstacle_mode','BOTH')}")
            print(f"  selected: {names}")
            any_col = False; any_near = False
            k = 1
            for i in range(len(sel)):
                for j in range(i+1, len(sel)):
                    r, ex = pair_report(sel[i], sel[j], k); k += 1
                    if not ex:
                        if r == "COLLIDE": any_col = True
                        elif r == "NEAR": any_near = True
            overall = "COLLIDE" if any_col else ("NEAR" if any_near else "CLEAR")
            print(f"  Overall: {overall}")
            self.report({'INFO'}, "Printed pairwise report to console")
            return {'FINISHED'}

        # 단일 선택: 타겟 vs 전체 리포트 + 전체 판정
        target = context.active_object if context.active_object in sel else sel[0]
        print(f"[Inspect] target={target.name} | mode={mode} | thr={thr_mm:.2f} mm | mesh_eps={eps_um:.2f} µm | obstacle_mode={getattr(sc,'collision_obstacle_mode','BOTH')}")
        any_col = False; any_near = False

        for r in robot:
            if r == target: continue
            rres, rex = pair_report(target, r)
            if not rex:
                if rres == "COLLIDE": any_col = True
                elif rres == "NEAR": any_near = True

        for o in obs:
            if o == target: continue
            ores, oex = pair_report(target, o)
            if not oex:
                if ores == "COLLIDE": any_col = True
                elif ores == "NEAR": any_near = True

        overall = "COLLIDE" if any_col else ("NEAR" if any_near else "CLEAR")
        print(f"  Overall: {overall}")
        self.report({'INFO'}, "Printed to console")
        return {'FINISHED'}

class RTEACH_OT_collision_owner_set_active(bpy.types.Operator):
    bl_idname = "rteach.collision_owner_set_active"
    bl_label = "Set Owner = Active"
    bl_options = {'REGISTER'}
    def execute(self, context):
        act = context.active_object
        if not act or act.type != 'MESH':
            self.report({'WARNING'}, "Active must be a mesh"); return {'CANCELLED'}
        for o in context.selected_objects:
            if o and o.type == 'MESH' and o != act:
                o.collision_grab_by_obj = act
                o.collision_is_grabbed = False
        _refresh_tint_now(recompute=True)
        return {'FINISHED'}

class RTEACH_OT_collision_owner_clear(bpy.types.Operator):
    bl_idname = "rteach.collision_owner_clear"
    bl_label = "Clear Owner"
    bl_options = {'REGISTER'}
    def execute(self, context):
        for o in context.selected_objects:
            if o and o.type == 'MESH':
                o.collision_grab_by_obj = None
        _refresh_tint_now(recompute=True)
        return {'FINISHED'}

class RTEACH_OT_collision_grab_pick_robot(bpy.types.Operator):
    bl_idname = "rteach.collision_grab_pick_robot"
    bl_label = "Pick by Robot"
    bl_options = {'REGISTER'}
    def execute(self, context):
        for o in context.selected_objects:
            if o and o.type == 'MESH':
                o.collision_grab_by_obj = None
                o.collision_is_grabbed = True
        _refresh_tint_now(recompute=True)
        return {'FINISHED'}

class RTEACH_OT_collision_grab_place_active(bpy.types.Operator):
    bl_idname = "rteach.collision_grab_place_active"
    bl_label = "Place to Active"
    bl_options = {'REGISTER'}
    def execute(self, context):
        act = context.active_object
        if not act or act.type != 'MESH':
            self.report({'WARNING'}, "Active must be a mesh"); return {'CANCELLED'}
        for o in context.selected_objects:
            if o and o.type == 'MESH' and o != act:
                o.collision_is_grabbed = False
                o.collision_grab_by_obj = act
        _refresh_tint_now(recompute=True)
        return {'FINISHED'}

class RTEACH_OT_collision_refresh_tint(bpy.types.Operator):
    bl_idname = "rteach.collision_refresh_tint"
    bl_label = ""
    bl_options = {'REGISTER'}
    def execute(self, context):
        _refresh_tint_now(recompute=True)
        return {'FINISHED'}

class RTEACH_OT_collision_tag_obstacle(bpy.types.Operator):
    bl_idname = "rteach.collision_tag_obstacle"
    bl_label = "Obstacle"
    set_flag: bpy.props.BoolProperty(default=True)
    def execute(self, context):
        sc = context.scene
        col = getattr(sc, "collision_obstacle_collection", None)
        for o in context.selected_objects:
            if not o or o.type != 'MESH':
                continue
            o.collision_is_obstacle = self.set_flag
            if col:
                if self.set_flag:
                    try:
                        if o.name not in col.objects and o not in col.all_objects:
                            col.objects.link(o)
                    except:
                        pass
                else:
                    try:
                        if o.name in col.objects:
                            col.objects.unlink(o)
                    except:
                        pass
        _refresh_tint_now(recompute=True)
        return {'FINISHED'}

class RTEACH_OT_collision_tag_grab(bpy.types.Operator):
    bl_idname = "rteach.collision_tag_grab"
    bl_label = "Grab"
    set_flag: bpy.props.BoolProperty(default=True)
    def execute(self, context):
        for o in context.selected_objects:
            if o and o.type == 'MESH':
                o.collision_is_grabbed = self.set_flag
        _refresh_tint_now(recompute=True)
        return {'FINISHED'}

class RTEACH_OT_collision_preset(bpy.types.Operator):
    bl_idname = "rteach.collision_preset"
    bl_label = "Preset"
    bl_options = {'REGISTER'}
    preset: bpy.props.EnumProperty(items=[('FAST','Fast',''),('ACCURATE','Accurate','')], default='FAST')
    def execute(self, context):
        sc = context.scene
        if self.preset == 'FAST':
            sc.collision_detection_mode = 'AABB'
            sc.collision_interval_ms = 40
            sc.collision_safe_margin_mm = 5.0
            sc.collision_mesh_eps_um = 10.0
            sc.collision_exclude_adjacent = True
            sc.collision_exclude_span = 1
            sc.collision_exclude_same_link = True
            sc.collision_grab_auto = True
            sc.collision_grab_span = 1
        else:
            sc.collision_detection_mode = 'MESH'
            sc.collision_interval_ms = 60
            sc.collision_safe_margin_mm = 5.0
            sc.collision_mesh_eps_um = 10.0
            sc.collision_exclude_adjacent = True
            sc.collision_exclude_span = 1
            sc.collision_exclude_same_link = True
            sc.collision_grab_auto = True
            sc.collision_grab_span = 1
        _refresh_tint_now(recompute=True)
        return {'FINISHED'}

def _on_toggle_tint(self, context):
    if context.scene.collision_tint_enable:
        _refresh_tint_now(recompute=True)
    else:
        _restore_all_colors()
        _set_view_color_type('MATERIAL')

def _on_choose_armature(self, context):
    nm = context.scene.collision_armature_name
    p = getattr(context.scene, "ik_motion_props", None)
    if p and nm and nm in bpy.data.objects:
        p.armature = nm
    _refresh_tint_now(recompute=True)

def _on_change_obstacle_collection(self, context):
    _refresh_tint_now(recompute=True)

class VIEW3D_PT_rteach_collision(bpy.types.Panel):
    bl_label = "Collision"
    bl_idname = "VIEW3D_PT_rteach_collision"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Robot Sim'
    def draw(self, context):
        sc = context.scene
        col = self.layout.column(align=True)

        row = col.row(align=True)
        row.operator("rteach.collision_preset", text="Fast").preset = 'FAST'
        row.operator("rteach.collision_preset", text="Accurate").preset = 'ACCURATE'

        box = col.box(); box.label(text="Robot & Scene")
        row = box.row(align=True); row.prop(sc, "collision_armature_name", text="Armature")
        row = box.row(align=True); row.prop(sc, "collision_obstacle_collection", text="Obstacles")
        if hasattr(sc, "collision_obstacle_mode"):
            row = box.row(align=True); row.prop(sc, "collision_obstacle_mode", text="Obstacle Mode")

        box = col.box(); box.label(text="Detection")
        row = box.row(align=True); row.prop(sc, "collision_detection_mode", text="Mode")
        row = box.row(align=True); row.prop(sc, "collision_safe_margin_mm", text="Safe Margin (mm)")
        if sc.collision_detection_mode == 'MESH':
            row = box.row(align=True); row.prop(sc, "collision_mesh_eps_um", text="Mesh ε (µm)")
        row = box.row(align=True); row.prop(sc, "collision_check_obstacles_pairs", text="Obs–Obs")
        if hasattr(sc, "collision_obsobs_grab_exclude"):
            row = box.row(align=True); row.prop(sc, "collision_obsobs_grab_exclude", text="Obs–Obs Grab Exclude")

        box = col.box(); box.label(text="Display")
        r = box.row(align=True)
        r.prop(sc, "collision_tint_enable", text="Tint")
        r.operator("rteach.collision_refresh_tint", icon="FILE_REFRESH", text="")
        r = box.row(align=True); r.prop(sc, "collision_outline_enable", text="Outline")

        box = col.box(); box.label(text="Interaction")
        r = box.row(align=True); r.prop(sc, "collision_grab_auto", text="Auto Grab (Robot)")
        if hasattr(sc, "collision_grab_auto_object"):
            r = box.row(align=True); r.prop(sc, "collision_grab_auto_object", text="Auto Owner (Parent/ChildOf)")
        r = box.row(align=True); r.prop(sc, "collision_grab_span", text="Grab Span")
        r = box.row(align=True)
        r.operator("rteach.collision_grab_pick_robot", text="Pick by Robot")
        r.operator("rteach.collision_grab_place_active", text="Place to Active")
        r = box.row(align=True)
        r.operator("rteach.collision_owner_set_active", text="Set Owner=Active")
        r.operator("rteach.collision_owner_clear", text="Clear Owner")

        box = col.box(); box.label(text="Tags")
        r = box.row(align=True)
        r.label(text="Obstacle:")
        r.operator("rteach.collision_tag_obstacle", text="Set").set_flag = True
        r.operator("rteach.collision_tag_obstacle", text="Clear").set_flag = False
        r = box.row(align=True)
        r.label(text="Grab:")
        r.operator("rteach.collision_tag_grab", text="Set").set_flag = True
        r.operator("rteach.collision_tag_grab", text="Clear").set_flag = False

        sels = [o for o in bpy.context.selected_objects if o and o.type == 'MESH']
        if sels:
            for o in sels[:12]:
                owner, src = _grab_owner(o)
                owner_nm = owner.name if owner else "—"
                r = box.row(align=True)
                r.label(text=o.name, icon='MESH_CUBE')
                r.label(text=("Obstacle ✓" if getattr(o,"collision_is_obstacle",False) else "Obstacle —"),
                        icon=('CHECKBOX_HLT' if getattr(o,"collision_is_obstacle",False) else 'CHECKBOX_DEHLT'))
                r.label(text=("Grab✓(Robot)" if getattr(o,"collision_is_grabbed",False) else "Grab—"),
                        icon=('CHECKBOX_HLT' if getattr(o,"collision_is_grabbed",False) else 'CHECKBOX_DEHLT'))
                r.label(text=f"Owner: {owner_nm}")
            if len(sels) > 12:
                box.label(text=f"... and {len(sels)-12} more")

        box = col.box(); box.label(text="Performance")
        r = box.row(align=True)
        r.prop(sc, "collision_interval_ms", text="Interval (ms)")
        r.prop(sc, "collision_debug_profile", text="Profile")
        if hasattr(sc, "collision_debug_details"):
            r.prop(sc, "collision_debug_details", text="Details")

        box = col.box(); box.label(text="Run")
        r1 = box.row(align=True)
        if not _state["running"]:
            r1.operator("rteach.collision_start", icon="PLAY", text="Start Live")
        else:
            r1.operator("rteach.collision_stop", icon="PAUSE", text="Stop Live")
        r1.operator("rteach.collision_scan_motion", icon="SEQ_LUMA_WAVEFORM", text="Scan Keyframes")
        r1.operator("rteach.collision_clear_markers", icon="TRASH", text="Clear Marks")
        r2 = box.row(align=True)
        r2.operator("rteach.collision_scan_once", icon="VIEWZOOM", text="Quick Check")
        r2.operator("rteach.collision_probe_selected", icon="INFO", text="Inspect Selected")

        if sc.collision_debug_profile:
            col.label(text=f"Delay: {_state['last_ms_total']:.1f} ms (Comp {_state['last_ms_comp']:.1f} ms)")
        col.label(text=_state["last_status"] or "Ready")

def _armature_enum(self, context):
    items = []
    seen = set()
    scn = context.scene
    p = getattr(scn, "ik_motion_props", None)
    cols = set()
    base = getattr(p, "base_object", None) if p else None
    if base:
        for c in base.users_collection:
            cols.add(c)
    if cols:
        for c in list(cols):
            for o in c.all_objects:
                if o.type == 'ARMATURE' and o.name not in seen:
                    items.append((o.name, o.name, "")); seen.add(o.name)
    if not items:
        for o in bpy.data.objects:
            if o.type == 'ARMATURE' and o.name not in seen:
                items.append((o.name, o.name, "")); seen.add(o.name)
    items.sort(key=lambda x: x[1].lower())
    return items or [("","", "")]

def _register_object_props():
    ob = bpy.types.Object
    if not hasattr(ob, "collision_is_obstacle"):
        ob.collision_is_obstacle = bpy.props.BoolProperty(name="Obstacle", default=False)
    if not hasattr(ob, "collision_is_grabbed"):
        ob.collision_is_grabbed = bpy.props.BoolProperty(name="Grabbed", default=False)
    if not hasattr(ob, "collision_grab_by_obj"):
        ob.collision_grab_by_obj = bpy.props.PointerProperty(name="Grab Owner", type=bpy.types.Object)

def _register_props():
    sc = bpy.types.Scene
    if not hasattr(sc, "collision_armature_name"):
        sc.collision_armature_name = bpy.props.EnumProperty(name="Armature", items=_armature_enum, update=_on_choose_armature)
    if not hasattr(sc, "collision_obstacle_collection"):
        sc.collision_obstacle_collection = bpy.props.PointerProperty(type=bpy.types.Collection, update=_on_change_obstacle_collection)
    if not hasattr(sc, "collision_obstacle_mode"):
        sc.collision_obstacle_mode = bpy.props.EnumProperty(name="Obstacle Mode", items=[('BOTH','Both',''),('TAG_ONLY','Tag Only',''),('COLLECTION_ONLY','Collection Only','')], default='BOTH', update=_on_change_obstacle_collection)
    if not hasattr(sc, "collision_detection_mode"):
        sc.collision_detection_mode = bpy.props.EnumProperty(name="Mode", items=[('AABB','AABB',''),('MESH','MESH','')], default='MESH')
    if not hasattr(sc, "collision_safe_margin_mm"):
        sc.collision_safe_margin_mm = bpy.props.FloatProperty(name="Safe Margin (mm)", default=5.0, min=0.0, soft_max=50.0)
    if not hasattr(sc, "collision_mesh_eps_um"):
        sc.collision_mesh_eps_um = bpy.props.FloatProperty(name="Mesh ε (µm)", default=10.0, min=0.1, max=100.0, precision=2, soft_min=0.1, soft_max=100.0)
    if not hasattr(sc, "collision_check_obstacles_pairs"):
        sc.collision_check_obstacles_pairs = bpy.props.BoolProperty(name="Obs–Obs", default=True)
    if not hasattr(sc, "collision_obsobs_grab_exclude"):
        sc.collision_obsobs_grab_exclude = bpy.props.BoolProperty(name="Obs–Obs Grab Exclude", default=False)
    if not hasattr(sc, "collision_tint_enable"):
        sc.collision_tint_enable = bpy.props.BoolProperty(name="Tint", default=True, update=_on_toggle_tint)
    if not hasattr(sc, "collision_outline_enable"):
        sc.collision_outline_enable = bpy.props.BoolProperty(name="Outline", default=True)
    if not hasattr(sc, "collision_grab_auto"):
        sc.collision_grab_auto = bpy.props.BoolProperty(name="Auto Grab", default=True)
    if not hasattr(sc, "collision_grab_span"):
        sc.collision_grab_span = bpy.props.IntProperty(name="Grab Span", default=1, min=0, max=3)
    if not hasattr(sc, "collision_interval_ms"):
        sc.collision_interval_ms = bpy.props.IntProperty(name="Interval (ms)", default=40, min=5, max=200)
    if not hasattr(sc, "collision_debug_profile"):
        sc.collision_debug_profile = bpy.props.BoolProperty(name="Profile", default=False)
    if not hasattr(sc, "collision_scan_substeps"):
        sc.collision_scan_substeps = bpy.props.IntProperty(name="Scan Substeps", default=0, min=0, max=5)
    if not hasattr(sc, "collision_scan_draw_sec"):
        sc.collision_scan_draw_sec = bpy.props.FloatProperty(name="Scan Draw (s)", default=2.0, min=0.2, max=10.0)
    if not hasattr(sc, "collision_exclude_adjacent"):
        sc.collision_exclude_adjacent = bpy.props.BoolProperty(name="Adj", default=True)
    if not hasattr(sc, "collision_exclude_span"):
        sc.collision_exclude_span = bpy.props.IntProperty(name="Span", default=1, min=0, max=3)
    if not hasattr(sc, "collision_exclude_same_link"):
        sc.collision_exclude_same_link = bpy.props.BoolProperty(name="Same", default=True)
    if not hasattr(sc, "collision_debug_details"):
        sc.collision_debug_details = bpy.props.BoolProperty(name="Details", default=False)
    if not hasattr(sc, "collision_grab_auto_object"):
        sc.collision_grab_auto_object = bpy.props.BoolProperty(name="Auto Owner", default=True)

def register():
    for c in (
        RTEACH_OT_collision_start, RTEACH_OT_collision_stop, RTEACH_OT_collision_scan_once,
        RTEACH_OT_collision_scan_motion, RTEACH_OT_collision_clear_markers,
        RTEACH_OT_collision_probe_selected, RTEACH_OT_collision_refresh_tint,
        RTEACH_OT_collision_tag_obstacle, RTEACH_OT_collision_tag_grab,
        RTEACH_OT_collision_preset,
        RTEACH_OT_collision_owner_set_active, RTEACH_OT_collision_owner_clear,
        RTEACH_OT_collision_grab_pick_robot, RTEACH_OT_collision_grab_place_active,
        VIEW3D_PT_rteach_collision
    ):
        bpy.utils.register_class(c)
    _register_object_props()
    _register_props()

def unregister():
    for c in (
        VIEW3D_PT_rteach_collision,
        RTEACH_OT_collision_grab_place_active, RTEACH_OT_collision_grab_pick_robot,
        RTEACH_OT_collision_owner_clear, RTEACH_OT_collision_owner_set_active,
        RTEACH_OT_collision_preset,
        RTEACH_OT_collision_tag_grab, RTEACH_OT_collision_tag_obstacle,
        RTEACH_OT_collision_refresh_tint, RTEACH_OT_collision_probe_selected,
        RTEACH_OT_collision_clear_markers, RTEACH_OT_collision_scan_motion,
        RTEACH_OT_collision_scan_once, RTEACH_OT_collision_stop, RTEACH_OT_collision_start
    ):
        try: bpy.utils.unregister_class(c)
        except: pass
    try: bpy.app.handlers.depsgraph_update_post.remove(_depsgraph_listener)
    except: pass
    if _state["h_view"]:
        try: bpy.types.SpaceView3D.draw_handler_remove(_state["h_view"], 'WINDOW')
        except: pass
        _state["h_view"] = None
    _restore_all_colors()
    _set_view_color_type('MATERIAL')
