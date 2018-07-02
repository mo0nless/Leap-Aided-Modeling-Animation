import bpy
import scipy.interpolate as inter
from math import *
from mathutils import *
from . import vector_utils as VU
from operator import attrgetter

class InterpolateOperator(bpy.types.Operator):
    """Interpolate Operator"""
    bl_idname = "object.interpolate_op"
    bl_label = "Interpolate curve"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = context.object
        if (obj is not None and obj.type == "CURVE" and obj.data.dimensions=="2D" and obj.data.splines[0].type == "POLY"):
            curve_data = obj.data
            data_points = []
            for s in curve_data.splines:
                for p in s.points:
                    if p.co not in data_points:
                        data_points.append(p.co)
            if len(data_points) > 5:
                coord_X = [p[0] for p in data_points]
                coord_Y = [p[1] for p in data_points]
                nurbs_curve = bpy.data.curves.new(name="leap_curve_nurbs",type="CURVE")
                nurbs_curve.dimensions = "2D"
                object_data = bpy.data.objects.new("leap_curve_nurbs_object",nurbs_curve)
                object_data.location = (0,0,0)
                bpy.context.scene.objects.link(object_data)

                polyline = nurbs_curve.splines.new("NURBS")
                interpolated = inter.splprep([coord_X,coord_Y],s=context.scene.smoothing_property,k=context.scene.order_property)
                cX = interpolated[0][1][0]
                cY = interpolated[0][1][1]
                n_points = len(cX)
                polyline.points.add(n_points-1)
                for i in range(n_points):
                    polyline.points[i].co = (cX[i],cY[i],0.0,1.0)
                polyline.order_u = context.scene.order_property+1
                polyline.use_cyclic_u = context.scene.close_curve_property
                obj.select = False
                context.scene.objects.active = object_data
                object_data.select = True
        else:
            self.report({'WARNING'},"Operation not allowed")
        return {"FINISHED"}

    def invoke(self, context,event):
        return self.execute(context)

class ExtrudeOperator(bpy.types.Operator):
    """Extrude Operator"""
    bl_idname = "object.extrude_op"
    bl_label = "Extrude curve"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        curve_object = context.object
        if (curve_object is not None and curve_object.type == "CURVE" and curve_object.data.splines[0].type == "NURBS"):
            tr_length = context.scene.tr_length_property
            subdivision = context.scene.tr_subdivision_property
            spline_diff = tr_length/(subdivision-1)
            curve_data = curve_object.data
            data_points = []
            for p in curve_data.splines[0].points:
                data_points.append(p.co)
            n_points = len(data_points)
            nurbs_surface = bpy.data.curves.new(name="leap_surface_nurbs",type="SURFACE")
            nurbs_surface.dimensions = "3D"
            object_data = bpy.data.objects.new("leap_surface_nurbs_object",nurbs_surface)
            object_data.location = context.object.location
            context.scene.objects.link(object_data)

            for i in range(subdivision):
                polyline = nurbs_surface.splines.new("NURBS")
                polyline.points.add(n_points-1)
                for p, new_p in zip(polyline.points,data_points):
                    p.co = Vector((new_p.x,new_p.y,new_p.z + spline_diff*i,1.0))
                    p.select = True
                polyline.order_u = curve_data.splines[0].order_u
                polyline.use_cyclic_u = curve_data.splines[0].use_cyclic_u
            curve_object.select = False
            context.scene.objects.active = object_data
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.curve.make_segment()
            bpy.ops.object.mode_set(mode='OBJECT')
            nurbs_surface.splines[0].order_u = nurbs_surface.splines[0].order_u
        else:
            self.report({'WARNING'},"Operation not allowed")
        return {"FINISHED"}

class SpinOperator(bpy.types.Operator):
    """Spin Operator"""
    bl_idname = "object.spin_op"
    bl_label = "Spin curve"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        curve_object = context.object
        if(curve_object is not None and curve_object.type == "CURVE" and curve_object.data.dimensions=="2D" and curve_object.data.splines[0].type == "NURBS"):
            subdivision = context.scene.sp_subdivision_property
            complete_spin = 2*pi
            spline_diff = complete_spin/(subdivision)
        
            prev_rotation = curve_object.rotation_euler
            curve_data = curve_object.data
            data_points = curve_data.splines[0].points
            dp_rot = VU.rotate_points(data_points,prev_rotation.to_quaternion(),Vector((0.0,0.0,0.0)))
            for p, new_p in zip(data_points,dp_rot):
                p.co = Vector((new_p.x,new_p.y,new_p.z,1.0))
            curve_object.rotation_euler = Euler((0.0,0.0,0.0),'XYZ')
            n_points = len(data_points)
            nurbs_surface = bpy.data.curves.new(name="leap_surface_nurbs",type="SURFACE")
            nurbs_surface.dimensions = "3D"
            object_data = bpy.data.objects.new("leap_surface_nurbs_object",nurbs_surface)
            object_data.location = curve_object.location
            object_data.scale = curve_object.scale
            context.scene.objects.link(object_data)

            for i in range(subdivision):
                polyline = nurbs_surface.splines.new("NURBS")
                polyline.points.add(n_points-1)
                q_rot = Euler((spline_diff*i,0.0,prev_rotation.z),'XYZ').to_quaternion()
                data_points_rotated_coord = VU.rotate_points(data_points,q_rot,-curve_object.location)
                for p, new_p in zip(polyline.points,data_points_rotated_coord):
                    p.co = Vector((new_p.x,new_p.y,new_p.z,1.0))
                    p.select = True
                polyline.order_u = curve_data.splines[0].order_u
                polyline.use_cyclic_u = curve_data.splines[0].use_cyclic_u
            curve_object.select = False
            context.scene.objects.active = object_data
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.curve.make_segment()
            bpy.ops.object.mode_set(mode='OBJECT')
            context.object.data.splines[0].use_cyclic_u = True
            object_data.select = True
        else:
            self.report({'WARNING'},"Operation not allowed")
        return {"FINISHED"}



class ExtrudePathOperator(bpy.types.Operator):
    """Extrude Path Operator"""
    bl_idname = "object.extrude_path_op"
    bl_label = "Extrude curve on path"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        curve_object = context.object
        if (curve_object is not None and curve_object.type == "CURVE" and curve_object.data.splines[0].type == "NURBS"):
            path_name = context.scene.exp_object_property
            path_obj = context.scene.objects.get(path_name)
            if path_obj is not None and path_obj.type == "CURVE":
                curve_data = curve_object.data
                path_data = path_obj.data
                prev_rotation = curve_object.rotation_euler
                data_points_prev = curve_data.splines[0].points
                dp_rot = VU.rotate_points(data_points_prev,prev_rotation.to_quaternion(),Vector((0.0,0.0,0.0)))
                for p, new_p in zip(data_points_prev,dp_rot):
                    p.co = Vector((new_p.x,new_p.y,new_p.z,1.0))
                curve_object.rotation_euler = Euler((0.0,0.0,0.0),'XYZ')
                data_points = []
                for p in curve_data.splines[0].points:
                    data_points.append(p.co)
                prev_rotation = path_obj.rotation_euler
                path_points_prev = path_data.splines[0].points
                dp_rot = VU.rotate_points(path_points_prev,prev_rotation.to_quaternion(),Vector((0.0,0.0,0.0)))
                for p, new_p in zip(path_points_prev,dp_rot):
                    p.co = Vector((new_p.x,new_p.y,new_p.z,1.0))
                path_obj.rotation_euler = Euler((0.0,0.0,0.0),'XYZ')
                path_points = []
                for p in path_data.splines[0].points:
                    if p not in path_points:
                        path_points.append(p.co)
                n_points_curve = len(data_points)
                n_points_path = len(path_points)
                nurbs_surface = bpy.data.curves.new(name="leap_surface_nurbs",type="SURFACE")
                nurbs_surface.dimensions = "3D"
                object_data = bpy.data.objects.new("leap_surface_nurbs_object",nurbs_surface)
                object_data.location = context.object.location
                context.scene.objects.link(object_data)

                surface_points = [data_points]
                for i in range(1,n_points_path):
                    diff = path_points[i]-path_points[i-1]
                    surface_points.append([])
                    for j in range(n_points_curve):
                        surface_points[i].append(surface_points[i-1][j]+diff)
                                  
                for i in range(n_points_path):
                    polyline = nurbs_surface.splines.new("NURBS")
                    polyline.points.add(n_points_curve-1)
                    for p, new_p in zip(polyline.points,surface_points[i]):
                        p.co = Vector((new_p.x,new_p.y,new_p.z,1.0))
                        p.select = True
                    polyline.order_u = curve_data.splines[0].order_u
                    polyline.use_cyclic_u = curve_data.splines[0].use_cyclic_u
                curve_object.select = False
                bpy.ops.object.mode_set(mode='EDIT')
                bpy.ops.curve.make_segment()
                bpy.ops.object.mode_set(mode='OBJECT')
                object_data.select = True
                nurbs_surface.splines[0].order_u = nurbs_surface.splines[0].order_u
            else:
                self.report({"WARNING"},"Wrong path")
        else:
            self.report({'WARNING'},"Operation not allowed")
        return {"FINISHED"}


class SkinningOperator(bpy.types.Operator):
    """Skinning Operator"""
    bl_idname = "object.skinning_op"
    bl_label = "Skin curves"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        curves_objects = context.selected_objects
        n_curves = len(curves_objects)
        if n_curves < 4:
            self.report({'WARNING'},"Too few curves selected")
            return {"FINISHED"}
        for o in curves_objects:
            if (o is None or o.type != "CURVE" or o.data.dimensions != "2D" or o.data.splines[0].type != "NURBS"):
                self.report({'WARNING'},"Selected objects are not curves")
                return {"FINISHED"}
        ordered_curves_objects = []
        for i in range(n_curves):
            min_z_curve = min(curves_objects,key=attrgetter("location.z"))
            ordered_curves_objects.append(min_z_curve)
            curves_objects.remove(min_z_curve)
        curves_datas = [o.data for o in ordered_curves_objects]
        n_points = len(curves_datas[0].splines[0].points)
        for i in range(1,n_curves):
            if len(curves_datas[i].splines[0].points) != n_points:
                self.report({'WARNING'},"Curves must have same number of control points")
                return {"FINISHED"}
        locs = [o.location for o in ordered_curves_objects]
        nurbs_surface = bpy.data.curves.new(name="leap_surface_nurbs",type="SURFACE")
        nurbs_surface.dimensions = "3D"
        object_data = bpy.data.objects.new("leap_surface_nurbs_object",nurbs_surface)
        object_data.location = (0,0,0)
        context.scene.objects.link(object_data)

        for i in range(n_curves):
            curve_to_skin = curves_datas[i]
            curve_loc = locs[i]
            data_points = curve_to_skin.splines[0].points
            polyline = nurbs_surface.splines.new("NURBS")
            polyline.points.add(n_points-1)
            for p, new_p in zip(polyline.points,data_points):
                d_co = new_p.co
                p.co = Vector((d_co.x+curve_loc.x,d_co.y+curve_loc.y,d_co.z+curve_loc.z,1.0))
                p.select = True
            polyline.order_u = curve_to_skin.splines[0].order_u
            polyline.use_cyclic_u = curve_to_skin.splines[0].use_cyclic_u
        for o in ordered_curves_objects:
            o.select = False
        context.scene.objects.active = object_data
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.curve.make_segment()
        bpy.ops.object.mode_set(mode='OBJECT')
        nurbs_surface.splines[0].order_u = nurbs_surface.splines[0].order_u
        return {"FINISHED"}



class SwingingOperator(bpy.types.Operator):
    """Swinging Operator"""
    bl_idname = "object.swinging_op"
    bl_label = "Swing curve"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        T_name = context.scene.swinT_object_property
        T_obj = context.scene.objects.get(T_name)
        P_name = context.scene.swinP_object_property
        P_obj = context.scene.objects.get(P_name)
        curves_objects = [T_obj,P_obj]
        n_curves = len(curves_objects)
        if n_curves < 2:
            self.report({'WARNING'},"Too few curves selected")
            return {"FINISHED"}
        for o in curves_objects:
            if (o is None or o.type != "CURVE" or o.data.splines[0].type != "NURBS"):
                self.report({'WARNING'},"Selected objects are not curves")
                return {"FINISHED"}
        curves_datas = [o.data for o in curves_objects]
        T_data = curves_datas[0]
        P_data = curves_datas[1]
        n_T_points = len(T_data.splines[0].points)
        n_P_points = len(P_data.splines[0].points)
        nurbs_surface = bpy.data.curves.new(name="leap_surface_nurbs",type="SURFACE")
        nurbs_surface.dimensions = "3D"
        object_data = bpy.data.objects.new("leap_surface_nurbs_object",nurbs_surface)
        object_data.location = (0,0,0)
        context.scene.objects.link(object_data)


        data_T_points = T_data.splines[0].points
        data_P_points = P_data.splines[0].points
        for i in range(0,n_P_points):
            polyline_surf = nurbs_surface.splines.new("NURBS")
            polyline_surf.points.add(n_T_points-1)
  
            xprof = data_P_points[i].co.x
            yprof = data_P_points[i].co.y
            zprof = data_P_points[i].co.z

            for p,new_p in zip(polyline_surf.points,data_T_points):
                alpha = 1
                p.co = Vector(((alpha*xprof*new_p.co.x),(alpha*xprof*new_p.co.y),zprof,1.0))
                p.select = True
            polyline_surf.use_cyclic_u = T_data.splines[0].use_cyclic_u
            polyline_surf.order_u = T_data.splines[0].order_u
            polyline_surf.order_v = T_data.splines[0].order_v

        context.scene.objects.active = object_data
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.curve.make_segment()
        bpy.ops.object.mode_set(mode='OBJECT')
        nurbs_surface.splines[0].order_u = nurbs_surface.splines[0].order_u
        return {"FINISHED"}

