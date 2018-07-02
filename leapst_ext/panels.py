import bpy
from . import operators as OPS
from . import blender_utils as BU

class SmoothSelectPanel(bpy.types.Panel):
    bl_idname = "OBJECT_PT_interpolating"
    bl_label = "Interpolating"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOL_PROPS'

    bpy.types.Scene.smoothing_property = bpy.props.FloatProperty(name="Smoothing", default=0.05, min=0.0, max=100.0)
    bpy.types.Scene.order_property = bpy.props.IntProperty(name="Degree", default=2, min=2, max=5)
    bpy.types.Scene.close_curve_property = bpy.props.BoolProperty(name="Closed", default=False)

    @classmethod
    def poll(cls,context):
        obj = context.object
        return (obj is not None and obj.type == "CURVE"  and obj.data.splines[0].type == "POLY" and (obj.data.dimensions=="2D"))# or obj.data.dimensions=="3D"))

    def draw(self, context):
        layout = self.layout
        layout.prop(context.scene,"smoothing_property")
        layout.prop(context.scene,"order_property")
        layout.prop(context.scene,"close_curve_property")
        layout.operator(OPS.InterpolateOperator.bl_idname)


class ExtrudeSelectPanel(bpy.types.Panel):
    bl_idname = "OBJECT_PT_extrude"
    bl_label = "Extrude Curve"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOL_PROPS'

    bpy.types.Scene.tr_length_property = bpy.props.FloatProperty(name="Length", default=1.0, min=1.0, max=100.0)
    bpy.types.Scene.tr_subdivision_property = bpy.props.IntProperty(name="Subdivision", default=2, min=2, max=100)

    @classmethod
    def poll(cls,context):
        obj = context.object
        return (obj is not None and obj.type == "CURVE" and obj.data.splines[0].type == "NURBS")
            
    def draw(self, context):
        layout = self.layout
        layout.prop(context.scene,"tr_length_property")
        layout.prop(context.scene,"tr_subdivision_property")
        layout.operator(OPS.ExtrudeOperator.bl_idname)

class SpinSelectPanel(bpy.types.Panel):
    bl_idname = "OBJECT_PT_spin"
    bl_label = "Spin Curve"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOL_PROPS'

    bpy.types.Scene.sp_subdivision_property = bpy.props.IntProperty(name="Subdivision", default=4, min=4, max=100)

    @classmethod
    def poll(cls,context):
        obj = context.object
        return (obj is not None and obj.type == "CURVE" and obj.data.dimensions=="2D" and obj.data.splines[0].type == "NURBS")

    def draw(self, context):
        layout = self.layout
        layout.prop(context.scene,"sp_subdivision_property")
        layout.operator(OPS.SpinOperator.bl_idname)

class ExtrudePathSelectPanel(bpy.types.Panel):
    bl_idname = "OBJECT_PT_extrude_path"
    bl_label = "Extrude Curve on Path"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOL_PROPS'

    bpy.types.Scene.exp_object_property = bpy.props.StringProperty(name="Path curve")

    @classmethod
    def poll(cls,context):
        obj = context.object
        return (obj is not None and obj.type == "CURVE" and obj.data.splines[0].type == "NURBS")      

    def draw(self, context):
        layout = self.layout
        layout.prop_search(context.scene,"exp_object_property",context.scene,"objects", icon="CURVE_DATA")
        layout.operator(OPS.ExtrudePathOperator.bl_idname)

class SkinningSelectPanel(bpy.types.Panel):
    bl_idname = "OBJECT_PT_skinning"
    bl_label = "Skin curves"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOL_PROPS'

    @classmethod
    def poll(cls,context):
        objs = context.selected_objects
        if len(objs) < 4:
            return False
        for o in objs:
            if (o is None or o.type != "CURVE" or o.data.dimensions !="2D" or o.data.splines[0].type != "NURBS"):
                return False
        return True     

    def draw(self, context):
        layout = self.layout
        layout.operator(OPS.SkinningOperator.bl_idname)

class SwingingSelectPanel(bpy.types.Panel):
    bl_idname = "OBJECT_PT_swinging"
    bl_label = "Swinging curves"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOL_PROPS'

    bpy.types.Scene.swinP_object_property = bpy.props.StringProperty(name="P curve")
    bpy.types.Scene.swinT_object_property = bpy.props.StringProperty(name="T curve")


    @classmethod
    def poll(cls,context):
        objs = context.selected_objects
        if len(objs) < 2:
            return False
        for o in objs:
            if (o is None or o.type != "CURVE" or o.data.splines[0].type != "NURBS"):
                return False
        return True     

    def draw(self, context):
        layout = self.layout
        layout.prop_search(context.scene,"swinT_object_property",context.scene,"objects", icon="CURVE_DATA")
        layout.prop_search(context.scene,"swinP_object_property",context.scene,"objects", icon="CURVE_DATA")
        layout.operator(OPS.SwingingOperator.bl_idname)

class SmoothSelectPanel3D(bpy.types.Panel):
    bl_idname = "OBJECT_PT_interpolating_3D"
    bl_label = "Animation Tools"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'#_PROPS'

    @classmethod
    def poll(cls,context):
        obj = context.object
        return (obj is not None and obj.type == "CURVE"  and (obj.data.splines[0].type == "POLY" or obj.data.splines[0].type == "NURBS") and obj.data.dimensions=="3D")

    def draw(self, context):
        layout = self.layout
        snc = context.scene
        col1 = layout.row(align=False)
        col1.label("Approximation:")
        layout.prop(snc,"smoothing")
        layout.prop(snc,"order")
        col2 = layout.row(align=False)
        col2.label("Animation:")
        layout.prop(snc,"fstart")
        layout.prop(snc,"path_length")
        col = layout.row(align=False)
        col.label("Object Orientation:")
        layout.prop(snc, "object_move")
        if snc['object_move'] == 2:
            layout.prop(snc, "offset_obj")
        layout.prop(snc, "time_move")
        layout.prop(snc, "constant_vel")
        layout.prop(snc,"keep_data")

        
def init_ui_prop(snc, smoothing_default=3.0, order_default=3, path_length_default= 5, sample_default = 10, object_move_default=0, time_move_default = False, constant_vel_default = False, offset_obj_default = 0.1, offset_check_default = False, keep_data_default = False, fstart_default = 0):
    """
    Initializes properties for the UI panel settings, properties are
    contained in the Scene object
    """         
    bpy.types.Scene.smoothing = bpy.props.FloatProperty(
        name = "Smoothing",
        description = "Curve's Smoothing",
        min = 1,
        max = 30)
    
    bpy.types.Scene.order = bpy.props.IntProperty(
        name = "Degree",
        description = "Curve's Degree",
        min = 1,
        max = 10)
    
    bpy.types.Scene.path_length = bpy.props.IntProperty(
        name = "Animation Length",
        description = "Animation duration in seconds",
        min = 1,
        max = 100000)

    bpy.types.Scene.sample = bpy.props.IntProperty(
        name = "Forward Frames",
        description = "Number of Frames to look at",
        min = 1,
        max = 1000)

    bpy.types.Scene.object_move = bpy.props.EnumProperty(items= (('0', 'Frenet Frame', 'Orientamento Frenet Frame'),    
                                                 ('1', 'Object Following Path', 'Orientamento seguendo la direzione del dito rilevata'),    
                                                 ('2', 'Follow Object', 'Orientamento focalizzato su un oggetto lungo il percorso'),    
                                                 ('3', 'Center of Interest', 'Orientamento con un centro di interesse')),
                                                 name = "")
    bpy.types.Scene.time_move = bpy.props.BoolProperty(
        name="Leap seconds", 
        description = "Animation duration in seconds depends on drawing time",
        default=False)

    bpy.types.Scene.constant_vel = bpy.props.BoolProperty(
        name="Constant Velocity", 
        description = "The object has constant velocity",
        default=False)

    bpy.types.Scene.offset_obj = bpy.props.FloatProperty(
        name = "Distance",
        description = "Object distance from camera",
        min = 0,
        max = 1,
        default = 0.1)

    bpy.types.Scene.offset_check = bpy.props.BoolProperty(
        name="Object Distance", 
        description = "Turn on object distance setting",
        default=False)

    bpy.types.Scene.keep_data = bpy.props.BoolProperty(
        name="Keep Animation Data", 
        description = "Keep in memory Leap Motion Data",
        default=False)

    bpy.types.Scene.fstart = bpy.props.IntProperty(
        name = "Frame Start",
        description = "Frame Start",
        min = 0,
        max = 100000)

    snc['smoothing'] = smoothing_default
    snc['order'] = order_default
    snc['path_length'] = path_length_default
    snc['object_move'] = object_move_default
    snc['time_move'] = time_move_default
    snc['constant_vel'] = constant_vel_default
    snc['offset_obj'] = offset_obj_default
    snc['offset_check'] = offset_check_default
    snc['sample'] = sample_default
    snc['keep_data'] =keep_data_default
    snc['fstart'] =fstart_default
