import bpy
from math import *
from mathutils import *
from . import vector_utils as VU
import scipy.interpolate as inter
from numpy import *
#import scipy.misc.derivative as derivative

duration = 0
N = 0
def get_region3d(c):
    return get_area3d(c).spaces[0].region_3d

#Metodo che prende l'area di Blender dove è contenuta la vista
def get_area3d(c):
    areas = c.screen.areas
    for a in areas:
        if a.type == "VIEW_3D":
            okarea = a
    return okarea

#Metodo che restituisce True se il punto p è all'interno dell'oggetto obj
def point_inside(p, obj):
    mat = obj.matrix_world.copy()
    mat.invert()
    orig = mat*p
    result, point, normal, index = obj.closest_point_on_mesh(orig)
    p2 = point-orig
    v = p2.dot(normal)
    #print("Nome: "+obj.name+" - Valore:"+str(v))
    return not(v < 0.0)

def point_boundbox(pointer, obj_selected):
    bbox = obj_selected.bound_box #bpy.context.selected_objects[0].bound_box
    xmin = min(pos[0] for pos in bbox)
    ymin = min(pos[1] for pos in bbox)
    zmin = min(pos[2] for pos in bbox)
    xmax = max(pos[0] for pos in bbox)
    ymax = max(pos[1] for pos in bbox)
    zmax = max(pos[2] for pos in bbox)

    dx = xmax - xmin
    dy = ymax - ymin
    dz = zmax - zmin

    def bar(p, d, t, m):
        return (p + 1) / 2.0 * d * (1 + t * 2) + m - d * t

    t = 0.8
    x_ = bar(pointer.x, dx, t, xmin)
    y_ = bar(pointer.y, dy, t, ymin)
    z_ = bar(pointer.z, dz, t, zmin)
    z_ -= dz * 0.42

    #print("point boundbox",x_, y_, z_)
    return x_, y_, z_

#Metodo per costruire una superficie di skin sulle curve spls
def skinning_curves(ctx,spls):
    ts = [t for t,c,k in spls]
    ks = [k for t,c,k in spls]
    n_curves = len(spls)
    k = ks[0]
    for j in ks:
        if j != k:
            return False
    knot_set = set()
    for t in ts:
        knot_set = knot_set.union(t)
    nurbs_surface = bpy.data.curves.new(name="leap_surface_nurbs",type="SURFACE")
    nurbs_surface.dimensions = "3D"
    object_surf_data = bpy.data.objects.new("leap_surface_nurbs_object",nurbs_surface)
    object_surf_data.location = (0,0,0)
    ctx.scene.objects.link(object_surf_data)

    for i in range(n_curves):
        knots = set(ts[i])
        tck = spls[i]
        nP = len(tck[1][0])
        knot_diff = knot_set - knots
        for new_k in knot_diff:
            tck = inter.insert(new_k,tck)
            nP +=1
        
        cX,cY,cZ = tck[1]
        cX = cX[:nP]
        cY = cY[:nP]
        nurbs_curve = bpy.data.curves.new(name="leap_curve_nurbs",type="CURVE")
        nurbs_curve.dimensions = "2D"
        object_curve_data = bpy.data.objects.new("leap_curve_nurbs_object",nurbs_curve)
        object_curve_data.location = (0,0,0.5*i)
        ctx.scene.objects.link(object_curve_data)

        polyline_curve = nurbs_curve.splines.new("NURBS")
        polyline_curve.points.add(nP-1)
        for p,x,y in zip(polyline_curve.points,cX,cY):
            p.co = (x,y,0.0,1.0)
        polyline_curve.order_u = k+1

        polyline_surf = nurbs_surface.splines.new("NURBS")
        polyline_surf.points.add(nP-1)

        for p,x,y in zip(polyline_surf.points,cX,cY):
            p.co = Vector((x,y,0.5*i,1.0))
            p.select = True
        polyline_surf.order_u = k+1
    ctx.scene.objects.active = object_surf_data
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.curve.make_segment()
    bpy.ops.object.mode_set(mode='OBJECT')
    nurbs_surface.splines[0].order_u = nurbs_surface.splines[0].order_u

#Metodo per costruire una superficie di swung sulla curva traiettoria path e la curva profilo profile
def swinging_curves(ctx,path,profile):
    cX_path = path[1][0]
    cY_path = path[1][1]
    kpa = path[2]
    path_n_points = len(cX_path)
    cX_profile = profile[1][0]
    cZ_profile = profile[1][2]
    kpr = profile[2]

    profile_n_points = len(cX_profile) 

    nurbs_surface = bpy.data.curves.new(name="leap_surface_nurbs",type="SURFACE")
    nurbs_surface.dimensions = "3D"
    object_surf_data = bpy.data.objects.new("leap_surface_nurbs_object",nurbs_surface)
    object_surf_data.location = (0,0,0)
    ctx.scene.objects.link(object_surf_data)
    
    #Aggiungo curva traiettoria smooth
    nurbs_curve = bpy.data.curves.new(name="leap_curve_nurbs",type="CURVE")
    nurbs_curve.dimensions = "3D"
    object_curve_data = bpy.data.objects.new("leap_curve_nurbs_object",nurbs_curve)
    object_curve_data.location = (0,0,0)
    ctx.scene.objects.link(object_curve_data)

    polyline_curve = nurbs_curve.splines.new("NURBS")
    polyline_curve.points.add(path_n_points-1)
    for p,x,y in zip(polyline_curve.points,cX_path,cY_path):
        p.co = (x,y,0.0,1.0)
    polyline_curve.order_u = kpa+1

    #Aggiungo curva profilo smooth
    nurbs_curve = bpy.data.curves.new(name="leap_curve_nurbs",type="CURVE")
    nurbs_curve.dimensions = "3D"
    object_curve_data = bpy.data.objects.new("leap_curve_nurbs_object",nurbs_curve)
    object_curve_data.location = (0,0,0)
    ctx.scene.objects.link(object_curve_data)

    polyline_curve = nurbs_curve.splines.new("NURBS")
    polyline_curve.points.add(profile_n_points-1)
    for p,x,z in zip(polyline_curve.points,cX_profile,cZ_profile):
        p.co = (x,0.0,z,1.0)
    polyline_curve.order_u = kpr+1

    #Aggiungo superficie swinging
    for i in range(0,profile_n_points):
        polyline_surf = nurbs_surface.splines.new("NURBS")
        polyline_surf.points.add(path_n_points-1)
  
        xprof = cX_profile[i]+1
        zprof = cZ_profile[i]

        for p,xpath,ypath in zip(polyline_surf.points,cX_path,cY_path):
            xpath += 1
            ypath += 1
            alpha = 1
            p.co = Vector(((alpha*xprof*xpath),(alpha*xprof*ypath),zprof,1.0))
            p.select = True
        polyline_surf.use_cyclic_u = ctx.scene.close_curve_property
        polyline_surf.order_u = kpa+1
        polyline_surf.order_v = kpr+1

    ctx.scene.objects.active = object_surf_data
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.curve.make_segment()
    bpy.ops.object.mode_set(mode='OBJECT')
    nurbs_surface.splines[0].order_u = nurbs_surface.splines[0].order_u

def animation_path (ctx, path, vel, pos, dirct, obj_selected, cpoints, seconds, time, points_position,u,name_prev_curve):
    cX_path = path[1][0]
    cY_path = path[1][1]
    cZ_path = path[1][2]

    kpa = path[2]
    path_n_points = len(cX_path)
    nurbs_curve = None
    object_curve_data = None 
    
    for i in obj_selected:
        if i.name == name_prev_curve: 
            object_curve_data = i

    if nurbs_curve == None and object_curve_data == None:
        nurbs_curve = bpy.data.curves.new(name="leap_curve",type="CURVE")
        nurbs_curve.dimensions = "3D"
        nurbs_curve.use_path = True

        object_curve_data = bpy.data.objects.new("leap_curve",nurbs_curve)
        object_curve_data.location = (0,0,0)
        ctx.scene.objects.link(object_curve_data)

        polyline_curve = nurbs_curve.splines.new("NURBS")
        polyline_curve.points.add(path_n_points-1)
        for p,x,y,z in zip(polyline_curve.points,cX_path,cY_path,cZ_path):
            p.co = (x,y,z,1.0)
        polyline_curve.order_u = kpa+1

        name_prev_curve = object_curve_data.name

    vel = calculate_norma(vel) #normalizzazione del vettore velocità
    v_sum = 0.0
    
    for i in vel:
        v_sum += i

    vel_med = v_sum/len(vel)
    duration = ctx.scene['path_length']*30

    if ctx.scene['time_move']:  #controllo se True, prendo il valore dei secondi per la durata del percorso
        object_curve_data.data.path_duration = seconds*30 
        ctx.scene.frame_end = (seconds*30)+int(ctx.scene['fstart'])
    else:
        object_curve_data.data.path_duration = duration
        ctx.scene.frame_end = duration+int(ctx.scene['fstart'])

    print("Velelocità media:", vel_med)
    print("Lunghezza Array Velocità:", len(vel))
    print("Secondi:", seconds)
    print("Path Length Disegno:", seconds*30)
    print("Lunghezza Della Curva (Numero punti):", cpoints)
    print("Lunghezza dell'animazione:", object_curve_data.data.path_duration)

    insertKeyFrames(ctx, object_curve_data.data.path_duration, vel, pos, dirct, obj_selected, object_curve_data, cpoints, time, seconds, points_position, path,u)

    return name_prev_curve

def calculate_norma(vel):
    M2 = []
    for i in vel:
        M2.append(sqrt((i.x)**2+(i.y)**2+(i.z)**2))
    return M2

def insertKeyFrames(ctx, duration, vel, pos, dirct, obj_selected, curve, cpoints, time, seconds, points_position, path,u):
    N = len(vel)-1
    framesPerPoint = (duration / (cpoints-1))
    path_orientation = int(ctx.scene['object_move'])
    ctx.scene.frame_start = int(ctx.scene['fstart']) 
    ctx.scene.frame_current = int(ctx.scene['fstart'])
    vel_max = 0 
    V = []
    S = []
    offset = []
    look_at = []
    v_sum = 0.0
    timeframe = []
    view = get_region3d(bpy.context)
    mesh = []
    camera = None
    curve.data.use_path = False
    obj_in_use = False
    
    for i in obj_selected:
        flag = False
        empty_obj = None
        if i.type == 'MESH': 
            for j in ctx.scene.objects:
                if j.name == ("empty_"+i.name) and path_orientation == 3:
                    flag = True
                    obj_in_use = True
                if j.name == ("empty_"+i.name) and flag == False: 
                    bpy.ops.object.select_all(action='DESELECT')
                    j.select = True
                    bpy.ops.object.delete() 
                    flag = False
            if flag == False:
                empty_obj = bpy.data.objects.new( "empty_"+i.name, None )
                ctx.scene.objects.link(empty_obj)
            copyLocConstraints = [ c for c in i.constraints if c.type == 'COPY_TRANSFORMS']
            if flag == False:
                for c in copyLocConstraints:
                    i.constraints.remove( c ) # Remove constraint
                    print("Constraint Mesh REMOVE")
            mesh.append(i)

        if i.type == 'CAMERA': 
            for j in ctx.scene.objects:
                if j.name == ("empty_"+i.name) and path_orientation == 3:
                    flag = True
                    obj_in_use = True
                if j.name == ("empty_"+i.name) and flag == False:
                    bpy.ops.object.select_all(action='DESELECT')
                    j.select = True
                    bpy.ops.object.delete() 
                    flag = False
            if flag == False:
                empty_obj = bpy.data.objects.new( "empty_"+i.name, None )
                ctx.scene.objects.link(empty_obj)
            copyLocConstraints = [ c for c in i.constraints if c.type == 'COPY_TRANSFORMS']
            if flag == False:
                for c in copyLocConstraints:
                    i.constraints.remove( c ) # Remove constraint
                    print("Constraint Camera REMOVE")
            camera = i

    #calcolo della velocità massima, velocità media, dello spazio totale percorso e lo quello di ogni 10 frame
    for i in vel:
        if i>vel_max:
            vel_max = i
        v_sum += i
        
    vel_med = v_sum/len(vel)
    ml_space = seconds*vel_med 

    space = 0.0
    for i in range(0,N):
        if i == 0:
            t = time[i]
        else:
            t = time[i] - time[i-1]
        timeframe.append(t)
        r = abs(t) * abs(vel[i])
        S.append(r)
        space += r 
    #prendo l'ultima porzione di spazio con l'ultima velocità rilevata ed il tempo 
        if i == (N-1):
            t = seconds - time[i]
            r = abs(t) * abs(vel[N])
            space += r 

    for i in range(0,N):  
        l = S[i]/space
        V.append(l)  

    #l'offset in un punto è ottenuto sommando l'offset in quel punto più l'offset del punto precedente
    for i in range(0,N):
        if i == 0:
            v = V[i]
        else:
            v = (V[i] + offset[i-1])
        offset.append(v)

    print("Frames per Point:",framesPerPoint)
    print("Velocità max:",vel_max)
    print("Spazio percorso Totale (sec*velmed):", ml_space)
    print("Spazio percorso Totale:", space)
    print("Tempo registrato:",time)
    print("Tempo ogni 10 frame/segmenti:",timeframe)
    print("Spazio percorso ogni 10 frame:",S) 
    print("Offset ogni 10 frame:",V) 


    #calcolo per l'orientamento dell'oggetto CENTER OF INTEREST
    if path_orientation == 3 and camera != None and mesh != None:
        print("Center of Interest")

        object_1 = None
        object_2 = None
        if camera == None:
            object_1 = mesh[0]
            object_2 = mesh[1]
        else:
            object_1 = mesh[0]
            object_2 = camera
        #aggiungo il constraint follow path al secondo oggetto 
        object_data = bpy.data.objects['empty_'+object_2.name]
        object_data.location = Vector((0.0,0.0,0.0))
        ctx.scene.objects.active = object_data
        objs = object_data.constraints.new(type="FOLLOW_PATH") 
        objs.name = "leap_animation"
        objs.target = curve 
        objs.use_fixed_location = True
        objs.use_curve_follow = True
        objs.forward_axis = "TRACK_NEGATIVE_Z"
        objs.up_axis = "UP_Y"
        overrider = {}
        for window in ctx.window_manager.windows:
                screen = window.screen 
                for area in screen.areas:
                    if area.type == 'VIEW_3D':
                        for region in area.regions:
                            if region.type == 'WINDOW':
                                overrider =  {'window': window, 'screen': screen, 
                                                'area': area, 'region': region, 
                                                'scene': ctx.scene,
                                                'constraint':object_data.constraints["leap_animation"],
                                                'object':object_data}
                                break
        bpy.ops.constraint.followpath_path_animate(overrider,constraint='leap_animation')

        if not ctx.scene['constant_vel']:
            #aggiungo i keyframes per la velocità
            for pIndex in range(0, len(offset)): 
                frameNumber = int(framesPerPoint * (points_position[pIndex]))+int(ctx.scene['fstart'])      
                objs.offset_factor = offset[pIndex]
                print("")
                print("frame number:",frameNumber)
                print("")
                print("Offset value:",objs.offset_factor)
                object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=frameNumber)
                print("pIndex:",pIndex)


        objs.offset_factor = 0.0
        object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=int(ctx.scene['fstart']))
        objs.offset_factor = 1.0
        object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=duration+int(ctx.scene['fstart']))

        #aggiungo il constraint track to per il focus sull'oggetto
        objs = object_data.constraints.new(type="TRACK_TO") 
        objs.name = "leap_coi"
        objs.target = bpy.data.objects['empty_'+object_1.name]
        objs.track_axis = "TRACK_NEGATIVE_Z"
        objs.up_axis= "UP_Y"

        #aggiungo il constraint Copy Transformation all'oggetto empty
        if obj_in_use == False:
            emp = bpy.data.objects['empty_'+object_1.name]
            emp.location = object_1.location
            emp.scale = object_1.scale
            emp.rotation_euler = object_1.rotation_euler
        object_data = object_1
        obj = object_data.constraints.new(type='COPY_TRANSFORMS')
        obj.target = bpy.data.objects['empty_'+object_1.name]

        #aggiungo il constraint Copy Transformation all'oggetto empty
        object_data = object_2
        obj = object_data.constraints.new(type='COPY_TRANSFORMS')
        obj.target = bpy.data.objects['empty_'+object_2.name]
    
    #calcolo della terna(w,u,v) per l'orientamento dell'oggetto OBJECT FOLLOWING PATH
    elif path_orientation == 1 and (camera != None or mesh != None):
        for i in range(0,len(dirct)): 
            a = dirct[i]
            look_at.append(a)#(VU.quaternion_obj_rotation(a.x,a.y,a.z))

        object_active = None
        if camera == None:
            object_active = mesh[0]
        else:
            object_active = camera

        #aggiungo il constraint follow path
        object_data = bpy.data.objects['empty_'+object_active.name]
        object_data.location = Vector((0.0,0.0,0.0))
        object_data.rotation_mode = 'QUATERNION'
        ctx.scene.objects.active = object_data
        objs = object_data.constraints.new(type="FOLLOW_PATH") 
        objs.name = "leap_animation"
        objs.target = curve
        objs.use_fixed_location = True
        objs.forward_axis = "TRACK_NEGATIVE_Z"
        objs.up_axis = "UP_Y"
        objs.use_curve_follow = True
        overrider = {}
        for window in ctx.window_manager.windows:
                screen = window.screen 
                for area in screen.areas:
                    if area.type == 'VIEW_3D':
                        for region in area.regions:
                            if region.type == 'WINDOW':
                                overrider =  {'window': window, 'screen': screen, 
                                                'area': area, 'region': region, 
                                                'scene': ctx.scene,
                                                'constraint':object_data.constraints["leap_animation"],
                                                'object':object_data}
                                break
        bpy.ops.constraint.followpath_path_animate(overrider,constraint='leap_animation')

        #aggiungo i keyframes per la velocità e l'orientamento
        for pIndex in range(0, len(offset)): 
            frameNumber = int(framesPerPoint * (points_position[pIndex]))+int(ctx.scene['fstart'])       
            object_data.rotation_quaternion = look_at[pIndex] 

            if not ctx.scene['constant_vel']:
                objs.offset_factor = offset[pIndex]
                print("")
                print("frame number:",frameNumber)
                print("")
                print("Offset value:",objs.offset_factor)
                object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=frameNumber)

            object_data.keyframe_insert(data_path='rotation_quaternion', frame=frameNumber)

        objs.offset_factor = 0.0
        object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=int(ctx.scene['fstart']))

        objs.offset_factor = 1.0
        object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=duration+int(ctx.scene['fstart']))

        #aggiungo il constraint Copy Transformation all'oggetto 
        object_data = object_active
        obj = object_data.constraints.new(type='COPY_TRANSFORMS')
        obj.target = bpy.data.objects['empty_'+object_active.name]

    #calcolo della terna(w,u,v) per l'orientamento FOLLOW OBJECT
    elif path_orientation == 2 and mesh != None and camera != None:
        print("Follow Object")
        offset_distance = ctx.scene['offset_obj']
        first_deriv = VU.calculate_derivate(u,path,1)
        second_deriv = VU.calculate_derivate(u,path,2)
        counter = 0
        for i in range(0,len(first_deriv)): 
            if counter != (len(points_position)):
                if i == (points_position[counter]-1):
                    w = Vector(first_deriv[i])
                    u = w.cross(Vector(second_deriv[i]))
                    v = u.cross(w)

                    a = linalg.norm(w) #sqrt((w.x)**2+(w.y)**2+(w.z)**2)#/40
                    b = linalg.norm(u) #sqrt((u.x)**2+(u.y)**2+(u.z)**2)#/40
                    c = linalg.norm(v) #sqrt((v.x)**2+(v.y)**2+(v.z)**2)#/40

                    counter += 1
                    look_at.append(VU.quaternion_obj_rotation(a,b,c))
            else:
                break
        
        object_1 = None
        object_2 = None
        if camera == None:
            object_1 = mesh[0]
            object_2 = mesh[1]
        else:
            object_1 = mesh[0]
            object_2 = camera
        #aggiungo il constraint follow path alla camera
        object_data = bpy.data.objects['empty_'+object_2.name]
        object_data.location = Vector((0.0,0.0,0.0))
        emp = bpy.data.objects['empty_'+object_2.name]
        emp.scale = object_2.scale
        ctx.scene.objects.active = object_data
        objs = object_data.constraints.new(type="FOLLOW_PATH") 
        objs.name = "leap_animation"
        objs.target = curve 
        objs.use_fixed_location = True
        objs.use_curve_follow = True
        objs.forward_axis = "TRACK_NEGATIVE_Z"
        objs.up_axis = "UP_Y"
        overrider = {}
        for window in ctx.window_manager.windows:
                screen = window.screen 
                for area in screen.areas:
                    if area.type == 'VIEW_3D':
                        for region in area.regions:
                            if region.type == 'WINDOW':
                                overrider =  {'window': window, 'screen': screen, 
                                                'area': area, 'region': region, 
                                                'scene': ctx.scene,
                                                'constraint':object_data.constraints["leap_animation"],
                                                'object':object_data}
                                break
        bpy.ops.constraint.followpath_path_animate(overrider,constraint='leap_animation')

        if not ctx.scene['constant_vel']:
            #aggiungo i keyframes per la velocità alla camera
            for pIndex in range(0, N): 
                frameNumber = int(framesPerPoint * (points_position[pIndex]))+int(ctx.scene['fstart'])        
                objs.offset_factor = offset[pIndex] - (offset_distance/2)
                print("")
                print("frame number:",frameNumber)
                print("")
                print("Offset value:",objs.offset_factor)
                object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=frameNumber)
                print("pIndex:",pIndex)


        objs.offset_factor = 0.0
        object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=int(ctx.scene['fstart']))
        objs.offset_factor = 1.0 - (offset_distance/2)
        object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=duration+int(ctx.scene['fstart']))

        #aggiungo il constraint track to alla camera per il focus sull'oggetto
        objs = object_data.constraints.new(type="TRACK_TO") 
        objs.name = "leap_coi"
        objs.target = bpy.data.objects['empty_'+object_1.name] 
        objs.track_axis = "TRACK_NEGATIVE_Z"
        objs.up_axis= "UP_Y"

        #aggiungo il constraint follow path all'altro oggetto
        object_data = bpy.data.objects['empty_'+object_1.name] 
        object_data.location = Vector((0.0,0.0,0.0))
        object_data.rotation_mode = 'QUATERNION'
        ctx.scene.objects.active = object_data
        objs = object_data.constraints.new(type="FOLLOW_PATH") 
        objs.name = "leap_animation"
        objs.target = curve 
        objs.use_fixed_location = True
        objs.use_curve_follow = True
        objs.forward_axis = "TRACK_NEGATIVE_Z"
        objs.up_axis = "UP_Y"
        overrider = {}
        for window in ctx.window_manager.windows:
                screen = window.screen 
                for area in screen.areas:
                    if area.type == 'VIEW_3D':
                        for region in area.regions:
                            if region.type == 'WINDOW':
                                overrider =  {'window': window, 'screen': screen, 
                                                'area': area, 'region': region, 
                                                'scene': ctx.scene,
                                                'constraint':object_data.constraints["leap_animation"],
                                                'object':object_data}
                                break
        bpy.ops.constraint.followpath_path_animate(overrider,constraint='leap_animation')

        #aggiungo i keyframes per la velocità e l'orientamento Frenet Frame
        for pIndex in range(0, N): 
            frameNumber = int(framesPerPoint * (points_position[pIndex]))+int(ctx.scene['fstart'])       
            object_data.rotation_quaternion = look_at[pIndex]

            if not ctx.scene['constant_vel']:
                objs.offset_factor = offset[pIndex] + (offset_distance/2)
                print("")
                print("frame number:",frameNumber)
                print("")
                print("Offset value:",objs.offset_factor)
                object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=frameNumber)

            object_data.keyframe_insert(data_path='rotation_quaternion', frame=frameNumber)

        objs.offset_factor = 0.0 + (offset_distance/2)
        object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=int(ctx.scene['fstart']))
        
        objs.offset_factor = 1.0
        object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=duration+int(ctx.scene['fstart']))

        #aggiungo il constraint Copy Transformation all'oggetto empty
        emp = bpy.data.objects['empty_'+object_1.name]
        emp.scale = object_1.scale
        emp.rotation_euler = object_1.rotation_euler
        object_data = object_1
        obj = object_data.constraints.new(type='COPY_TRANSFORMS')
        obj.target = bpy.data.objects['empty_'+object_1.name]

        object_data = object_2
        obj = object_data.constraints.new(type='COPY_TRANSFORMS')
        obj.target = bpy.data.objects['empty_'+object_2.name]

    #calcolo della terna(w,u,v) per l'orientamento FRENET FRAME
    elif path_orientation == 0:
        print("Frenet Frame")
        first_deriv = VU.calculate_derivate(u,path,1)
        second_deriv = VU.calculate_derivate(u,path,2)
        counter = 0
        first = Quaternion()
        last = Quaternion()
        for i in range(0,len(first_deriv)): 
            if counter != (len(points_position)):
                if i == (points_position[counter]-1):
                    w = Vector(first_deriv[i])
                    u = w.cross(Vector(second_deriv[i]))
                    v = u.cross(w)

                    a = linalg.norm(w) #sqrt((w.x)**2+(w.y)**2+(w.z)**2)#/40
                    b = linalg.norm(u) #sqrt((u.x)**2+(u.y)**2+(u.z)**2)#/40
                    c = linalg.norm(v) #sqrt((v.x)**2+(v.y)**2+(v.z)**2)#/40

                    counter += 1
                    look_at.append(VU.quaternion_obj_rotation(a,b,c))
            else:
                w = Vector(first_deriv[0])
                first = VU.quaternion_obj_rotation(w.x,w.y,w.z)
                w = Vector(first_deriv[len(first_deriv)-1])
                last = VU.quaternion_obj_rotation(w.x,w.y,w.z)
                break

        object_active = None
        if camera == None:
            object_active = mesh[0]
        else:
            object_active = camera

        #aggiungo il constraint follow path
        object_data = bpy.data.objects['empty_'+object_active.name] 
        object_data.location = Vector((0.0,0.0,0.0))
        object_data.rotation_mode = 'QUATERNION'
        ctx.scene.objects.active = object_data
        objs = object_data.constraints.new(type="FOLLOW_PATH") 
        objs.name = "leap_animation"
        objs.target = curve
        objs.use_fixed_location = True
        objs.forward_axis = "TRACK_NEGATIVE_Z"
        objs.up_axis = "UP_Y"
        objs.use_curve_follow = True
        overrider = {}
        for window in ctx.window_manager.windows:
                screen = window.screen 
                for area in screen.areas:
                    if area.type == 'VIEW_3D':
                        for region in area.regions:
                            if region.type == 'WINDOW':
                                overrider =  {'window': window, 'screen': screen, 
                                                'area': area, 'region': region, 
                                                'scene': ctx.scene,
                                                'constraint':object_data.constraints["leap_animation"],
                                                'object':object_data}
                                break
        bpy.ops.constraint.followpath_path_animate(overrider,constraint='leap_animation')

        #aggiungo i keyframes per la velocità e l'orientamento
        for pIndex in range(0, N): 
            frameNumber = int(framesPerPoint * (points_position[pIndex]))+int(ctx.scene['fstart'])       
            object_data.rotation_quaternion = look_at[pIndex]

            if not ctx.scene['constant_vel']:
                objs.offset_factor = offset[pIndex]
                print("")
                print("frame number:",frameNumber)
                print("")
                print("Offset value:",objs.offset_factor)
                object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=frameNumber)

            object_data.keyframe_insert(data_path='rotation_quaternion', frame=frameNumber)

        objs.offset_factor = 0.0
        object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=int(ctx.scene['fstart']))
        
        objs.offset_factor = 1.0
        object_data.keyframe_insert(data_path='constraints["leap_animation"].offset_factor', frame=duration+int(ctx.scene['fstart']))

        #aggiungo il constraint Copy Transformation all'oggetto empty
        emp = bpy.data.objects['empty_'+object_active.name]
        emp.scale = object_active.scale
        object_data = object_active
        obj = object_data.constraints.new(type='COPY_TRANSFORMS')
        obj.target = bpy.data.objects['empty_'+object_active.name]
