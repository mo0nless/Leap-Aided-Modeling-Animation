bl_info = {
    "name": "Leap motion Ext",
    "category": "Object",
}
#if "bpy" in locals():
#    import importlib
#    importlib.reload(leapmotion)
#else:
#    from . import leapmotion

import bpy, bmesh
import math, mathutils, ctypes
from math import *
from mathutils import *
from . import vector_utils as VU
from . import blender_utils as BU
from . import leap_utils as LU
from . import panels as PNLS
from . import operators as OPS
from numpy import *
import sys
from . import Leap

class ModalTimerOperator(bpy.types.Operator):
    """Leap Motion Support"""
    bl_idname = "wm.leap_motion_ext"
    bl_label = "Leap Motion Extension"

    _timer = None
    listener = None
    controller = None
    circle_turn = 0
    mode = 0
    mode_camera = 0
    camera_state = "ALL"
    modes_names = ["Moving","Sculpting","Modify","Selection"]
    leap_pointer = None
    leap_circle = None
    curve_object = None
    curve_anim_path = None
    path_curve = None
    obj_selected = None
    marker_plane_coord = None
    prev_marker_coord = None
    checkMarkers = False
    checkCamera = False
    skinning_splines = []
    drawing_mode = ""
    counter = 0
    counter_two = 0
    cpoints = 1
    vec_vel = []
    vec_pos = []
    vec_dir = []
    vec_time = []
    finger_time = 0.0
    finger_last_vel = 0.0
    sculpt_flag = True
    vertices_flag = True
    points_position = []
    apply_stroke = True
    u = 0
    deriv = None
    name_prev_curve = None
    size_proportional = 1
    mirror=False
    

    def modal(self, context, event):
        if event.type == "ESC":
            return self.cancel(context)

        if event.type == "RIGHT_SHIFT" and event.value == "PRESS":
            self.mode = (self.mode + 1) % len(self.modes_names)
            print("Passaggio alla modalità: "+self.modes_names[self.mode])
            #self.report({'INFO'},"Passaggio alla modalità: "+self.modes_names[self.mode])
        
        if event.type in ["D","E","J","K","COMMA"] and event.value == "PRESS":
            #Inizio del disegno della curva
            if event.type == "D" and self.modes_names[self.mode] == "Moving" and bpy.context.mode == "OBJECT":
                if self.curve_object is None and self.drawing_mode == "":
                    print("Inizio disegno della curva")
                    self.drawing_mode = "DRAWING"
                    curve_data = bpy.data.curves.new(name="leap_curve",type="CURVE")
                    curve_data.dimensions = "2D"
                    object_data = bpy.data.objects.new("leap_curve_object",curve_data)
                    object_data.location = (0,0,0)
                    object_data.select = True
                    self.curve_object = object_data

                    bpy.context.scene.objects.link(object_data)
                    context.scene.objects.active = object_data
                
                    polyline = curve_data.splines.new("POLY")
                    polyline.points[0].co = (0.0,0.0,0.0,1.0)

                    r3d = BU.get_region3d(bpy.context)
                    r3d.view_rotation = Quaternion((1.0,0.0,0.0,0.0)) 
                elif self.drawing_mode == "DRAWING":
                    #Fine del disegno della curva
                    print("Disegno terminato")
                    bpy.ops.object.interpolate_op()
                    self.curve_object = None
                    self.drawing_mode = ""
            #Inizio del disegno del percorso curva
            elif event.type == "COMMA" and self.modes_names[self.mode] == "Moving" and bpy.context.mode == "OBJECT":
                if self.curve_object is None and self.drawing_mode == "" and context.scene['keep_data'] == False:
                    print("Inizio disegno della curva percorso")
                    self.drawing_mode = "DRAWING_PATH"
                    #inizializzo la curva che si vede durante il disegno con il dito
                    curve_data = bpy.data.curves.new(name="leap_curve_path",type="CURVE")
                    curve_data.dimensions = "3D"
                    curve_data.use_path = True
                    object_data = bpy.data.objects.new("leap_curve_p_object",curve_data)
                    object_data.location = (0,0,0)
                    object_data.select = True
                    self.curve_object = object_data

                    bpy.context.scene.objects.link(object_data)
                    context.scene.objects.active = object_data
                
                    polyline = curve_data.splines.new("POLY")
                    cursor_pos = Vector(bpy.context.scene.cursor_location)
                    polyline.points[0].co = (cursor_pos.x,cursor_pos.y,cursor_pos.z,1.0)

                    self.vec_vel = []
                    self.vec_pos = []
                    self.vec_dir = []
                    self.vec_time = []
                    self.points_position = []

                    self.counter = 0
                    self.counter_two = 0
                    self.cpoints = 1
                    self.u = 0
                    self.deriv = None

                elif self.drawing_mode == "DRAWING_PATH":
                    obj_selected = bpy.context.selected_objects
                    print("Disegno terminato")
                    vel = []
                    pos = []
                    dirct = []
                    time = []
                    sec_deriv=[]
                    #interpolazione della curva 3D
                    if self.curve_anim_path is None:
                        self.curve_anim_path, self.u = VU.interpolate_curve_3D(self.curve_object, context.scene['smoothing'], context.scene['order']) 
                        self.deriv = VU.calculate_derivate(self.u, self.curve_anim_path, 2)
                        for i in self.deriv:
                            sec_deriv.append(linalg.norm(Vector(i)))
                        max_curvature = sec_deriv[0]
                        min_curvature = sec_deriv[0]
                        for i in sec_deriv:
                            if i>max_curvature:
                                max_curvature = i
                            if i<min_curvature:
                                min_curvature = i
                        print(len(self.vec_pos))
                        print(len(self.u))
                        print(len(sec_deriv))
                        print(self.cpoints)
                        curvature_range = max_curvature/3
                        print("Max Curvature", max_curvature)
                        print("Range Min Curvature", curvature_range)
                        print("Min Curvature", min_curvature)

                        print(self.u[5])
                        if len(sec_deriv) > len(self.vec_pos):
                            diff = len(sec_deriv)-len(self.vec_pos)
                            for i in range(0,diff):
                                pop = sec_deriv.pop(i)
                                print("Pop Value",pop)
                       
                        #campionamento della velocità direzione e tempo secondo il grado di curvatura
                        for x in range(0,len(sec_deriv)):
                            if sec_deriv[x] > (curvature_range*2):
                                vel.append(self.vec_vel[x]) 
                                pos.append(self.vec_pos[x])
                                dirct.append(self.vec_dir[x])
                                time.append(self.vec_time[x])
                                self.points_position.append(x+1)
                            elif sec_deriv[x] > (curvature_range) and sec_deriv[x] <= (curvature_range*2):
                                if self.counter == 2:
                                    vel.append(self.vec_vel[x]) 
                                    pos.append(self.vec_pos[x])
                                    dirct.append(self.vec_dir[x])
                                    time.append(self.vec_time[x])
                                    self.counter = 0
                                    self.points_position.append(x+1)
                                self.counter += 1
                            elif sec_deriv[x] <= (curvature_range):
                                if sec_deriv[x] == 0.0:
                                    self.counter_two +=1
                                else:
                                    if self.counter_two == 3:
                                        vel.append(self.vec_vel[x]) 
                                        pos.append(self.vec_pos[x])
                                        dirct.append(self.vec_dir[x])
                                        time.append(self.vec_time[x])
                                        self.counter_two = 0
                                        self.points_position.append(x+1)
                                    self.counter_two +=1

                        #controllo se l'ultimo campionamento coincide con l'ultimo punto sulla curva
                        if pos[len(pos)-1] == self.vec_pos[len(self.vec_pos)-1]:
                            vel.pop()
                            pos.pop()
                            dirct.pop()
                            time.pop()

                        vel.append(self.finger_last_vel)

                        self.vec_pos = pos
                        self.vec_time = time
                        self.vec_dir = dirct
                        self.vec_vel = vel

                    self.name_prev_curve = BU.animation_path(bpy.context,self.curve_anim_path, self.vec_vel, self.vec_pos, self.vec_dir, obj_selected, self.cpoints, self.finger_time, self.vec_time, self.points_position, self.u, self.name_prev_curve)
                    bpy.ops.object.select_all(action='DESELECT')
                    self.curve_object.hide = True
                    #bpy.ops.object.delete()

                    if context.scene['keep_data'] == False :
                        self.vec_vel = []
                        self.vec_pos = []
                        self.vec_dir = []
                        self.vec_time = []
                        self.points_position = []

                        self.counter = 0
                        self.counter_two = 0
                        self.cpoints = 1
                        self.u = 0
                        self.deriv = None

                        self.curve_object = None
                        self.curve_anim_path = None
                        self.drawing_mode = ""

            elif self.curve_object is not None and event.type == "E" and self.drawing_mode in ["DRAWING","EXTRUDING"]:
                #Inizio estrusione curva
                if self.drawing_mode == "DRAWING":
                    self.drawing_mode = "EXTRUDING"
                    bpy.ops.object.interpolate_op()
                    bpy.ops.object.extrude_op()
                    self.curve_object = context.object
                    e_rot = Euler((pi/4,0,0),'XYZ')
                    r3d = BU.get_region3d(bpy.context)
                    r3d.view_rotation = e_rot.to_quaternion()
                else:
                    #Finita estrusione
                    print("Estrusione terminata")
                    self.curve_object = None
                    self.drawing_mode = ""
            elif self.curve_object is not None and event.type == "J" and self.drawing_mode in ["DRAWING","SKINNING"]:
                #Inizio skinning curva
                self.drawing_mode = "SKINNING"
                curve_tck = VU.interpolate_curve(self.curve_object,0.05,2)
                if curve_tck is not None:
                    #bpy.ops.object.delete()
                    self.skinning_splines.append(curve_tck)
                    self.curve_object.select = False

                    curve_data = bpy.data.curves.new(name="leap_curve",type="CURVE")
                    curve_data.dimensions = "2D"
                    object_data = bpy.data.objects.new("leap_curve_object",curve_data)
                    object_data.location = (0,0,0)
                    object_data.select = True
                    
                    bpy.context.scene.objects.link(object_data)
                    context.scene.objects.active = object_data
                    self.curve_object = object_data
                
                    polyline = curve_data.splines.new("POLY")
                    polyline.points[0].co = (0.0,0.0,0.0,1.0)
                elif self.drawing_mode == "SKINNING":
                    #Finito skinning
                    n_curves = len(self.skinning_splines)
                    self.curve_object.select = False
                    if n_curves > 1:
                        BU.skinning_curves(bpy.context,self.skinning_splines)
                    else:
                        print("Curva non valida")
                    self.skinning_splines = []
                    self.drawing_mode = ""
                    self.curve_object = None
            elif self.curve_object is not None and event.type == "K" and self.drawing_mode in ["DRAWING","SWINGING"]:
                if self.drawing_mode == "DRAWING":
                    #Inizio swinging curva
                    self.path_curve = VU.interpolate_curve(self.curve_object,0.05,2)
                    if self.path_curve is not None:
                        self.drawing_mode = "SWINGING"
                        self.curve_object.select = False
                        curve_data = bpy.data.curves.new(name="leap_curve",type="CURVE")
                        curve_data.dimensions = "2D"
                        object_data = bpy.data.objects.new("leap_curve_object",curve_data)
                        object_data.location = (0,0,0)
                        object_data.select = True
                        self.curve_object = object_data

                        bpy.context.scene.objects.link(object_data)
                        context.scene.objects.active = object_data
                
                        polyline = curve_data.splines.new("POLY")
                        polyline.points[0].co = (0.0,0.0,0.0,1.0)

                        r3d = BU.get_region3d(bpy.context)
                        r3d.view_rotation = Euler((pi/2,0,0),'XYZ').to_quaternion()
                elif self.drawing_mode == "SWINGING":
                    #Finito swinging
                    profile_curve = VU.interpolate_curve(self.curve_object,0.05,2)
                    self.curve_object.select = False
                    if profile_curve is not None:
                        BU.swinging_curves(bpy.context,self.path_curve,profile_curve)
                    self.drawing_mode = ""
                    self.curve_object = None
                    self.path_curve = None
            return {'PASS_THROUGH'} 
        
        if event.type in ["X","Y","Z"] and event.value == "PRESS" and event.oskey == True:
            if event.type == "X":
                if self.camera_state != "X":
                    self.camera_state = "X"
                else:
                    self.camera_state = "ALL"
            if event.type == "Y":
                if self.camera_state != "Y":
                    self.camera_state = "Y"
                else:
                    self.camera_state = "ALL"
            if event.type == "Z":
                if self.camera_state != "Z":
                    self.camera_state = "Z"
                else:
                    self.camera_state = "ALL"

        if self.leap_pointer != None and (self.modes_names[self.mode] == "Sculpting" or self.modes_names[self.mode] == "Modify") and event.type == "COMMA":
            if self.sculpt_flag == True and self.modes_names[self.mode] == "Sculpting":
                self.sculpt_flag = False
            elif self.sculpt_flag == False and self.modes_names[self.mode] == "Sculpting":
                self.sculpt_flag = True

            elif self.vertices_flag == True and self.modes_names[self.mode] == "Modify":
                self.vertices_flag = False
            elif self.vertices_flag == False and self.modes_names[self.mode] == "Modify":
                self.vertices_flag = True

        
        if event.shift == True and event.type == "COMMA" and event.value == "PRESS":
            #Passaggio da rotazione vista con mano a rotazione vista con marker
            self.checkMarkers = not self.checkMarkers
            print("Cambiato in: "+str(self.checkMarkers))

        if event.type == "P" and event.value == "PRESS":
            #Attivazione movimento camera
            self.camera_state = "ALL"
            self.checkCamera = not self.checkCamera
            print("Camera cambiata in: "+str(self.checkCamera))

        if self.checkMarkers and event.type == "TIMER" :
            #Rilevazione marker
            frame = self.controller.frame()
            imgs = frame.images
            l_img = imgs[0]
            r_img = imgs[1]

            XL = YL = XR = YR = -1

            MIN_MARKER_PIXELS = 50
            MAX_MARKERS_PIXELS = 160
            MIN_PIXEL_BRIGHTNESS = 250

            if l_img.is_valid and r_img.is_valid:
                l_data = l_img.data

                left_markers = []
                right_markers = []
 
                pix_set = set()
                for py in range(0,l_img.height,4):
                    for px in range(0,l_img.width,4):
                        if ((py*l_img.width) + px) not in pix_set and l_data[(py*l_img.width) + px] > MIN_PIXEL_BRIGHTNESS:
   
                            SP,SminX,SminY,SmaxX,SmaxY,pix_set  = LU.number_of_marked_pixels(l_data,l_img.width,l_img.height,px,py,0,200, pix_set) 
                            #print("SP:",str(SP))
                            if SP > MIN_MARKER_PIXELS and SP <MAX_MARKERS_PIXELS:
                                
                                XL = (SminX+SmaxX) // 2
                                YL = (SminY+SmaxY) // 2
                                #print("LEFT:"+str(SP)+". ["+str(XL)+","+str(YL)+"]")
                                left_markers.append((SP,XL,YL))

                if len(left_markers) >= 3:
                    r_data = r_img.data
                    pix_set = set()
                    for py in range(0,r_img.height,4):
                        for px in range(0,r_img.width,4):
                            if ((py*r_img.width) + px) not in pix_set and r_data[(py*r_img.width) + px] > MIN_PIXEL_BRIGHTNESS:
                                
                                SP,SminX,SminY,SmaxX,SmaxY, pix_set = LU.number_of_marked_pixels(r_data,r_img.width,r_img.height,px,py,0,200,pix_set)
                                #print("SP:",str(SP))
                                if SP > MIN_MARKER_PIXELS and SP <MAX_MARKERS_PIXELS:
                                    
                                    XR = (SminX+SmaxX) // 2
                                    YR = (SminY+SmaxY) // 2
                                    #print("RIGHT:"+str(SP)+". ["+str(XR)+","+str(YR)+"]")
                                    right_markers.append((SP,XR,YR))

                    if len(right_markers) == len(left_markers):
                        marker_points = []
                        right_markers.sort()
                        left_markers.sort()
                        for i in range(0,len(right_markers)):
                            SPR,XR,YR = right_markers[i]
                            SPL,XL,YL = left_markers[i]

                            SPdiff = abs(SPL-SPR)
                            
                            slopes_left = l_img.rectify(Leap.Vector(XL,YL,0))
                            slopes_right = r_img.rectify(Leap.Vector(XR,YR,0))
    
                            zp = 40/(slopes_right.x - slopes_left.x)
                            yp = zp * slopes_right.y
                            xp = zp * slopes_right.x - 20
                            #pixel_pos = Vector((xp,yp,zp))
                            pixel_pos = Vector((xp,yp))
                            marker_points.append(pixel_pos)
                            #print(pixel_pos)
                            #print("Differenza di pixel: "+str(SPdiff))
                            #print("-----------------------------------")

                        if self.marker_plane_coord is None:
                            self.marker_plane_coord = marker_points
                        rotQ, tran = VU.rigid_transform_3D(self.marker_plane_coord, marker_points)
                        #print("Errore: "+str(rotQ*self.marker_plane_coord[0]+tran-marker_points[0]))
                        self.marker_plane_coord = marker_points
                        #print("Rotazione: "+str(rotQ))
                        #print("Translazione: "+str(tran))
                        r3d = BU.get_region3d(bpy.context)
                        r3d.view_rotation = rotQ*r3d.view_rotation
                        view_rotation = r3d.view_rotation
                        move_vector = VU.rotate_vector(Vector((tran.x,tran.y,0)),view_rotation)
                        r3d.view_location = r3d.view_location - Vector((move_vector.x/10,move_vector.y/10,0))

                    else:
                        print("Rilevati marker diversi") 
                else:
                    print("Non sono stati rilevati 3 marker ma "+str(len(left_markers)))
                      
            else:
                print("Immagini non valide")          

        if event.type == 'TIMER':
            frame = self.controller.frame()
            oldframe = self.controller.frame(1)
            scene = bpy.context.scene       #prendo la scena
            iBox = oldframe.interaction_box #interactionBox mapping coordinate Leap

            hands_old = LU.get_hands_dict(oldframe)
            hands_new = LU.get_hands_dict(frame)

            obj = scene.objects.active      #seleziono l'oggetto attivo
            r3d = BU.get_region3d(bpy.context) #seleziono l'area 3D schermo
            
            #Creazione/Distruzione del puntatore in base alla modalità attuale
            if self.modes_names[self.mode] != "Selection" and self.modes_names[self.mode] != "Sculpting" and self.modes_names[self.mode] != "Modify":
                if self.leap_pointer:
                    print("Oggetto distrutto")
                    selobjs = [] #array degli oggetti
                    for o in bpy.data.objects:
                        if (o.name == "leappointer" or o.name == "sphere_pointer"):
                            o.select = True
                        else:
                            if (o.select):
                                selobjs.append(o)
                                o.select = False
                    self.leap_pointer = None
                    bpy.ops.object.mode_set(mode = 'OBJECT')
                    bpy.ops.object.delete()
                    for o in selobjs:
                        o.select = True
            elif obj is not None and obj.mode == "OBJECT" and self.modes_names[self.mode] == "Selection" :
                if self.leap_pointer.name == "sphere_pointer":
                    print("Oggetto distrutto")
                    selobjs = [] #array degli oggetti
                    for o in bpy.data.objects:
                        if (o.name == "sphere_pointer"):
                            o.select = True
                        else:
                            if (o.select):
                                selobjs.append(o)
                                o.select = False
                    self.leap_pointer = None
                    bpy.ops.object.mode_set(mode = 'OBJECT')
                    bpy.ops.object.delete()
                    for o in selobjs:
                        o.select = True
                if not self.leap_pointer:
                    print("Creazione oggetto")
                    selobjs = bpy.context.selected_objects
                    bpy.ops.wm.append(directory = "/usr/share/blender/2.78/scripts/startup/leapst_ext/blenderobj/leap.blend/Object/", filename="leappointer")
                    self.leap_pointer = bpy.data.objects["leappointer"]
                    bpy.ops.object.select_all(action='DESELECT')
                    for o in selobjs:
                        if o.name != "leappointer":
                            o.select = True
                self.leap_pointer.rotation_euler = r3d.view_rotation.to_euler() 
            elif obj is not None and obj.mode == "OBJECT" and self.modes_names[self.mode] == "Sculpting":
                if self.leap_pointer:
                    print("Oggetto distrutto")
                    selobjs = [] #array degli oggetti
                    for o in bpy.data.objects:
                        if (o.name == "leappointer"):
                            o.select = True
                        else:
                            if (o.select):
                                selobjs.append(o)
                                o.select = False
                    self.leap_pointer = None
                    bpy.ops.object.mode_set(mode = 'OBJECT')
                    bpy.ops.object.delete()
                    for o in selobjs:
                        o.select = True
                if not self.leap_pointer:
                    print("Creazione oggetto")
                    selobjs = bpy.context.selected_objects
                    bpy.ops.wm.append(directory = "/usr/share/blender/2.78/scripts/startup/leapst_ext/blenderobj/pointer.blend/Object/", filename="sphere_pointer")
                    self.leap_pointer = bpy.data.objects["sphere_pointer"]
                    bpy.ops.object.select_all(action='DESELECT')
                    for o in selobjs:
                        if o.name != "sphere_pointer" and o.type == "MESH":
                            o.select = True
                            self.obj_selected = o
                    if self.modes_names[self.mode] == "Sculpting":
                        bpy.data.objects[self.obj_selected.name].select = True   # select the object to be sculpted
                        bpy.ops.object.mode_set(mode = 'SCULPT')  
            elif obj is not None and obj.mode == "SCULPT" and self.modes_names[self.mode] == "Modify":
                #bpy.data.objects[self.obj_selected.name].select = True   # select the object to be sculpted
                bpy.ops.object.mode_set(mode = 'OBJECT') 
            elif obj is not None and obj.mode == "EDIT" and self.modes_names[self.mode] == "Modify":
                context.scene.tool_settings.proportional_edit = 'ENABLED'

            #Mano sinistra
            if hands_old["Left"] and not self.checkMarkers and self.checkCamera and self.curve_object is None:
                hando = hands_old["Left"]
                handn = hands_new["Left"]
                #controllo se la mano è aperta
                if handn and handn.grab_strength == 0 and hando.grab_strength == 0: 
                    fingerpoint_list_new = handn.fingers.extended()
                    fingerpoint_list_old = hando.fingers.extended()
                    if len(fingerpoint_list_new) == 3 and len(fingerpoint_list_old) == 3: #controllo il numero delle dita
                        for index in range(len(fingerpoint_list_new)): #scorro le dita trovate 
                            #spostamento asse X
                            if self.camera_state == "ALL" or self.camera_state == "X":
                                handn_pos = handn.palm_position
                                handn_pos_norm = iBox.normalize_point(handn_pos,False)

                                hando_pos = hando.palm_position
                                hando_pos_norm = iBox.normalize_point(hando_pos,False)

                                hand_x_diff = handn_pos_norm.x - hando_pos_norm.x

                                if abs(hand_x_diff) > 0.005:
                                    euler_rot = Euler((0.0,-8*hand_x_diff,0.0),'XYZ')
                                    r3d.view_rotation = r3d.view_rotation * euler_rot.to_quaternion()
                            #spostamento asse Z
                            if self.camera_state == "ALL" or self.camera_state == "Z":
                                handn_pos = handn.palm_position
                                handn_pos_norm = iBox.normalize_point(handn_pos,False)

                                hando_pos = hando.palm_position
                                hando_pos_norm = iBox.normalize_point(hando_pos,False)

                                hand_mov_vector = handn_pos_norm.z - hando_pos_norm.z#Vector((0.0,0.0,handn_pos_norm.z)) - Vector((0.0,0.0,hando_pos_norm.z))
                                
                                flag = 1
                                if hand_mov_vector > 0:
                                    flag = 1
                                else:
                                    flag = -1
                                if abs(hand_mov_vector) > 0.003:
                                    #view_rotation = r3d.view_rotation
                                    #move_vector = VU.rotate_vector(hand_mov_vector,view_rotation)
                                    #w = hand_mov_vector
                                    r3d.view_distance = r3d.view_distance - ((sqrt((hand_mov_vector)**2))*40*flag)
                            #spostamento asse Y
                            if self.camera_state == "ALL" or self.camera_state == "Y":    
                                handn_pos = handn.palm_position
                                handn_pos_norm = iBox.normalize_point(handn_pos,False)

                                hando_pos = hando.palm_position
                                hando_pos_norm = iBox.normalize_point(hando_pos,False)

                                hand_y_diff = handn_pos_norm.y - hando_pos_norm.y
                                if abs(hand_y_diff) > 0.006:
                                    euler_rot = Euler((-8*hand_y_diff,0.0,0.0),'XYZ')
                                    r3d.view_rotation = r3d.view_rotation * euler_rot.to_quaternion()

                        handn_n = handn.palm_normal
                        hando_n = hando.palm_normal

                        rotation_n = VU.rotation_quaternion(Vector((hando_n.x,hando_n.y,hando_n.z)),Vector((handn_n.x,handn_n.y,handn_n.z)))
                        normal_euler = rotation_n.to_euler()
                        
                        if abs(normal_euler.x) > 0.0003 and abs(normal_euler.z) > 0.0003:
                            euler_rot = Euler((-10*normal_euler.x,0.0,-5*normal_euler.z),'XYZ')
                            r3d.view_rotation = r3d.view_rotation * euler_rot.to_quaternion()

            if hands_old["Left"] and obj.type is not None and obj.mode == "EDIT" and self.modes_names[self.mode] == "Modify":
                hando = hands_old["Left"]
                handn = hands_new["Left"]
                #controllo se la mano è aperta
                if handn and handn.grab_strength == 0 and hando.grab_strength == 0: 
                    fingerpoint_list_new = handn.fingers.extended()
                    fingerpoint_list_old = hando.fingers.extended()
                    if len(fingerpoint_list_new) == 2 and len(fingerpoint_list_old) == 2:
                        handn_pos = handn.palm_position
                        handn_pos_norm = iBox.normalize_point(handn_pos,False)

                        hando_pos = hando.palm_position
                        hando_pos_norm = iBox.normalize_point(hando_pos,False)

                        hand_mov_vector = handn_pos_norm.y - hando_pos_norm.y
                        flag = 1
                        if hand_mov_vector > 0:
                            flag = 1
                        else:
                            flag = -1
                        self.size_proportional = self.size_proportional + ((sqrt((hand_mov_vector)**2)*20*flag))
                        if obj.type == "MESH" and obj.mode == "EDIT":
                            if self.size_proportional < 1:
                                self.size_proportional = 1
                            if self.size_proportional > 10:
                                self.size_proportional = 10
                        if obj.type == "MESH" and obj.mode == "SCULPT":
                            if self.size_proportional < 1:
                                self.size_proportional = 1
                            if self.size_proportional > 100:
                                self.size_proportional = 100
                        print(self.size_proportional)

                    if len(fingerpoint_list_new) == 5 and len(fingerpoint_list_old) == 5 and obj.type == "MESH" and obj.mode == "EDIT":
                        self.mirror = True

            #Mano destra
            if hands_old["Right"]:
                hando = hands_old["Right"]
                handn = hands_new["Right"]
                #Mano destra non chiusa in qualsiasi modalità
                if handn and handn.grab_strength == 0 and hando.grab_strength == 0: #controllo se la mano è aperta
                    fingerpoint_list_new = handn.fingers.extended()
                    fingerpoint_list_old = hando.fingers.extended()
                    if len(fingerpoint_list_new) == 5 and len(fingerpoint_list_old) == 5:
                        if obj.type == "MESH" and obj.mode == "EDIT" and self.modes_names[self.mode] == "Modify" and self.leap_pointer:
                            fingerpoint_new = fingerpoint_list_new[0]
                            fingerpoint_old = fingerpoint_list_old[0]

                            #Posizione
                            tip_new = fingerpoint_new.tip_position
                            tip_PNorm_new = iBox.normalize_point(tip_new,False)

                            tip_old = fingerpoint_old.tip_position
                            tip_PNorm_old = iBox.normalize_point(tip_old,False)

                            z_gap = (tip_PNorm_new.z - tip_PNorm_old.z)
                            self.leap_pointer.scale = self.leap_pointer.scale + 10*Vector((z_gap,z_gap,z_gap))
                    if len(fingerpoint_list_new) == 2 and len(fingerpoint_list_old) == 2: #controllo se il numero di dita è 1
                        fingerpoint_new = fingerpoint_list_new[0]
                        fingerpoint_old = fingerpoint_list_old[0]

                        #Posizione
                        tip_new = fingerpoint_new.tip_position
                        tip_PNorm_new = iBox.normalize_point(tip_new,False)

                        tip_old = fingerpoint_old.tip_position
                        tip_PNorm_old = iBox.normalize_point(tip_old,False)

                        #Direzione
                        tip_dir_new = fingerpoint_new.direction
                        tip_DNorm_new = iBox.normalize_point(tip_dir_new,False)

                        tip_dir_old = fingerpoint_old.direction
                        tip_DNorm_old = iBox.normalize_point(tip_dir_old,False)

                        tip_PNorm_vector = Vector((tip_PNorm_new.x,tip_PNorm_new.y,tip_PNorm_new.z)) - Vector((tip_PNorm_old.x,tip_PNorm_old.y,tip_PNorm_old.z)) 
                        view_rotation = r3d.view_rotation
                        move_vector = (VU.rotate_vector(tip_PNorm_vector,view_rotation))*40
                        tip_DNorm_vector = Vector((tip_DNorm_new.x,tip_DNorm_new.y,tip_DNorm_new.z)) - Vector((tip_DNorm_old.x,tip_DNorm_old.y,tip_DNorm_old.z))
                        
                        if obj.type == "MESH" and obj.mode == "EDIT" and self.modes_names[self.mode] == "Modify":
                            #bpy.ops.view3d.select_circle(x=move_vector.x, y=move_vector.y, radius=10)
                            new_rot = VU.rotation_quaternion(Vector((0.0,1.0,0.0)),tip_DNorm_vector)
                            new_rot.normalize()
                            self.leap_pointer.rotation_mode = 'QUATERNION'
                            self.leap_pointer.location = move_vector+self.leap_pointer.location
                            self.leap_pointer.rotation_quaternion = view_rotation * Quaternion((2,-1, 0, 0)) * new_rot
                            #loc_obj_space = [v.co for v in self.leap_pointer.data.vertices]
                            msh = obj.data
                            bm = bmesh.from_edit_mesh(msh)
                            lvert = []
                            for v in bm.verts:
                                if BU.point_inside(v.co, self.leap_pointer):
                                    if v.select and self.vertices_flag == False:
                                        v.select = False
                                    elif self.vertices_flag == True:
                                        v.select = True
                            bmesh.update_edit_mesh(msh, True)
                            
                    if len(fingerpoint_list_new) == 1 and len(fingerpoint_list_old) == 1: #controllo se il numero di dita è 1
                        fingerpoint_new = fingerpoint_list_new[0]
                        fingerpoint_old = fingerpoint_list_old[0]
                        #Indice della mano destra
                        if fingerpoint_new.type == 1 and fingerpoint_old.type == 1: #controllo se il dito è l'indice
                            #Posizione
                            tip_new = fingerpoint_new.tip_position
                            tip_PNorm_new = iBox.normalize_point(tip_new,False)

                            tip_old = fingerpoint_old.tip_position
                            tip_PNorm_old = iBox.normalize_point(tip_old,False)

                            #Direzione
                            tip_dir_new = fingerpoint_new.direction
                            tip_DNorm_new = iBox.normalize_point(tip_dir_new,False)

                            tip_dir_old = fingerpoint_old.direction
                            tip_DNorm_old = iBox.normalize_point(tip_dir_old,False)

                            #Velocità
                            tip_velocity_new = fingerpoint_new.tip_velocity
                            tip_VNorm_new = iBox.normalize_point(tip_velocity_new,False)       

                            view_rotation = r3d.view_rotation
                            
                            tip_PNorm_vector = Vector((tip_PNorm_new.x,tip_PNorm_new.y,tip_PNorm_new.z)) - Vector((tip_PNorm_old.x,tip_PNorm_old.y,tip_PNorm_old.z))

                            tip_VNorm_vector = Vector((tip_VNorm_new.x,tip_VNorm_new.y,tip_VNorm_new.z)) 

                            move_vector = (VU.rotate_vector(tip_PNorm_vector,view_rotation))*40

                            tip_DNorm_vector = Vector()     

                            if self.modes_names[self.mode] == "Modify" and obj is not None:
                                z_gap = (tip_PNorm_new.z - tip_PNorm_old.z)
                                if obj.mode == "OBJECT":
                                    #Scaling di un oggetto
                                    obj.scale = obj.scale + 10*Vector((z_gap,z_gap,z_gap))
                                    
                                elif obj.type == "MESH" and obj.mode == "EDIT":
                                    #Scaling di mesh in EDIT MODE con i vertici
                                    msh = obj.data
                                    bm = bmesh.from_edit_mesh(msh)
                                    lvert = []
                                    for v in bm.verts:
                                        if v.select:
                                            lvert.append(v)
                                    if len(bm.verts) == len(lvert):
                                        scale_factor = Vector((1.0,1.0,1.0)) + 4*Vector((z_gap,z_gap,z_gap))
                                        bmesh.ops.scale(bm,vec=scale_factor,verts=lvert)
                                        bmesh.update_edit_mesh(msh, True)
                                    else:
                                        if not hands_old["Left"]:
                                            self.mirror = False 
                                        bpy.ops.transform.translate(value=(move_vector.x,move_vector.y,move_vector.z), 
                                            constraint_axis=(False, False, False), 
                                            constraint_orientation='GLOBAL', 
                                            mirror=self.mirror, 
                                            proportional='ENABLED', 
                                            proportional_edit_falloff=str(context.scene.tool_settings.proportional_edit_falloff), 
                                            proportional_size=self.size_proportional)
                                        #bmesh.ops.translate(bm,vec=move_vector,verts=lvert)
                                        #bmesh.update_edit_mesh(msh, True)
                            else:
                                
                                if self.modes_names[self.mode] == "Moving" and self.drawing_mode in ["DRAWING_PATH"]:
                                    tip_DNorm_vector = VU.rotation_quaternion(Vector((tip_DNorm_old.x,tip_DNorm_old.y,tip_DNorm_old.z)),Vector((tip_DNorm_new.x,tip_DNorm_new.y,tip_DNorm_new.z)))

                                else:
                                    tip_DNorm_vector = Vector((tip_DNorm_new.x,tip_DNorm_new.y,tip_DNorm_new.z)) - Vector((tip_DNorm_old.x,tip_DNorm_old.y,tip_DNorm_old.z))

                                if self.modes_names[self.mode] == "Moving" and self.drawing_mode in ["DRAWING","SKINNING"] and self.curve_object is not None and abs(move_vector.x) < 0.2 and abs(move_vector.y) < 0.2 and (abs(move_vector.x) > 0.05 or abs(move_vector.y) > 0.05):
                                    #Disegno della curva NURBS
                                    splines = self.curve_object.data.splines
                                    last_spline = len(splines) - 1
                                    polyline = splines[last_spline]
                                    lpc = polyline.points[0].co
                                    npc = (move_vector.x+lpc[0],move_vector.y+lpc[1],0.0,lpc[3])
                                    polyline.points.add(1)
                                    polyline.points[1].co = npc
                                    next_polyline = splines.new("POLY")
                                    next_polyline.points[0].co = npc
                                    
                                if self.modes_names[self.mode] == "Moving" and self.drawing_mode in ["DRAWING_PATH"] and self.curve_object is not None:# and abs(move_vector.x) > 0.12 and abs(move_vector.y) > 0.12 and abs(move_vector.z) > 0.12: # and (abs(move_vector.x) > 0.05 or abs(move_vector.y) > 0.05 or abs(move_vector.z) > 0.05):
                                    #Disegno della curva NURBS PATH
                                    splines = self.curve_object.data.splines
                                    last_spline = len(splines) - 1
                                    polyline = splines[last_spline]
                                    lpc = polyline.points[0].co
                                    npc = (move_vector.x+lpc[0],move_vector.y+lpc[1],move_vector.z+lpc[2],lpc[3])
                                    polyline.points.add(1)
                                    polyline.points[1].co = npc
                                    next_polyline = splines.new("POLY")
                                    next_polyline.points[0].co = npc
                                    self.cpoints +=1 
                                    self.finger_time = fingerpoint_new.time_visible
                                    self.finger_last_vel = tip_VNorm_vector

                                    self.vec_vel.append(tip_VNorm_vector) 
                                    self.vec_pos.append(move_vector)#(tip_PNorm_vector) 
                                    self.vec_dir.append(tip_DNorm_vector)
                                    self.vec_time.append(fingerpoint_new.time_visible)

                                elif self.modes_names[self.mode] == "Moving" and self.drawing_mode == "SWINGING" and self.curve_object is not None and self.path_curve is not None and abs(move_vector.x) < 0.2 and abs(move_vector.z) < 0.2 and (abs(move_vector.x) > 0.05 or abs(move_vector.z) > 0.05):
                                    #Disegno della curva PROFILE di SWINGING
                                    splines = self.curve_object.data.splines
                                    last_spline = len(splines) - 1
                                    polyline = splines[last_spline]
                                    lpc = polyline.points[0].co
                                    npc = (move_vector.x+lpc[0],0.0,move_vector.z+lpc[2],lpc[3])
                                    polyline.points.add(1)
                                    polyline.points[1].co = npc
                                    next_polyline = splines.new("POLY")
                                    next_polyline.points[0].co = npc
                                elif self.modes_names[self.mode] == "Moving" and self.drawing_mode == "EXTRUDING" and self.curve_object is not None and abs(move_vector.y) < 0.5 and abs(move_vector.y) > 0.05:
                                    #Estrusione superficie NURBS
                                    surf_points = self.curve_object.data.splines[0].points
                                    for p,i in zip(surf_points,range(len(surf_points))):
                                        if i%2 == 0:
                                            p.co.z += move_vector.y         
                                    self.curve_object.select = True     
                                elif self.modes_names[self.mode] == "Selection" and self.leap_pointer:
                                    #Selezione di un oggetto
                                    self.leap_pointer.location = self.leap_pointer.location + move_vector
                                    for o in scene.objects:
                                        if o.type == "MESH" and BU.point_inside(self.leap_pointer.location,o): 
                                            o.select = True
                                            scene.objects.active = o
                                        elif scene.objects.active != o:
                                            o.select = False
                                elif self.modes_names[self.mode] == "Sculpting" and self.leap_pointer:
                                    #sculpting tools
                                    for o in scene.objects:
                                        if o.type == "MESH" and self.modes_names[self.mode] == "Sculpting" and o.name != self.leap_pointer.name:
                                            if self.sculpt_flag == True:  
                                                self.apply_stroke = sculpting_tool(context, self.leap_pointer, tip_PNorm_new, tip_PNorm_old, tip_DNorm_vector, self.obj_selected, self.apply_stroke, self.size_proportional)
                                            else:
                                                sclaing_sculpt(context,tip_PNorm_new.z,tip_PNorm_old.z,self.leap_pointer)

                                elif self.curve_object is None and self.modes_names[self.mode] == "Moving" and obj is not None:  
                                    if obj.mode == "OBJECT":
                                        #Spostamento di un oggetto
                                        obj.location = obj.location + move_vector
                                    elif obj.type == "MESH" and obj.mode == "EDIT":
                                        #Spostamento di vertici
                                        msh = obj.data
                                        bm = bmesh.from_edit_mesh(msh)
                                        for v in bm.verts:
                                            if v.select:
                                                v.co = v.co + move_vector
                                        bmesh.update_edit_mesh(msh, True)

                #Mano destra chiusa in modalità Spostamento
                if self.modes_names[self.mode] == "Moving" and handn and handn.grab_strength == 1 and hando.grab_strength == 1 and obj is not None and self.drawing_mode == "":
                    #Rotazione oggetto
                    handn_n = handn.palm_normal
                    hando_n = hando.palm_normal

                    rotation_n = VU.rotation_quaternion(Vector((hando_n.x,hando_n.y,hando_n.z)),Vector((handn_n.x,handn_n.y,handn_n.z)))
                    normal_euler = rotation_n.to_euler()

                    handn_pos = handn.palm_position
                    handn_pos_norm = iBox.normalize_point(handn_pos,False)

                    hando_pos = hando.palm_position
                    hando_pos_norm = iBox.normalize_point(hando_pos,False)
 
                    d_mov = {}
                    d_mov["N"] = 0.0#-5*(normal_euler.z)
                    d_mov["X"] = 10*(handn_pos_norm.x - hando_pos_norm.x)
                    d_mov["Y"] = -10*(handn_pos_norm.y - hando_pos_norm.y)
                    #d_mov["Z"] = 10*(handn_pos_norm.z - hando_pos_norm.z)
                    
                    #max_mov = max(d_mov,key=lambda x: abs(d_mov[x]))
                    #print("Massimo:"+ max_mov +" - "+str(d_mov[max_mov]))
                    rotation = Euler((d_mov["Y"],d_mov["X"],d_mov["N"]),'XYZ').to_quaternion()
                    view_rotation = r3d.view_rotation.copy()
                    view_rotation_conj = view_rotation.copy()
                    view_rotation_conj.conjugate()

                    rot_tot = view_rotation * rotation * view_rotation_conj
                    
                    if obj.mode == "OBJECT":
                        #Rotazione oggetto
                        rot_mat = rot_tot.to_matrix().to_4x4()
                        orig_loc, orig_rot, orig_scale = obj.matrix_world.decompose()
                        orig_loc_mat = Matrix.Translation(orig_loc)
                        orig_rot_mat = orig_rot.to_matrix().to_4x4()
                        orig_scale_mat = Matrix.Scale(orig_scale[0],4,(1,0,0)) * Matrix.Scale(orig_scale[1],4,(0,1,0)) * Matrix.Scale(orig_scale[2],4,(0,0,1))  
                        obj.matrix_world = orig_loc_mat * rot_mat * orig_rot_mat * orig_scale_mat
                    elif obj.type == "MESH" and obj.mode == "EDIT":
                        #Rotazione vertici
                        msh = obj.data
                        bm = bmesh.from_edit_mesh(msh)
                        sel_verts = []
                        s_pos = Vector((0,0,0))
                        for v in bm.verts:
                            if v.select:
                                sel_verts.append(v)
                                s_pos = s_pos + v.co
                        centroid = s_pos/len(sel_verts)
                        bmesh.ops.rotate(bm,cent=centroid,matrix=rot_tot.to_matrix(),verts=sel_verts,space=obj.matrix_local)
                        bmesh.update_edit_mesh(msh, True)

            for gesture in frame.gestures():
                if gesture.type == Leap.Gesture.TYPE_CIRCLE:
                    circle = Leap.CircleGesture(gesture)
                    circle_pointable = circle.pointable
                    if circle_pointable.is_finger:
                        circle_finger = Leap.Finger(circle_pointable)
                        if circle_finger.type == 1 and circle_finger.hand.is_left:
                            n_turn = math.floor(circle.progress)
                            if n_turn > 2 and self.curve_object is not None and self.drawing_mode == "DRAWING":
                                #Finito disegno della curva e spin sull'asse X
                                self.drawing_mode = ""
                                bpy.ops.object.interpolate_op()
                                bpy.ops.object.spin_op()
                                e_rot = Euler((pi/4,0,0),'XYZ')
                                r3d = BU.get_region3d(bpy.context)
                                r3d.view_rotation = e_rot.to_quaternion()
                                self.curve_object = None
                            elif self.curve_object is None:
                                if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/2:
                                    clockwise = 1
                                else:
                                    clockwise = -1     
                                turn_diff = n_turn - self.circle_turn
                                mulpos = 0
                                if turn_diff < 0:
                                    self.circle_turn = n_turn
                                    mulpos = n_turn
                                if turn_diff > 0:
                                    self.circle_turn = n_turn
                                    mulpos = turn_diff
                                r3d.view_distance = max(0,r3d.view_distance + mulpos*clockwise)

        return {'PASS_THROUGH'}

    def execute(self, context):
        wm = context.window_manager
        self.controller = Leap.Controller()
        self.controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        self.controller.set_policy(Leap.Controller.POLICY_IMAGES)
        print("Controller preso")
        PNLS.init_ui_prop(context.scene, 
                        smoothing_default=3.0, 
                        order_default=3, 
                        path_length_default=5, 
                        sample_default = 10,
                        object_move_default=0, 
                        time_move_default = False, 
                        constant_vel_default = False, 
                        offset_obj_default = 0.1,
                        offset_check_default = False,
                        keep_data_default = False,
                        fstart_default = 0)
        self._timer = wm.event_timer_add(0.05, context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        print("Controller disconnesso")
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        return {"FINISHED"}

def sculpting_tool(context, leap_pointer, fing_pos_new, fing_pos_old, fing_dir, obj_selected, apply_stroke, size_proportional):
    """
    Apply actions in blender to operate sculpting tool
    """
    overrider = {}
    for window in context.window_manager.windows:
            screen = window.screen 
            for area in screen.areas:
                if area.type == 'VIEW_3D':
                    for region in area.regions:
                        if region.type == 'WINDOW':
                            overrider =  {'window': window, 'screen': screen, 'area': area, 'region': region, 'scene': context.scene,'active_object': bpy.data.objects[obj_selected.name], 'edit_object':bpy.data.objects[obj_selected.name]}
                            break     
                                
    stroke_prop = [{'name': 'LEAP_Sculpt_Brush', 'mouse': (0,0), 'time':1, 'pen_flip':True,}]# 'is_start': False}]
    brush_stroke = bpy.ops.sculpt.brush_stroke
    view = BU.get_region3d(bpy.context) 

    #new_rot = VU.rotation_quaternion(VU.quaternion_to_vector(fing_dir),Vector((0.0,1.0,0.0))) 
    new_rot = VU.rotation_quaternion(Vector((0.0,1.0,0.0)),fing_dir)
    new_rot.normalize()

    fing_pos = Vector((fing_pos_new.x,fing_pos_new.y,fing_pos_new.z)) - Vector((fing_pos_old.x,fing_pos_old.y,fing_pos_old.z))
    #x, y, z = foo(fing_pos_old.x, fing_pos_old.y, fing_pos_old.z)
    #x2, y2, z2 = x + fing_pos_new.x*dist, y + fing_pos_new.y*dist, z + fing_pos_new.z*dist
    #stroke_prop = [{'name': 'LEAP_Sculpt_Brush', 'mouse': (0,0), 'time':1, 'pen_flip':True}]

    cam_dist_glob = VU.rotate_vector(Vector((0, 0, view.view_distance)),view.view_rotation)                  # camera distance alignment
    view_align = (VU.rotate_vector(fing_pos, view.view_rotation))*40  #+ view.location

    leap_pointer.rotation_mode = 'QUATERNION'
    leap_pointer.location = view_align + leap_pointer.location #cam_align_tool_pos #+ pos_change  
    leap_pointer.rotation_quaternion = view.view_rotation * Quaternion((2,-1, 0, 0)) * new_rot
    context.scene.objects.active = obj_selected
    #apply_stroke = True
    """
    mat = leap_pointer.matrix_world
    loc_obj_space = [v.co for v in leap_pointer.data.vertices]
    obj_in_obj = []
    for i in loc_obj_space:
        #if BU.point_inside(i,obj_selected):
            #a = BU.point_boundbox(i)
            obj_in_obj.append(i)

    loc_world_space=list(map(lambda x: mat*x, loc_obj_space))


    for i in range(0, len(loc_world_space)):
        p = loc_world_space[i]    
        x, y, z = p[0], p[1], p[2]
    

    stroke = [{ "name": "stroke1",
                "mouse" : (0.0, 0.0),
                "pen_flip" : False,
                "is_start": True,
                "location": leap_pointer.location,#(x, y, z),
                "pressure": 1.0,
                "time": 1.0,
                "size":90
                },
                
               {"name" : "stroke2",
                "mouse" : (0.0, 0.0),
                "pen_flip" : False,
                "is_start" : False,
                "location":leap_pointer.location,#(x, y, z),
                "mouse": (0.0, 0.0),
                "pressure":1.0,
                "time":1.0,
                "size":90
                }]
    """

    if apply_stroke:
        stroke_prop[0]['is_start'] = True   
        stroke_prop[0]['location'] = leap_pointer.location
        print("Sculpt location: ", stroke_prop[0])
        stroke_prop[0]['mouse'] = (0,0) 
        stroke_prop[0]['pressure'] = 1.0
        stroke_prop[0]['size'] = int(size_proportional)
        apply_stroke = False
    else:
        stroke_prop[0]['is_start'] = False 
        stroke_prop[0]['location'] = leap_pointer.location
        print("Sculpt location: ", stroke_prop[0])
        stroke_prop[0]['mouse'] = (0,0) 
        stroke_prop[0]['pressure'] = 1.0
        stroke_prop[0]['size'] = int(size_proportional)
        apply_stroke = True             
                
    brush_stroke(overrider, stroke=stroke_prop)
    #print(stroke)
    return apply_stroke

def sclaing_sculpt(context, fing_pos_old, fing_pos_new, obj):
    z_gap = (fing_pos_new - fing_pos_old)
    obj.scale = obj.scale + 10*Vector((z_gap,z_gap,z_gap))


def register():
    bpy.utils.register_class(OPS.SwingingOperator)
    bpy.utils.register_class(PNLS.SwingingSelectPanel)
    bpy.utils.register_class(OPS.SkinningOperator)
    bpy.utils.register_class(PNLS.SkinningSelectPanel)
    bpy.utils.register_class(OPS.ExtrudePathOperator)
    bpy.utils.register_class(PNLS.ExtrudePathSelectPanel)
    bpy.utils.register_class(OPS.InterpolateOperator)
    bpy.utils.register_class(PNLS.SmoothSelectPanel)
    bpy.utils.register_class(OPS.ExtrudeOperator)
    bpy.utils.register_class(PNLS.ExtrudeSelectPanel)
    bpy.utils.register_class(OPS.SpinOperator)
    bpy.utils.register_class(PNLS.SpinSelectPanel)
    bpy.utils.register_class(ModalTimerOperator)
    bpy.utils.register_class(PNLS.SmoothSelectPanel3D)

    print("Leap Ext Registrazione")

def unregister():
    bpy.utils.unregister_class(ModalTimerOperator)
    bpy.utils.unregister_class(PNLS.SpinSelectPanel)
    bpy.utils.unregister_class(OPS.SpinOperator)
    bpy.utils.unregister_class(PNLS.ExtrudeSelectPanel)
    bpy.utils.unregister_class(OPS.ExtrudeOperator)
    bpy.utils.unregister_class(PNLS.SmoothSelectPanel)
    bpy.utils.unregister_class(OPS.InterpolateOperator)
    bpy.utils.unregister_class(PNLS.ExtrudePathSelectPanel)
    bpy.utils.unregister_class(OPS.ExtrudePathOperator)
    bpy.utils.unregister_class(PNLS.SkinningSelectPanel)
    bpy.utils.unregister_class(OPS.SkinningOperator)
    bpy.utils.unregister_class(PNLS.SwingingSelectPanel)
    bpy.utils.unregister_class(OPS.SwingingOperator)
    bpy.utils.unregister_class(PNLS.SmoothSelectPanel3D)

if __name__ == "__main__":
    register()

