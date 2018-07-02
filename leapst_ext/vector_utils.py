from mathutils import *
from math import *
from numpy import linalg
import numpy as NP
import scipy.interpolate as inter

#Metodo che trasforma il vettore v in quaternione
def vector_to_quaternion(v):
    return Quaternion((0.0,v.x,v.y,v.z))

#Metodo che trasforma il quaternione q in vettore
def quaternion_to_vector(q):
    return Vector((q.x,q.y,q.z))

#Metodo che restituisce il quaternione di rotazione tra v1 e v2
def rotation_quaternion(v1,v2):
    a = v1.cross(v2)
    qa = Quaternion((0.0,a.x,a.y,a.z))
    qa.w = 1 + v1.dot(v2)
    qa.normalize()
    return qa

#Metodo che ruota il vettore v in base al quaternione q
def rotate_vector(v,q):
    return q*v

#Metodo che ruota il vettore v in base al quaternione q e all'origine p
def rotate_vector_around_point(v,q,p):
    new_v = v - p
    rv = q*new_v   
    return rv + p

#Metodo che ruota i punti pts in base al quaternione q e all'origine o
def rotate_points(pts,q,o):
    new_pts = []
    for p in pts:
        p_coord = p.co
        p_vector = Vector((p_coord[0],p_coord[1],p_coord[2]))
        new_pts.append(rotate_vector_around_point(p_vector,q,o))
    return new_pts

def quaternion_obj_rotation (rot_x,rot_y,rot_z):
    # get angle
    alpha = acos( rot_z / sqrt( ( rot_x ** 2 + rot_y ** 2 + rot_z ** 2 ) ) )
    # get rotation axis
    if rot_z != 0:
        rot_axis_x = -rot_y * rot_z
        rot_axis_y = rot_x * rot_z
        rot_axis_z = 0
    else:
        rot_axis_x = 0
        rot_axis_y = 0
        rot_axis_z = 1
    
    # standardize vector for rotation axis
    weight = sqrt( rot_axis_x ** 2 + rot_axis_y ** 2 + rot_axis_z ** 2 )
    rot_axis_x = rot_axis_x / weight
    rot_axis_y = rot_axis_y / weight
    rot_axis_z = rot_axis_z / weight
    
    # get weights
    rot_axis_x = rot_axis_x * sin( alpha / 2 )
    rot_axis_y = rot_axis_y * sin( alpha / 2 )
    rot_axis_z = rot_axis_z * sin( alpha / 2 )
    rot_w = cos( alpha / 2 )
    rot_quat = Quaternion( ( rot_w, rot_axis_x, rot_axis_y, rot_axis_z ) ) # create new quaternion
    return rot_quat
    
#Metodo che calcola la rotazione e la traslazione tra il triangolo pA e il triangolo pB
def rigid_transform_3D(pA, pB):
    A = NP.mat(pA)
    B = NP.mat(pB)
    N = A.shape[0]
    centroid_A = NP.mean(A, axis=0)
    centroid_B = NP.mean(B, axis=0) 
    AA = A - NP.tile(centroid_A, (N, 1))
    BB = B - NP.tile(centroid_B, (N, 1))
    H = NP.transpose(AA) * BB
    U, S, Vt = linalg.svd(H)
    R = Vt.T * U.T
    if linalg.det(R) < 0:
        Vt[1,:] *= -1
        R = Vt.T * U.T
    t = -R*centroid_A.T + centroid_B.T
    rot = Matrix().to_3x3()
    for i in range(2):
        for j in range(2):
            rot[i][j] = R.tolist()[i][j]
    Q = rot.to_quaternion()
    t = Vector(t)
    return Q, t

#Metodo che interpola o approssima la curva obj 2D in base al livello di smoothing e il suo grado
def interpolate_curve(obj, smooth, degree):
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
            coord_Z = [p[2] for p in data_points]
 
            interpolated = inter.splprep([coord_X,coord_Y,coord_Z], s=smooth, k=degree)
            return interpolated[0]
        else:
            return None
    else:
        return None

#Metodo che interpola o approssima la curva obj 3D in base al livello di smoothing e il suo grado
def interpolate_curve_3D(obj, smooth, degree):
    if (obj is not None and obj.type == "CURVE" and obj.data.dimensions=="3D" and obj.data.splines[0].type == "POLY"):
        curve_data = obj.data
        data_points = []
        for s in curve_data.splines:
            for p in s.points:
                if p.co not in data_points:
                    data_points.append(p.co)
        if len(data_points) > 5:
            coord_X = [p[0] for p in data_points]
            coord_Y = [p[1] for p in data_points]
            coord_Z = [p[2] for p in data_points]
 
            interpolated = inter.splprep([coord_X,coord_Y,coord_Z], s=smooth, k=degree)
            return interpolated[0],interpolated[1]
        else:
            return None
    else:
        return None

def chunks(l, n):
    """Yield successive n-sized chunks from l."""
    for i in range(0, len(l), n):
        yield l[i:i + n]

def calculate_derivate (pos, curve, n):
    d = inter.splev(pos,curve,der=n)
    x = []
    y = []
    z = []
    l = 0
    d2 = []

    for i in d:
        a = chunks(NP.ndarray.flatten(i), len(NP.ndarray.flatten(i)))
        c = list(a)
        b = NP.reshape(c,(len(NP.ndarray.flatten(i)),1))

        if l == 0:
            for j in b: 
                x.append(float(j))
        if l == 1:
            for j in b:
                y.append(float(j))
        if l == 2:
            for j in b:
                z.append(float(j))
        l +=1  
    for i in range(0,len(x)):
        d2.append((Vector((x[i],y[i],z[i]))))
    #print(len(d2))
    return d2