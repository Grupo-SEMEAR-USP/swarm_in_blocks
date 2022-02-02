import numpy as np 

# Transform class that works with homogeneous coordinates
def translation(tx,ty,tz):
    model = [[1, 0, 0 , tx],
            [0, 1, 0, ty],
            [0, 0, 1, tz],
            [0, 0, 0, 1]]
    return np.array(model)

def rotationZ(angle):
    model = [[np.cos(angle), -np.sin(angle), 0 , 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]
    return np.array(model)

def rotationY(angle):
    model = [[np.cos(angle), 0, np.sin(angle) , 0],
            [0, 1, 0, 0],
            [-np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1]]
    return np.array(model)

def rotationX(angle):
    model = [[1, 0, 0 , 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle), np.cos(angle), 0],
            [0, 0, 0, 1]]
    return np.array(model)

def scale(sx, sy, sz):
    model = [[sx, 0, 0 , 0],
            [0, sy, 0, 0],
            [0, 0, sz, 0],
            [0, 0, 0, 1]]
    return np.array(model)

def translateFormation(form_pts, tx, ty, tz):

    assert(form_pts.shape == (1,3), "Formation points with wrong format!")
    model = translation(tx, ty, tz)
    form_pts = np.matmul(model, form_pts)
    return form_pts

def rotateFormation(form_pts, angleX, angleY, angleZ):

    assert(form_pts.shape == (1,3), "Formation points with wrong format!")
    
    model = np.eye(4)

    # Eq 1: model equals to rotz*roty*rotx
    if angleX != 0:
        model = np.matmul(rotationX(angleX), model)
    if angleY != 0:
        model = np.matmul(rotationY(angleY), model)
    if angleZ != 0:
        model = np.matmul(rotationZ(angleZ), model)
    
    # Eq 2: form_pts equals to model*form_pts
    form_pts = np.matmul(model, form_pts)
    return form_pts

def scaleFormation(form_pts, sx, sy, sz):

    assert(form_pts.shape == (1,3), "Formation points with wrong format!")

    model = scale(sx, sy, sz)
    form_pts = np.matmul(model, form_pts)
    return form_pts

def transformFormation(form_pts, sx, sy, sz, anglex, angleY, angleZ, tx, ty, tz):





