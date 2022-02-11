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

    # assert(form_pts.shape == (4,-1), "Formation points with wrong format!")
    model = translation(tx, ty, tz)
    form_pts = np.matmul(model, form_pts.T)
    return form_pts.T

def rotateFormation(form_pts, anglex, angley, anglez):

    # assert(form_pts.shape == (1,3), "Formation points with wrong format!")
    
    model = np.eye(4)

    # Eq 1: model equals to rotz*roty*rotx
    if anglex != 0:
        model = np.matmul(rotationX(anglex), model)
    if angley != 0:
        model = np.matmul(rotationY(angley), model)
    if anglez != 0:
        model = np.matmul(rotationZ(anglez), model)
    
    # Eq 2: form_pts equals to model*form_pts
    form_pts = np.matmul(model, form_pts.T)
    return form_pts.T

def scaleFormation(form_pts, sx, sy, sz):

    # assert(form_pts.shape == (1,3), "Formation points with wrong format!")

    model = scale(sx, sy, sz)
    form_pts = np.matmul(model, form_pts.T)
    return form_pts.T

def transformFormation(form_pts, sx, sy, sz, anglex, angley, anglez, tx, ty, tz):

    # assert(form_pts.shape == (1,3), "Formation points with wrong format!")

    modelS = scale(sx, sy, sz)

    modelR = np.eye(4)
    if anglex != 0:
        modelR = np.matmul(rotationX(anglex), modelR)
    if angley != 0:
        modelR = np.matmul(rotationY(angley), modelR)
    if anglez != 0:
        modelR = np.matmul(rotationZ(anglez), modelR)
    
    modelT = translation(tx, ty, tz)

    # Crate model matrix
    # Rotate first!!! Next scale and finally translate
    model = np.eye(4)
    model = np.matmul(modelR, model)
    model = np.matmul(modelS, model)
    model = np.matmul(modelT, model)

    # Apply model to form_points
    form_pts = np.matmul(model, form_pts.T)
    return form_pts.T



    







