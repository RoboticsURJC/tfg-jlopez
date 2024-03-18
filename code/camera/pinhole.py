

class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Point2D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = 1

# de 2d a 3d
def get3Dpoint(Point 2D):

    x = ((x-cx)*d)/fx
    y  = ((y -cy)*d)/ fy

    z = depth


    return 3dpoint

# de 3d a 2d 
def get2Dpoint(Point 3D):

    K = np.array([[333.76, 0.0, 310.99],
                  [0.0, 335.08, 230.93],
                  [0.0, 0.0, 1.0]])
    RT = np.array([[cos(40), 0, sin(40),     0],
                [    0 , 1,       0,     0],
                [-sin(40), 0, cos(40), 110]])
    2d = K*RT
    
    return 2d





# parámetros extrínsecos 

# punto en 3D
 
