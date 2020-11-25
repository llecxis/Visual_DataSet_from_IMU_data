import numpy as np


class CP_Rotation:

    def __init__(self, matrix = np.identity(3)):
        self.__rotMtx = matrix
           
            
    def quat_to_matrix( quat ):
        # https://github.com/rock-learning/pytransform3d
        w, x, y, z = quat
        x2 = 2.0 * x * x
        y2 = 2.0 * y * y
        z2 = 2.0 * z * z
        xy = 2.0 * x * y
        xz = 2.0 * x * z
        yz = 2.0 * y * z
        xw = 2.0 * x * w
        yw = 2.0 * y * w
        zw = 2.0 * z * w
    
        mtx = np.array([[1.0 - y2 - z2, xy - zw, xz + yw],
                      [xy + zw, 1.0 - x2 - z2, yz - xw],
                      [xz - yw, yz + xw, 1.0 - x2 - y2]])
        return mtx

    def matrix_to_quat( mtx ):
        # https://github.com/rock-learning/pytransform3d
        q = np.empty(4)
    
        # Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
        trace = np.trace(mtx)
        if trace > 0.0:
            sqrt_trace = np.sqrt(1.0 + trace)
            q[0] = 0.5 * sqrt_trace
            q[1] = 0.5 / sqrt_trace * (mtx[2, 1] - mtx[1, 2])
            q[2] = 0.5 / sqrt_trace * (mtx[0, 2] - mtx[2, 0])
            q[3] = 0.5 / sqrt_trace * (mtx[1, 0] - mtx[0, 1])
        else:
            if mtx[0, 0] > mtx[1, 1] and mtx[0, 0] > mtx[2, 2]:
                sqrt_trace = np.sqrt(1.0 + mtx[0, 0] - mtx[1, 1] - mtx[2, 2])
                q[0] = 0.5 / sqrt_trace * (mtx[2, 1] - mtx[1, 2])
                q[1] = 0.5 * sqrt_trace
                q[2] = 0.5 / sqrt_trace * (mtx[1, 0] + mtx[0, 1])
                q[3] = 0.5 / sqrt_trace * (mtx[0, 2] + mtx[2, 0])
            elif mtx[1, 1] > mtx[2, 2]:
                sqrt_trace = np.sqrt(1.0 + mtx[1, 1] - mtx[0, 0] - mtx[2, 2])
                q[0] = 0.5 / sqrt_trace * (mtx[0, 2] - mtx[2, 0])
                q[1] = 0.5 / sqrt_trace * (mtx[1, 0] + mtx[0, 1])
                q[2] = 0.5 * sqrt_trace
                q[3] = 0.5 / sqrt_trace * (mtx[2, 1] + mtx[1, 2])
            else:
                sqrt_trace = np.sqrt(1.0 + mtx[2, 2] - mtx[0, 0] - mtx[1, 1])
                q[0] = 0.5 / sqrt_trace * (mtx[1, 0] - mtx[0, 1])
                q[1] = 0.5 / sqrt_trace * (mtx[0, 2] + mtx[2, 0])
                q[2] = 0.5 / sqrt_trace * (mtx[2, 1] + mtx[1, 2])
                q[3] = 0.5 * sqrt_trace
        return q
        
    
    
    @classmethod
    def from_quat(cls, quat):
        return cls( CP_Rotation.quat_to_matrix(quat))
    
    @classmethod
    def from_matrix(cls, matrix):
        return cls( matrix )
    
    @classmethod
    def identity(cls):
        return cls( )

    @classmethod
    def random(cls, random_state=np.random.RandomState(0)):
        (u1,u2,u3) = random_state.rand(3)
        
        # http://planning.cs.uiuc.edu/node198.html
        # K. Shoemake.
        # Uniform random rotations.
        # In D. Kirk, editor, Graphics Gems III, pages 124-132. Academic, New York, 1992. 
        
        sqrt_1_m_u1 = np.sqrt(1-u1)
        sqrt_u1 = np.sqrt(u1)
        q1 = sqrt_1_m_u1 * np.sin(2 * np.pi * u2)
        q2 = sqrt_1_m_u1 * np.cos(2 * np.pi * u2)
        q3 = sqrt_u1 * np.sin(2 * np.pi * u3)
        q4 = sqrt_u1 * np.cos(2 * np.pi * u3)
        
        quat = np.array([q1,q2,q3,q4])
        return cls(CP_Rotation.quat_to_matrix(quat))

    def __mul__(self, other):
        
        multmtx = np.matmul(self.__rotMtx, other.as_matrix())
        return self.__class__(multmtx)
    
    def inv(self):
        return self.__class__(self.__rotMtx.copy().transpose())
    
    def as_matrix(self):
        return self.__rotMtx.copy()
    
    def as_quat(self):
        return CP_Rotation.matrix_to_quat(self.__rotMtx).copy()
    

if __name__ == "__main__":
    
    rot1 = CP_Rotation()
    print(type(rot1))    
    print(rot1.as_matrix())   

    mtx2 = np.array([[1,0,0], [0,0,1], [0,1,0]])   
    rot2 = CP_Rotation.from_matrix(mtx2)
    print(rot2.as_matrix())   
    
    quat = [1,0,0,0]
    rot3 = CP_Rotation.from_quat(quat)
    print(rot3.as_matrix())   
    
    rot4 = CP_Rotation()
    print(rot4.as_quat()) 
    
    rot5 = rot3 * rot4
    print(rot5.as_matrix()) 
    
    rot6 = CP_Rotation.random()
    print(rot6.as_matrix()) 
    print(np.linalg.det(rot6.as_matrix()))
    
    rot7 = rot6.inv()
    rot8 = rot6 * rot7
    print(rot8.as_matrix()) 
    
    print("\n45 deg around x")
    quat2 = [0.9238795, 0.3826834, 0, 0]
    rot9 = CP_Rotation.from_quat(quat2)
    print(rot9.as_matrix()) 
    
    print("\n45 deg around y")
    quat3 = [0.9238795, 0, 0.3826834, 0]
    rot10 = CP_Rotation.from_quat(quat3)
    print(rot10.as_matrix()) 
    
    print("\nfirst we rotate around X, then around Y")
    rot11 = rot9 * rot10
    print(rot11.as_matrix()) 
    
    # This will be inverse order
    print("\nInverse order")
    print((rot10*rot9).as_matrix()) 
    