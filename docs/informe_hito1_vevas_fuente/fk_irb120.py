import numpy as np
DH_TABLE = [
    (0.0,       0.290, 0.000, -np.pi / 2),
    (-np.pi / 2, 0.000, 0.270,  0.0),
    (0.0,       0.000, 0.070, -np.pi / 2),
    (0.0,       0.302, 0.000,  np.pi / 2),
    (0.0,       0.000, 0.000, -np.pi / 2),
    (0.0,       0.072, 0.000,  0.0),
]
JOINT_LIMITS_DEG = [(-165.,165.),(-110.,110.),(-110.,70.),(-160.,160.),(-120.,120.),(-400.,400.)]
JOINT_LIMITS = np.radians(np.array(JOINT_LIMITS_DEG))
def dh_matrix(theta,d,a,alpha):
    ct,st=np.cos(theta),np.sin(theta); ca,sa=np.cos(alpha),np.sin(alpha)
    return np.array([[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0,sa,ca,d],[0,0,0,1]])
def forward_kinematics(q,upto=6):
    q=np.asarray(q,float).ravel(); T=np.eye(4)
    for i in range(upto):
        off,d,a,al=DH_TABLE[i]; T=T@dh_matrix(q[i]+off,d,a,al)
    return T
