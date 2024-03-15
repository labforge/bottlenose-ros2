import numpy as np
import cv2

def projection(tvec, rvec, A):
    K = np.asarray(A).reshape((3,3))
    R, _ = cv2.Rodrigues(np.asarray(rvec))
    t = np.asarray(tvec)

    Rt = np.column_stack((R, t)) # [R|t]
    P = np.matmul(K, Rt) # A[R|t]
    return P

#- left camera
f = 3920.5591279053148
cx = 969.7971334640745
cy = 584.4473157723114

tvec = [0.,0.,0.0]
rvec = [0.,0.,0.0]
A = [f, 0.000000, cx, 0.000000, f, cy, 0.000000, 0.000000, 1.000000]

P_left = projection(tvec, rvec, A)
np.set_printoptions(suppress=True)
print('Left Projection Matrix')
print(P_left.flatten())

#- right camera
f = 3906.0098842005256
cx = 943.7348575297392
cy = 588.2642263670929

x = -0.13426739250964554
y = -0.0003015798952237751
z = 0.003460693498448736
rx = 0.0004211170826982093
ry = 0.0015364029917207122
rz = 0.0012196301332966598

tvec = [x, y, z]
rvec = [rz, ry, rz]
A = [f, 0.000000, cx, 0.000000, f, cy, 0.000000, 0.000000, 1.000000]

P_right = projection(tvec, rvec, A)
print('\nRight Projection Matrix')
print(P_right.flatten())

