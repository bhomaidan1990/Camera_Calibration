# @Author : Belal Hmedan
#------------------------------
import numpy as np
import matplotlib.pyplot as plt
import math
# plt.style.use('seaborn') # pretty matplotlib plots
# plt.rcParams['figure.figsize'] = (12, 8)
# import scikitplot as skplt 
# import cv2 as cv
#------------------------------
np.random.seed(10)
def Normalization(nd, x):
    '''
    Normalization of coordinates (centroid to the origin and mean distance of sqrt(2 or 3).
    Input
    -----
    nd: number of dimensions, 3 here
    x: the data to be normalized (directions at different columns and points at rows)
    Output
    ------
    Tr: the transformation matrix (translation plus scaling)
    x: the transformed data
    '''

    x = np.asarray(x)
    m, s = np.mean(x, axis=0), np.std(x)
    if nd == 2:
        Tr = np.array([[s, 0, m[0]], [0, s, m[1]], [0, 0, 1]])
    else:
        Tr = np.array([[s, 0, 0, m[0]], [0, s, 0, m[1]], [0, 0, s, m[2]], [0, 0, 0, 1]])
        
    Tr = np.linalg.inv(Tr)
    x = np.dot( Tr, np.concatenate( (x.T, np.ones((1,x.shape[0]))) ) )
    x = x[0:nd, :].T

    return Tr, x


def DLTcalib(nd, xyz, uv):
    '''
    Camera calibration by DLT using known object points and their image points.
    Input
    -----
    nd: dimensions of the object space, 3 here.
    xyz: coordinates in the object 3D space.
    uv: coordinates in the image 2D space.
    The coordinates (x,y,z and u,v) are given as columns and the different points as rows.
    There must be at least 6 calibration points for the 3D DLT.
    Output
    ------
     L: array of 11 parameters of the calibration matrix.
     err: error of the DLT (mean residual of the DLT transformation in units of camera coordinates).
    '''
    if (nd != 3):
        raise ValueError('%dD DLT unsupported.' %(nd))
    
    # Converting all variables to numpy array
    if(isinstance(xyz, list)):
        xyz = np.asarray(xyz)
    if(isinstance(uv, list)):
        uv = np.asarray(uv)

    n = xyz.shape[0]

    # Validating the parameters:
    if uv.shape[0] != n:
        raise ValueError('Object (%d points) and image (%d points) have different number of points.' %(n, uv.shape[0]))

    if (xyz.shape[1] != 3):
        raise ValueError('Incorrect number of coordinates (%d) for %dD DLT (it should be %d).' %(xyz.shape[1],nd,nd))

    if (n < 6):
        raise ValueError('%dD DLT requires at least %d calibration points. Only %d points were entered.' %(nd, 2*nd, n))
        
    # Normalize the data to improve the DLT quality (DLT is dependent of the system of coordinates).
    # This is relevant when there is a considerable perspective distortion.
    # Normalization: mean position at origin and mean distance equals to 1 at each direction.
    Txyz, xyzn = Normalization(nd, xyz)
    Tuv, uvn = Normalization(2, uv)

    A = []

    for i in range(n):
        x, y, z = xyzn[i, 0], xyzn[i, 1], xyzn[i, 2]
        u, v = uvn[i, 0], uvn[i, 1]
        A.append( [x, y, z, 1, 0, 0, 0, 0, -u * x, -u * y, -u * z, -u] )
        A.append( [0, 0, 0, 0, x, y, z, 1, -v * x, -v * y, -v * z, -v] )

    # Convert A to array
    A = np.asarray(A) 

    # Find the 11 parameters:
    U, S, V = np.linalg.svd(A)

    # The parameters are in the last line of Vh and normalize them
    L = V[-1, :] / V[-1, -1]
    # print(L)
    # Camera projection matrix
    H = L.reshape(3, nd + 1)
    # print(H)

    # Denormalization
    # pinv: Moore-Penrose pseudo-inverse of a matrix, generalized inverse of a matrix using its SVD
    H = np.dot( np.dot( np.linalg.pinv(Tuv), H ), Txyz )
    # print(H)
    H = H / H[-1, -1]
    # print(H)
    L = H.flatten('C')
    # print('L=',L)

    # Mean error of the DLT (mean residual of the DLT transformation in units of camera coordinates):
    uv2 = np.dot( H, np.concatenate( (xyz.T, np.ones((1, xyz.shape[0]))) ) ) 
    uv2 = uv2 / uv2[2, :] 
    # Mean distance:
    err = np.sqrt( np.mean(np.sum( (uv2[0:2, :].T - uv)**2, 1)) ) 

    return L, err

def DLT(P3D,P2D):
    # mu, sigma = 0, 1
    nd = 3
    errs = []
    for sigma in range(0,100,1): # 30
        # print('len__XYZ',len(xyz))
        noise = np.random.normal(0,sigma/10,(6,2))
        P2 = np.array(P2D).reshape(6,2)
        # print(P2)
        P3 = P2.astype(np.float)
        uv_noise = P3 + noise
        #print(type(noise),noise.shape)
        P, err = DLTcalib(nd, P3D, uv_noise)
        if(sigma):
            err +=errs[sigma-1]
        errs.append(err)    
    
    #-------------------------------
    print('Matrix')
    #print(P)
    P = P.reshape(3,4)
    H1 = P[:,:3]
    H2 = P[:,3]
    Q, R = np.linalg.qr(H1)
    print('Q=',Q,'\n----------\nR',R)
    K = np.linalg.inv(R)
    K = K / K[-1,-1]
    Rot = np.linalg.inv(Q)
    C = np.dot(np.linalg.inv(H1),H2)
    t = - np.dot(Rot, C)
    print('\n----------\nK =',K,'\n----------\nRot=',Rot,'\n----------\nt=',C)
    print('\nError = ',err)
    #-------------------------------
    for i in range(len(errs)):
        errs[i]=errs[i]/100
    with plt.style.context('Solarize_Light2'):
        plt.figure(num='Error vs Noise')
        x = []
        for i in range(len(errs)):
            x.append(i/10.) 
        plt.plot(x,errs, dashes=[6, 2],label='Error in pixels',color='#2ca02c')
        plt.legend()
        # plt.xscale('log',nonposx='clip')
        # plt.xlim(0., 10.0)
        plt.xlabel(r'Noise level ($\sigma$)',fontsize=14)
        plt.ylabel('Error in pixels',fontsize=14)
    plt.show()

def plot3D(point_list):
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    if(isinstance(point_list, list)):
        point_list = np.asarray(point_list).reshape(-1,3)
    # For each set of style and range settings, plot n random points in the box
    # defined by x in [23, 32], y in [0, 100], z in [zlow, zhigh].
    xs , ys, zs = point_list[0:6,0], point_list[0:6,1], point_list[0:6,2]
    xs2 , ys2, zs2 = point_list[6:,0], point_list[6:,1], point_list[6:,2]
    ax.scatter(xs, ys, zs, color='green', marker='o', label='3D Points')
    ax.scatter(xs2, ys2, zs2, color='red', marker='^', label='2D Points')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.view_init(elev=20., azim=-35)
    ax.legend()
    plt.show()
if __name__ == "__main__":
    pass
# Known 3D coordinates
xyz = np.array([[-875, 0, 9.755], [442, 0, 9.755], [1921, 0, 9.755], [2951, 0.5, 9.755],
[-4132, 0.5, 23.618],[-876, 0, 23.618]],dtype=np.float)
# Known pixel coordinates
uv = np.array([[76, 706], [702, 706], [1440, 706], [1867, 706], [264, 523], [625, 523]],dtype=np.float)
uv0 = np.zeros((uv.shape[0],uv.shape[1]+1),dtype=np.float)
uv0[:,[0,2]] = uv
#print(uv0)
#print(uv)
xyzuv0 = np.zeros((12,3))
xyzuv0[0:6,:]=xyz
xyzuv0[6:,:] = uv0 
#------------------------------------------------------------------------------
DLT(xyz,uv)
plot3D(xyzuv0)