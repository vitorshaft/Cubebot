import cv2
import numpy as np
import pathlib
import glob

#amostras coletadas em um tabuleiro 8x8 com casas de 1,95cm
pasta = "/coppeliaCamCalib/"
extensao = ".png"
tamQuad = 3.75
larg = 5
alt = 5

def calibrate_chessboard(dir_path, image_format, square_size, width, height):
    '''Calibrate a camera using chessboard images.'''
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    #images = pathlib.Path(dir_path).glob(f'*.{image_format}')
    images = glob.glob('C:/Users/vitor/3D Objects/Mestrado/Simulacao/Cubebot/coppeliaCamCalib/*.png')
    # Iterate through all images
    for fname in images:
        #img = cv2.imread(str(fname))
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (7,6), corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return [ret, mtx, dist, rvecs, tvecs]

calibrate_chessboard(pasta,extensao,tamQuad,larg,alt)
cv2.destroyAllWindows()