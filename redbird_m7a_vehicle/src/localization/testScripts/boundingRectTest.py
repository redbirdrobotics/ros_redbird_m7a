import cv2
import numpy as np

#pointsA = np.array([12,23],[34,45],[56,67])
pointsB = np.array([[12,23],[34,45],[56,67]]).reshape((-1,1,2)).astype(np.int32)
#pointsC = np.asmatrix(pointsB)
pointsD = np.mat([[12,23],[34,45],[56,67]])

#print cv2.boundingRect(pointsA)
#print cv2.boundingRect(pointsB)
print cv2.boundingRect(pointsB)
