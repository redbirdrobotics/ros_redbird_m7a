import cv2
import numpy as np

#pointsA = np.array([12,23],[34,45],[56,67])
pointsB = np.array([[12,23],[34,45],[56,67]])
pointsC = np.array([[[12,23],[34,45],[56,67]]])

#print cv2.boundingRect(pointsA)
print cv2.boundingRect(pointsB)
print cv2.boundingRect(pointsC)
