import numpy as np

someArray = np.array([[1,2,3,4,],[2,3,4,5],[3,4,5,6],[5,6,7,8]])

newArray = np.delete(someArray, [0,2], 0)

someList = [1,2,3,4,5,6,7,8,9]

newList = someList[5:7]

print newList

