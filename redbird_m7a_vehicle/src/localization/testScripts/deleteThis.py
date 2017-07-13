import numpy as np

sumArray = np.array([[1,2,3,4],[2,3,4,5],[3,4,5,6]])

newArray = np.delete(sumArray, 2, 0)
print sumArray, newArray

np.delete(sumArray, 1, 0)
print sumArray

np.delete(sumArray, 0, 0)
print sumArray

listA = [0,1,2,3,4,5,6,7,8,9,10,11,12,13]
listB = [1,7,8]

print range(len(listA))
for num in range(len(listB)):
    listA.remove(listB[num])
    

print listA
