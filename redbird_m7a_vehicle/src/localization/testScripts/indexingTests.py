import numpy as np

zero = np.zeros(((5,5,3)))
tpA = (1,2)
tpB = (2,2)
tpC = (1,3)

listA = [tpA, zero, tpB, zero, tpC]

listB = [index for index, elem in enumerate(listA) if type(elem) == tuple and elem[0] == 1]
print listB    

