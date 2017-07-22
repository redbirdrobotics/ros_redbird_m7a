def alterList(alist):
    alist.extend([1,2,3,4])    
    del alist[2]
    return

alist = []
bList = [5]

alterList(alist)
print alist
del bList[0:1]
print bList
