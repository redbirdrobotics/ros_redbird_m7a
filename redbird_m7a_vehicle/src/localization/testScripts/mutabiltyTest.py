def alterList(alist):
    alist.extend([1,2,3,4])    
    del alist[2]
    return

alist = []

alterList(alist)
print alist
