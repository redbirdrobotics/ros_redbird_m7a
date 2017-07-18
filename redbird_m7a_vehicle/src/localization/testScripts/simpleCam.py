import cv2

esc == False
while esc == False:
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if ret == True:
        cv2.imshow('cam', frame)
        k = cv2.waitKey(30) & 0xff
        esc = False
        if k == 27:
            esc = True
    else:
        print ret

cap.release()
