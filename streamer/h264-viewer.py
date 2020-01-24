import cv2

cap = cv2.VideoCapture("http://192.168.1.56:5001/")
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H', '2', '6', '4'))
while cap.isOpened():
    ret, frame = cap.read()
    if frame is None:
        continue
    cv2.imshow("Window", frame)
    cv2.waitKey(1)
