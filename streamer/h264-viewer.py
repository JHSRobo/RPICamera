import cv2

cap = cv2.VideoCapture("http://192.168.1.56:5000/")
#cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H', '2', '6', '4'))
#cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
#cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
while cap.isOpened():
    ret, frame = cap.read()
    if frame is None:
        continue
    cv2.imshow("Camera Feed", frame)
    cv2.waitKey(1)
