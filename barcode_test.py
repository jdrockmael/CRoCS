import cv2
import numpy as np
from pyzbar.pyzbar import decode
camera_id = 0
delay = 1
window_name = 'OpenCV Barcode'

#bd = cv2.barcode.BarcodeDetector()
cap = cv2.VideoCapture(camera_id)

while True:
    success, img = cap.read()

    for barcode in decode(img):
        myData = barcode.data.decode('utf-8')
        #print(myData)
        pts = np.array([barcode.polygon], np.int32)
        pts = pts.reshape((-1,1,2))

        cv2.polylines(img,[pts],True,(255,0,255), 5)
        pts2 = barcode.rect
        cv2.putText(img,myData, (pts2[0], pts2[1]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (255,0,255),2)
    
    cv2.imshow('Result', img)
    cv2.waitKey(1)
# while True:
#     ret, frame = cap.read()

#     if ret:
#         ret_bc, points, decoded_info = bd.detectAndDecode(frame)
        
#         if ret_bc:
#             print(points)
#             print(ret_bc)
#             print(decoded_info)
#             frame = cv2.polylines(frame, points.astype(int), True, (0, 255, 0), 3)
#             for s, p in zip(decoded_info, points):
#                 if s:
#                     print(s)
#                     frame = cv2.putText(frame, s, p[1].astype(int),
#                                         cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)
#         cv2.imshow(window_name, frame)

#     if cv2.waitKey(delay) & 0xFF == ord('q'):
#         break

cv2.destroyWindow(window_name)