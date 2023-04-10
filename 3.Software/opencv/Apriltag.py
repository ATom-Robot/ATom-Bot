import cv2
import pupil_apriltags as apriltag

cap = cv2.VideoCapture('http://192.168.4.1/stream')
at_detector = apriltag.Detector(families='tag36h11')

while (1):
    # 获得图像
    ret, frame = cap.read()
    # 检测按键
    k = cv2.waitKey(1)
    if k == 27:
        break

    # 检测apriltag
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray)
    for tag in tags:
        cv2.circle(frame, tuple(tag.corners[0].astype(
            int)), 4, (255, 0, 0), 2)  # left-top
        cv2.circle(frame, tuple(tag.corners[1].astype(
            int)), 4, (255, 0, 0), 2)  # right-top
        cv2.circle(frame, tuple(tag.corners[2].astype(
            int)), 4, (255, 0, 0), 2)  # right-bottom
        cv2.circle(frame, tuple(tag.corners[3].astype(
            int)), 4, (255, 0, 0), 2)  # left-bottom
    # 显示检测结果
    cv2.imshow('capture', frame)

cap.release()
cv2.destroyAllWindows()
