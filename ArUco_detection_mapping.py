import cv2
import numpy as np

x1, x2 = 0, 639
x3, x4 = 0, 479
y1, y2 = 0, 0
y3, y4 = 0, 0
mx, cx = 0, 0
my, cy = 0, 0
def calibration():
    global x1, x2, x3, x4, y1, y2, y3, y4, mx, cx, my, cy
    try:
        y1 = float(input("Enter the top left x coordinate of robot:"))
        y2 = float(input("Enter the bottom right x coordinate of robot:"))
        mx, cx = calculate_line_equation(x1, y1, x2, y2)
        
        y3 = float(input("Enter the top left y coordinate of robot:"))
        y4 = float(input("Enter the bottom right y coordinate of robot:"))
        my, cy = calculate_line_equation(x3, y3, x4, y4)
        
        print(f"mx={mx}, cx={cx}, my={my}, cy={cy}")

    except ValueError as e:
        print(f"Error: {e}")
        return

def calculate_line_equation(x1, y1, x2, y2):
    if x2 - x1 == 0:
        raise ValueError("The line is vertical, slope is undefined.")
    m = (y2 - y1) / (x2 - x1)
    c = y1 - m * x1
    return m, c

def getRobotCoordinates(cameraX, cameraY, mx, cx, my, cy):
    return (cameraX * mx + cx, cameraY * my + cy)

def main():
    global mx, cx, my, cy
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    cap = cv2.VideoCapture(1)
    detected_marker_ids = set()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame. Exiting.")
            break

        corners, ids, _ = detector.detectMarkers(frame)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                marker_id = ids[i][0]
                c = corners[i][0]

                centerX = int((c[0][0] + c[2][0]) / 2)
                centerY = int((c[0][1] + c[2][1]) / 2)

                cv2.circle(frame, (centerX, centerY), 5, (0, 0, 255), -1)
                cv2.putText(frame, f'{centerX}, {centerY}',
                            (centerX + 20, centerY + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 255), 1, cv2.LINE_AA)

                if marker_id == 1 and marker_id not in detected_marker_ids:
                    detected_marker_ids.add(marker_id)
                    print(f"Marker 1 detected!")
                    print(f"Robot coordinates: {getRobotCoordinates(centerX, centerY, mx, cx, my, cy)}")
                elif marker_id == 2 and marker_id not in detected_marker_ids:
                    detected_marker_ids.add(marker_id)
                    print(f"Marker 2 detected!")
                    print(f"Robot coordinates: {getRobotCoordinates(centerX, centerY, mx, cx, my, cy)}")

        cv2.imshow("Image", frame)

        pressedKey = cv2.waitKey(1) & 0xFF
        if pressedKey == ord('q'):
            print("Exiting...")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
