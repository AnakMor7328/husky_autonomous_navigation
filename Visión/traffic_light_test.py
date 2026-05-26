import cv2
import numpy as np

cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("No se pudo abrir la cámara")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("No se pudo leer imagen")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Rangos HSV básicos
    red1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
    red2 = cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
    red_mask = red1 + red2

    yellow_mask = cv2.inRange(hsv, (20, 100, 100), (35, 255, 255))
    green_mask = cv2.inRange(hsv, (40, 80, 80), (85, 255, 255))

    red_pixels = cv2.countNonZero(red_mask)
    yellow_pixels = cv2.countNonZero(yellow_mask)
    green_pixels = cv2.countNonZero(green_mask)

    values = {
        "RED": red_pixels,
        "YELLOW": yellow_pixels,
        "GREEN": green_pixels
    }

    color = max(values, key=values.get)

    if values[color] < 500:
        color = "NONE"

    if color == "RED":
        state = "STOP"
    elif color == "YELLOW":
        state = "SLOW"
    elif color == "GREEN":
        state = "GO"
    else:
        state = "NO_TRAFFIC_LIGHT"

    cv2.putText(frame, f"Color: {color}", (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    cv2.putText(frame, f"State: {state}", (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    cv2.imshow("Traffic Light Test", frame)

    print(f"Color: {color} | State: {state}")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()