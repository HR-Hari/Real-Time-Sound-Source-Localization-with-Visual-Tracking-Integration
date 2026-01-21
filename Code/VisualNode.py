from ultralytics import YOLO
import cv2
import numpy as np



GRID_COLS = 9          
GRID_ROWS = 6
CAMERA_FOV = 90        
CONF_THRESHOLD = 0.5
PHONE_CLASS_ID = 67    


def angle_to_grid_column(theta, fov=CAMERA_FOV, grid_cols=GRID_COLS):
    """
    Maps audio angle (degrees) to visual grid column
    """
    half_fov = fov / 2
    theta = max(-half_fov, min(half_fov, theta))  
    col_width = fov / grid_cols
    col = int((theta + half_fov) / col_width)
    return min(grid_cols - 1, max(0, col))

model = YOLO("yolov8n.pt", verbose=False)


cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Camera not accessible")
    exit()
print("✅ Camera started")


last_printed_objects = set()  
frame_count = 0               

while True:
    ret, frame = cap.read()
    if not ret:
        break

    img_h, img_w, _ = frame.shape
    cell_w = img_w / GRID_COLS
    cell_h = img_h / GRID_ROWS

    audio_angle = 30  # Sample Angle
    audio_grid_col = angle_to_grid_column(audio_angle)

    results = model.predict(frame, verbose=False) 

    detected_objects = []

    for result in results:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            if cls_id == PHONE_CLASS_ID and conf > CONF_THRESHOLD:
                x_center, y_center, w, h = box.xywh[0]

                grid_x = int(x_center / cell_w)
                grid_y = int(y_center / cell_h)

                detected_objects.append((grid_y, grid_x, conf))

                x1 = int(x_center - w/2)
                y1 = int(y_center - h/2)
                x2 = int(x_center + w/2)
                y2 = int(y_center + h/2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"Phone {conf:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    for (gy, gx, conf) in detected_objects:
        if gx == audio_grid_col:
            cv2.putText(
                frame,
                "SOUND SOURCE",
                (int(gx * cell_w) + 10, int(gy * cell_h) + 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )
 
            if (gy, gx) not in last_printed_objects:
                print(f"SOUND SOURCE detected at grid ({gy}, {gx}), confidence={conf:.2f}")
                last_printed_objects.add((gy, gx))


    for c in range(1, GRID_COLS):
        cv2.line(frame, (int(c * cell_w), 0), (int(c * cell_w), img_h), (200, 200, 200), 1)
    for r in range(1, GRID_ROWS):
        cv2.line(frame, (0, int(r * cell_h)), (img_w, int(r * cell_h)), (200, 200, 200), 1)


    ax1 = int(audio_grid_col * cell_w)
    ax2 = int((audio_grid_col + 1) * cell_w)
    cv2.rectangle(frame, (ax1, 0), (ax2, img_h), (255, 0, 0), 2)

    cv2.putText(
        frame,
        f"Audio Angle: {audio_angle} deg",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 0),
        2
    )

    annotated_frame = result.plot()

    cv2.imshow("YOLO Grid-Based Audio-Visual Localization", annotated_frame)

    frame_count += 1
    if frame_count % 20 == 0:
        last_printed_objects.clear()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
