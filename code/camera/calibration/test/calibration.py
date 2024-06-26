import numpy as np
import cv2
import time
from picamera2 import Picamera2

# Initialize and configure Picamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (800, 800), "format": "RGB888"})
picam2.configure(config)
picam2.start()

# Create a window and set its position
cv2.namedWindow("Image Feed")
cv2.moveWindow("Image Feed", 159, -25)

# Initialize frame counters and timestamp
prev_frame_time = time.time()
cal_image_count = 0
frame_count = 0

while True:
    # Capture frame
    frame = picam2.capture_array()

    # Increment frame count
    frame_count += 1

    # Save image every 30 frames
    if frame_count == 30:
        cv2.imwrite(f"cal_image_{cal_image_count}.jpg", frame)
        cal_image_count += 1
        frame_count = 0

    # Calculate and display FPS
    new_frame_time = time.time()
    fps = 1 / (new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time
    cv2.putText(frame, f"FPS: {int(fps)}", (10, 40), cv2.FONT_HERSHEY_PLAIN, 3, (100, 255, 0), 2, cv2.LINE_AA)

    # Show the frame
    cv2.imshow("Image Feed", frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cv2.destroyAllWindows()
picam2.stop()
