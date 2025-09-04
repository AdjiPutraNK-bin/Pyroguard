#!/usr/bin/env python3
import cv2
import numpy as np
import os
import time

print(f"DISPLAY: {os.environ.get('DISPLAY')}")
print("Testing OpenCV GUI with video simulation...")

# Create a test image
test_image = np.zeros((400, 600, 3), dtype=np.uint8)
test_image[:, :] = [0, 255, 0]  # Green background

# Add some moving elements to simulate video
frame_count = 0

try:
    cv2.namedWindow("GUI Test", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("GUI Test", 600, 400)
    cv2.moveWindow("GUI Test", 200, 200)
    cv2.setWindowProperty("GUI Test", cv2.WND_PROP_VISIBLE, 1)
    cv2.setWindowProperty("GUI Test", cv2.WND_PROP_TOPMOST, 1)
    print("Window created successfully")

    while frame_count < 100:  # Run for about 10 seconds
        # Create moving pattern
        moving_image = test_image.copy()
        y_pos = (frame_count * 5) % 400
        cv2.circle(moving_image, (300, y_pos), 20, (255, 0, 0), -1)
        cv2.putText(moving_image, f"Frame: {frame_count}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(moving_image, "Press ESC to exit", (50, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("GUI Test", moving_image)

        key = cv2.waitKey(100)  # 100ms delay
        if key == 27:  # ESC
            break

        frame_count += 1
        print(f"Frame {frame_count}/100 - Window should be updating")

    cv2.destroyAllWindows()
    print("GUI test completed successfully - video was moving!")

except Exception as e:
    print(f"GUI test failed: {e}")
    import traceback
    traceback.print_exc()
