#!/usr/bin/env python3
import cv2
import numpy as np
import os
import time

print("Testing fire_node GUI simulation...")

# Simulate what fire_node does
window_name = "YOLO Fire Detection"
frame_count = 0

try:
    # Create window like fire_node does
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 800, 600)
    cv2.moveWindow(window_name, 100, 100)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_VISIBLE, 1)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)
    print("Window created successfully")

    while frame_count < 50:
        # Create a simulated camera image with fire detection overlay
        sim_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # Add fire detection simulation
        if frame_count % 10 == 0:
            # Simulate fire detection
            cv2.rectangle(sim_image, (200, 150), (400, 350), (0, 0, 255), 3)
            cv2.putText(sim_image, f"FIRE DETECTED! Frame: {frame_count}", (50, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        # Add status text
        cv2.putText(sim_image, f"Images Processed: {frame_count}", (50, 430),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(sim_image, "Confidence: 0.85 | Fire Dist: 5.2 m", (50, 460),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Display image
        cv2.imshow(window_name, sim_image)

        # Force window update
        cv2.waitKey(1)

        frame_count += 1
        print(f"Frame {frame_count}/50 - GUI should be updating")

        time.sleep(0.1)  # Simulate processing delay

    print("Fire node GUI simulation completed successfully!")
    cv2.destroyAllWindows()

except Exception as e:
    print(f"Fire node GUI simulation failed: {e}")
    import traceback
    traceback.print_exc()
