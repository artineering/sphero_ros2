#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple test script for ArUco marker detection.

This script tests the ArUco detector by:
1. Creating an ArucoDetector instance
2. Opening the webcam
3. Detecting ArUco markers in real-time
4. Displaying the camera feed with detected markers highlighted

Press 'q' to quit.
"""

import sys
import cv2
import time

# Add the aruco_slam package to Python path
sys.path.insert(0, '/home/svaghela/ros2_ws_2/src/aruco_slam')

from aruco_slam.aruco_detector import ArucoDetector


def main():
    """Main test function."""
    print("="*60)
    print("ArUco Detector Test")
    print("="*60)
    print()
    print("Initializing detector...")

    try:
        # Create detector instance (using camera 0, DICT_4X4_50)
        detector = ArucoDetector(camera_id=2, dict_type=cv2.aruco.DICT_4X4_1000)
        print("✓ Detector created successfully")
        print()

        # Start visualization
        print("Starting camera...")
        detector.begin_visualization()
        print("✓ Camera opened successfully")
        print()

        print("="*60)
        print("Detecting ArUco markers...")
        print("Press 'q' to quit")
        print("="*60)
        print()

        frame_count = 0
        start_time = time.time()

        while True:
            # Get frame from camera
            frame = detector.get_frame()
            if frame is None:
                print("Failed to capture frame")
                break

            # Detect markers
            corners, ids, rejected = detector.detect_markers(frame)

            # Draw detected markers on frame
            if ids is not None:
                # Draw marker boundaries
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Draw marker IDs and centers
                for i, marker_id in enumerate(ids.flatten()):
                    marker_id = int(marker_id)

                    # Get marker object
                    marker = detector.get_tag(marker_id)
                    if marker is not None:
                        center = marker.get_center()
                        if center is not None:
                            # Draw center point
                            cv2.circle(frame, (int(center[0]), int(center[1])), 5, (0, 255, 0), -1)

                            # Draw marker ID
                            cv2.putText(
                                frame,
                                f"ID: {marker_id}",
                                (int(center[0]) + 10, int(center[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (0, 255, 0),
                                2
                            )

                # Display detection info
                detection_text = f"Detected {len(ids)} marker(s): {[int(x) for x in ids.flatten()]}"
            else:
                detection_text = "No markers detected"

            # Draw detection status
            cv2.putText(
                frame,
                detection_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            # Calculate and display FPS
            frame_count += 1
            elapsed_time = time.time() - start_time
            if elapsed_time > 0:
                fps = frame_count / elapsed_time
                cv2.putText(
                    frame,
                    f"FPS: {fps:.1f}",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    2
                )

            # Show frame
            cv2.imshow('ArUco Detector Test', frame)

            # Check for quit key ('q')
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print()
                print("Quitting...")
                break

        # Cleanup
        detector.end_visualization()
        print()
        print("="*60)
        print("Test completed successfully")
        print(f"Total frames processed: {frame_count}")
        print(f"Average FPS: {frame_count / elapsed_time:.1f}")

        # Display all detected markers
        all_tags = detector.get_tags()
        if all_tags:
            print()
            print("All detected markers:")
            for marker_id, marker in all_tags.items():
                print(f"  - Marker ID {marker_id}: {len(marker.get_center_history()) + 1} detections")
        else:
            print()
            print("No markers were detected during the test")

        print("="*60)

    except RuntimeError as e:
        print(f"✗ Error: {e}")
        print()
        print("Troubleshooting:")
        print("  - Check that camera is connected and accessible")
        print("  - Try different camera_id (0, 1, 2, etc.)")
        print("  - Check camera permissions: ls -l /dev/video*")
        return 1

    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
