
import cv2
import numpy as np
import sys


def detect_yellow_pipeline(frame, hsv_lower, hsv_upper):
    """
    Detect yellow pipeline in frame
    
    Returns:
        - detected: bool
        - centroid: (x, y) pixel position
        - normalized_x: -1 (left) to +1 (right), 0 = centered
        - normalized_y: -1 (top) to +1 (bottom), 0 = centered  
        - angle: pipeline orientation in degrees
        - mask: binary mask for visualization
    """
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None, None, None, None, None, mask
    
    largest = max(contours, key=cv2.contourArea)
    
    
    if cv2.contourArea(largest) < 1000:
        return None, None, None, None, None, mask
    
    # Calculate centroid
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None, None, None, None, None, mask
    
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    
    # Normalize to [-1, 1] where 0 = center
    norm_x = (cx - width/2) / (width/2)
    norm_y = (cy - height/2) / (height/2)
    
    # Calculate orientation
    [vx, vy, x, y] = cv2.fitLine(largest, cv2.DIST_L2, 0, 0.01, 0.01)
    angle = np.arctan2(float(vy[0]), float(vx[0])) * 180 / np.pi  # FIXED LINE
    
    return True, (cx, cy), norm_x, norm_y, angle, mask

def draw_visualization(frame, detected, centroid, norm_x, norm_y, angle):
    """Draw all visualization overlays"""
    vis = frame.copy()
    height, width = frame.shape[:2]
    
    # Draw center crosshair (target position)
    center_color = (0, 255, 255)  # Yellow
    cv2.line(vis, (width//2, 0), (width//2, height), center_color, 2)
    cv2.line(vis, (0, height//2), (width, height//2), center_color, 2)
    cv2.circle(vis, (width//2, height//2), 20, center_color, 2)
    
    if not detected:
        # No pipeline detected
        cv2.putText(vis, "NO PIPELINE DETECTED", (10, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        return vis
    
    cx, cy = centroid
    
    # Draw pipeline centroid (big red dot)
    cv2.circle(vis, (cx, cy), 15, (0, 0, 255), -1)  # Filled red
    cv2.circle(vis, (cx, cy), 20, (255, 255, 255), 3)  # White outline
    
    # Draw orientation line
    length = 100
    angle_rad = angle * np.pi / 180
    end_x = int(cx + length * np.cos(angle_rad))
    end_y = int(cy + length * np.sin(angle_rad))
    cv2.line(vis, (cx, cy), (end_x, end_y), (255, 0, 255), 4)  # Purple line
    
    # Draw offset arrow (from center to centroid)
    cv2.arrowedLine(vis, (width//2, height//2), (cx, cy), (0, 255, 0), 3)
    
    # Calculate offset distance in pixels
    offset_pixels = np.sqrt((cx - width//2)**2 + (cy - height//2)**2)
    
    # Display information
    y_pos = 40
    line_height = 35
    
    # Box background for text
    cv2.rectangle(vis, (5, 5), (400, 220), (0, 0, 0), -1)
    cv2.rectangle(vis, (5, 5), (400, 220), (255, 255, 255), 2)
    
    # Centroid position
    cv2.putText(vis, f"Centroid: ({cx}, {cy})", 
               (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    y_pos += line_height
    
    # Normalized position
    color = (0, 255, 0) if abs(norm_x) < 0.1 else (0, 165, 255)
    cv2.putText(vis, f"Normalized X: {norm_x:+.3f}", 
               (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    y_pos += line_height
    
    cv2.putText(vis, f"Normalized Y: {norm_y:+.3f}", 
               (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    y_pos += line_height
    
    # Offset
    cv2.putText(vis, f"Offset: {offset_pixels:.1f} px", 
               (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    y_pos += line_height
    
    # Angle
    cv2.putText(vis, f"Angle: {angle:.1f} deg", 
               (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    y_pos += line_height
    
    # Status
    if abs(norm_x) < 0.1 and abs(norm_y) < 0.1:
        status = "CENTERED!"
        status_color = (0, 255, 0)
    elif abs(norm_x) < 0.3:
        status = "NEARLY CENTERED"
        status_color = (0, 255, 255)
    else:
        status = "OFF CENTER"
        status_color = (0, 165, 255)
    
    cv2.putText(vis, status, (10, y_pos), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
    
    # Direction hint (what AUV should do)
    y_pos += 50
    if abs(norm_x) > 0.05:
        direction = "MOVE RIGHT" if norm_x < 0 else "MOVE LEFT"
        cv2.putText(vis, f"-> {direction}", (10, y_pos), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    return vis


def main():
    if len(sys.argv) < 2:
        print("Usage: python pipeline_detector.py <video_path>")
        print("Example: python pipeline_detector.py sim_video.mp4")
        sys.exit(1)
    
    video_path = sys.argv[1]
    
    # Open video
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Cannot open video {video_path}")
        sys.exit(1)
    
    # Get video info
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print(f"\n{'='*50}")
    print(f"Video: {video_path}")
    print(f"Resolution: {width}x{height}")
    print(f"FPS: {fps:.1f}")
    print(f"Total Frames: {total_frames}")
    print(f"{'='*50}\n")
    
    # HSV thresholds for yellow (TUNE THESE!)
    hsv_lower = np.array([15, 100, 100])  # [H, S, V]
    hsv_upper = np.array([35, 255, 255])
    
    # Create windows
    cv2.namedWindow('Pipeline Detection', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mask View', cv2.WINDOW_NORMAL)
    cv2.namedWindow('HSV Tuner', cv2.WINDOW_NORMAL)
    
    # Create trackbars for HSV tuning
    cv2.createTrackbar('H Min', 'HSV Tuner', hsv_lower[0], 180, lambda x: None)
    cv2.createTrackbar('H Max', 'HSV Tuner', hsv_upper[0], 180, lambda x: None)
    cv2.createTrackbar('S Min', 'HSV Tuner', hsv_lower[1], 255, lambda x: None)
    cv2.createTrackbar('S Max', 'HSV Tuner', hsv_upper[1], 255, lambda x: None)
    cv2.createTrackbar('V Min', 'HSV Tuner', hsv_lower[2], 255, lambda x: None)
    cv2.createTrackbar('V Max', 'HSV Tuner', hsv_upper[2], 255, lambda x: None)
    
    print("Controls:")
    print("  'q' - Quit")
    print("  'p' - Pause/Resume")
    print("  SPACE - Next frame (when paused)")
    print("  Adjust sliders to tune yellow detection\n")
    
    paused = False
    frame_count = 0
    detection_count = 0
    
    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                print("\nEnd of video")
                break
            frame_count += 1
        else:
            ret = True
        
        if ret and frame is not None:
            # Get current HSV values from trackbars
            hsv_lower[0] = cv2.getTrackbarPos('H Min', 'HSV Tuner')
            hsv_upper[0] = cv2.getTrackbarPos('H Max', 'HSV Tuner')
            hsv_lower[1] = cv2.getTrackbarPos('S Min', 'HSV Tuner')
            hsv_upper[1] = cv2.getTrackbarPos('S Max', 'HSV Tuner')
            hsv_lower[2] = cv2.getTrackbarPos('V Min', 'HSV Tuner')
            hsv_upper[2] = cv2.getTrackbarPos('V Max', 'HSV Tuner')
            
            # Detect pipeline
            detected, centroid, norm_x, norm_y, angle, mask = detect_yellow_pipeline(
                frame, hsv_lower, hsv_upper
            )
            
            if detected:
                detection_count += 1
            
            # Visualize
            vis_frame = draw_visualization(frame, detected, centroid, norm_x, norm_y, angle)
            
            # Add frame counter
            cv2.putText(vis_frame, f"Frame: {frame_count}/{total_frames}", 
                       (width - 250, height - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            if paused:
                cv2.putText(vis_frame, "PAUSED", 
                           (width - 150, height - 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Show windows
            cv2.imshow('Pipeline Detection', vis_frame)
            cv2.imshow('Mask View', mask)
            
            # Empty window for tuner (just shows trackbars)
            tuner_display = np.zeros((50, 400, 3), dtype=np.uint8)
            cv2.putText(tuner_display, "Adjust HSV values above", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.imshow('HSV Tuner', tuner_display)
        
        # Handle keyboard
        key = cv2.waitKey(1 if not paused else 0) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('p'):
            paused = not paused
            print(f"{'Paused' if paused else 'Resumed'}")
        elif key == ord(' ') and paused:
            # Step one frame forward
            ret, frame = cap.read()
            if ret:
                frame_count += 1
    
    # Print statistics
    if frame_count > 0:
        detection_rate = (detection_count / frame_count) * 100
        print(f"\n{'='*50}")
        print(f"Statistics:")
        print(f"  Frames processed: {frame_count}")
        print(f"  Detections: {detection_count}")
        print(f"  Detection rate: {detection_rate:.1f}%")
        print(f"\nFinal HSV values:")
        print(f"  Lower: [{hsv_lower[0]}, {hsv_lower[1]}, {hsv_lower[2]}]")
        print(f"  Upper: [{hsv_upper[0]}, {hsv_upper[1]}, {hsv_upper[2]}]")
        print(f"{'='*50}\n")
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()