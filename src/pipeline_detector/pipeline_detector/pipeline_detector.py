import cv2
import numpy as np
import sys


class ArUcoDetector:
    def __init__(self):
        # Use integer directly - avoids enum lookup segfault on some builds
        # DICT_ARUCO_ORIGINAL = 16
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(16)

        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.adaptiveThreshConstant = 7
        self.parameters.minMarkerPerimeterRate = 0.03
        self.parameters.maxMarkerPerimeterRate = 4.0

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.detected_markers = []
        self.marker_positions = {}
        self.confirmation_buffer = {}

        self._last_corners = []
        self._last_ids = None

    def _preprocess(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return clahe.apply(gray)

    def detect(self, frame):
        """Run detectMarkers ONCE per frame, cache result, return new confirmed IDs."""
        try:
            gray = self._preprocess(frame)
            corners, ids, _ = self.detector.detectMarkers(gray)
        except Exception as e:
            print(f"ArUco detect error: {e}")
            self._last_corners = []
            self._last_ids = None
            return []

        self._last_corners = corners if corners is not None else []
        self._last_ids = ids

        new_detections = []

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_id = int(marker_id)
                corner = corners[i][0]
                cx = float(np.mean(corner[:, 0]))
                cy = float(np.mean(corner[:, 1]))

                if self._is_new_marker(marker_id, cx, cy):
                    self.confirmation_buffer.setdefault(marker_id, 0)
                    self.confirmation_buffer[marker_id] += 1

                    if self.confirmation_buffer[marker_id] >= 3:
                        if marker_id not in self.detected_markers:
                            self.detected_markers.append(marker_id)
                            self.marker_positions[marker_id] = (cx, cy)
                            new_detections.append(marker_id)
                            print(f"✓ NEW MARKER CONFIRMED: ID {marker_id}")
                        del self.confirmation_buffer[marker_id]

        return new_detections

    def _is_new_marker(self, marker_id, x, y, min_distance=200):
        if marker_id not in self.marker_positions:
            return True
        px, py = self.marker_positions[marker_id]
        return np.hypot(x - px, y - py) > min_distance

    def visualize(self, frame):
        """Draw cached corners — no second detectMarkers call."""
        if self._last_ids is not None and len(self._last_corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame, self._last_corners, self._last_ids)
        return frame

    def get_marker_list(self):
        return self.detected_markers


def detect_yellow_pipeline(frame, hsv_lower, hsv_upper):
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

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None, None, None, None, None, mask

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    norm_x = (cx - width / 2) / (width / 2)
    norm_y = (cy - height / 2) / (height / 2)

    [vx, vy, _, _] = cv2.fitLine(largest, cv2.DIST_L2, 0, 0.01, 0.01)
    angle = np.degrees(np.arctan2(float(vy[0]), float(vx[0])))

    return True, (cx, cy), norm_x, norm_y, angle, mask


def draw_visualization(frame, detected, centroid, norm_x, norm_y, angle):
    vis = frame.copy()
    height, width = frame.shape[:2]

    if not detected:
        cv2.putText(vis, "NO PIPELINE DETECTED", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        return vis

    cx, cy = centroid
    cv2.circle(vis, (cx, cy), 15, (0, 0, 255), -1)
    cv2.arrowedLine(vis, (width // 2, height // 2), (cx, cy), (0, 255, 0), 3)
    cv2.putText(vis,
                f"offset: ({norm_x:.2f}, {norm_y:.2f})  angle: {angle:.1f}deg",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    return vis


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 pipeline_detector.py <video_path>")
        sys.exit(1)

    cap = cv2.VideoCapture(sys.argv[1])
    fps = cap.get(cv2.CAP_PROP_FPS)
    print("FPS:", fps)
    delay = int(1000 / fps) if fps > 0 else 33
    print("Delay (ms):", delay)
    if not cap.isOpened():
        print(f"Error: Cannot open video {sys.argv[1]}")
        sys.exit(1)

    hsv_lower = np.array([15, 30, 30])
    hsv_upper = np.array([90, 255, 255])

    aruco_detector = ArUcoDetector()

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            break
        if not isinstance(frame, np.ndarray) or frame.size == 0:
            continue

        detected, centroid, norm_x, norm_y, angle, mask = detect_yellow_pipeline(
            frame, hsv_lower, hsv_upper
        )

        # detect() must run before visualize() to populate the cache
        new_markers = aruco_detector.detect(frame)

        if new_markers:
            all_markers = aruco_detector.get_marker_list()
            with open("markers.txt", "w") as f:
                f.write(",".join(map(str, all_markers)))
            print(f"  markers.txt updated: {all_markers}")

        vis_frame = draw_visualization(frame, detected, centroid, norm_x, norm_y, angle)
        vis_frame = aruco_detector.visualize(vis_frame)

        markers = aruco_detector.get_marker_list()
        text = f"Markers: {markers}" if markers else "No markers yet"
        cv2.putText(vis_frame, text,
                    (10, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("Pipeline Detection", vis_frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    markers = aruco_detector.get_marker_list()
    with open("markers.txt", "w") as f:
        f.write(",".join(map(str, markers)))
    print(f"Final markers saved: {markers}")


if __name__ == "__main__":
    main()
