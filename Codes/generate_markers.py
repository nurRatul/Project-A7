import os
import cv2
import numpy as np


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "markers")
    os.makedirs(output_dir, exist_ok=True)

    marker_pixels = 1000
    sheet = np.full((3 * marker_pixels, 2 * marker_pixels, 3), 255, dtype=np.uint8)

    for marker_id in range(1, 5):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_pixels)
        marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        path = os.path.join(output_dir, f"marker_{marker_id}.png")
        cv2.imwrite(path, marker_bgr)
        print("saved", path)

        label = f"ID {marker_id}  (4x4, DICT_4X4_50)"
        cv2.putText(marker_bgr, label, (20, marker_pixels - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)
        row = (marker_id - 1) // 2
        col = (marker_id - 1) % 2
        sheet[row * marker_pixels:(row + 1) * marker_pixels, col * marker_pixels:(col + 1) * marker_pixels] = marker_bgr

    sheet_path = os.path.join(output_dir, "markers_sheet.png")
    cv2.imwrite(sheet_path, sheet)
    print("saved", sheet_path)


if __name__ == "__main__":
    main()
