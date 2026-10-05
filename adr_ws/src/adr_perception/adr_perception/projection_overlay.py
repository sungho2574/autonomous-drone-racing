"""Draw GT-projected gate corners over detector debug images."""
import cv2
import numpy as np


def draw_projection(image, projected, reason, source):
    vis = image.copy()
    for gate_id, uv in projected:
        q = np.rint(uv).astype(np.int32)
        color = (255, 255, 0)  # cyan: predicted from GT, not detected corners
        cv2.polylines(vis, [q], True, color, 2, cv2.LINE_AA)
        for x, y in q:
            cv2.drawMarker(vis, (int(x), int(y)), color, cv2.MARKER_CROSS, 10, 1)
        x, y = q[np.argmin(q[:, 1])]
        label = f'G{gate_id} predicted'
        cv2.putText(vis, label, (max(0, int(x)), max(45, int(y)-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
    for i, text in enumerate((f'Prediction: {source}', f'GT projection: {reason}')):
        cv2.putText(vis, text, (8, 16 + i*18), cv2.FONT_HERSHEY_SIMPLEX,
                    .42, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, text, (8, 16 + i*18), cv2.FONT_HERSHEY_SIMPLEX,
                    .42, (255, 255, 0), 1, cv2.LINE_AA)
    return vis
