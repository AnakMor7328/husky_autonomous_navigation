from pyk4a import PyK4A, Config, ColorResolution, DepthMode, FPS
import cv2
import numpy as np

k4a = PyK4A(
    Config(
        color_resolution=ColorResolution.RES_720P,
        depth_mode=DepthMode.NFOV_UNBINNED,
        camera_fps=FPS.FPS_30,
    )
)

k4a.start()

print("Azure Kinect iniciada. Presiona q para salir.")

while True:
    capture = k4a.get_capture()

    if capture.color is not None:
        color = capture.color[:, :, :3]
        color = cv2.cvtColor(color, cv2.COLOR_BGRA2BGR) if color.shape[2] == 4 else color

        cv2.imshow("Azure Kinect RGB", color)

    if capture.depth is not None:
        depth = capture.depth

        h, w = depth.shape
        cx, cy = w // 2, h // 2
        distance_mm = depth[cy, cx]

        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = depth_vis.astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        cv2.circle(depth_vis, (cx, cy), 5, (255, 255, 255), -1)
        cv2.putText(
            depth_vis,
            f"Centro: {distance_mm} mm",
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2
        )

        cv2.imshow("Azure Kinect Depth", depth_vis)

        print(f"Distancia centro: {distance_mm} mm")

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

k4a.stop()
cv2.destroyAllWindows()
