"""
Convert AVI videos in assets/ to GIFs for README embedding.
Uses OpenCV to read AVI and imageio to write GIF (no ffmpeg required).
"""
import os
import sys

try:
    import cv2
    import imageio
except ImportError:
    print("Install: pip install opencv-python imageio")
    sys.exit(1)

ASSETS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")
FPS_GIF = 5   # GIF frame rate (lower = smaller file)
SCALE = 0.5   # scale factor (0.5 = half size) to keep GIF size reasonable
FRAME_SKIP = 1  # use every Nth frame (1 = all, 2 = half)

def avi_to_gif(avi_path, gif_path=None, fps=FPS_GIF, scale=SCALE, frame_skip=FRAME_SKIP):
    if gif_path is None:
        gif_path = os.path.splitext(avi_path)[0] + ".gif"
    cap = cv2.VideoCapture(avi_path)
    if not cap.isOpened():
        print(f"Cannot open {avi_path}")
        return False
    frames = []
    n = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if n % frame_skip != 0:
            n += 1
            continue
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if scale != 1.0:
            w, h = int(frame_rgb.shape[1] * scale), int(frame_rgb.shape[0] * scale)
            frame_rgb = cv2.resize(frame_rgb, (w, h), interpolation=cv2.INTER_AREA)
        frames.append(frame_rgb)
        n += 1
    cap.release()
    if not frames:
        print(f"No frames read from {avi_path}")
        return False
    imageio.mimsave(gif_path, frames, fps=fps, loop=0)
    print(f"Saved {gif_path} ({len(frames)} frames)")
    return True

def main():
    os.makedirs(ASSETS, exist_ok=True)
    for name in ("manipolatoreILC", "manipolatorePID"):
        avi = os.path.join(ASSETS, f"{name}.avi")
        gif = os.path.join(ASSETS, f"{name}.gif")
        if not os.path.isfile(avi):
            print(f"Skip (not found): {avi}")
            continue
        avi_to_gif(avi, gif)

if __name__ == "__main__":
    main()
