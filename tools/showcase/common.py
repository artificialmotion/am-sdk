"""Shared helpers for the showcase clips (SIMULATED media). Needs matplotlib, imageio-ffmpeg.

Style: paper #f4f3ee background, ink #161616, a small 'Simulated' caption, no em dashes.
Clips are drawn as matplotlib figures, streamed frame by frame to ffmpeg (H.264), then converted to a
palette GIF at <= 720 px wide.
"""
import os
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import imageio_ffmpeg

PAPER, INK, GREY, RULE, ACCENT = "#f4f3ee", "#161616", "#7a7a74", "#cfcdc4", "#b3261e"
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "python"))
OUT = os.path.join(HERE, "..", "..", "docs", "media")

plt.rcParams.update({
    "figure.facecolor": PAPER, "axes.facecolor": PAPER, "savefig.facecolor": PAPER,
    "axes.edgecolor": INK, "axes.labelcolor": INK, "text.color": INK, "xtick.color": INK, "ytick.color": INK,
    "font.family": "DejaVu Sans", "font.size": 8, "axes.linewidth": 0.8, "axes.spines.top": False,
    "axes.spines.right": False, "lines.linewidth": 1.4, "axes.grid": False,
})


def new_fig(w_px=720, h_px=400, scale=2):
    """Figure of w_px x h_px 'logical' pixels, rendered at scale x."""
    dpi = 100 * scale
    return plt.figure(figsize=(w_px / 100, h_px / 100), dpi=dpi)


def caption(fig, text, y=0.02, size=6.5, color=GREY, x=0.015, ha="left"):
    fig.text(x if ha == "left" else 1 - x, y, text, fontsize=size, color=color, ha=ha, va="bottom")


def title(fig, text, sub=None):
    fig.text(0.015, 0.975, text, fontsize=11, va="top", color=INK)
    if sub:
        fig.text(0.015, 0.925, sub, fontsize=7, va="top", color=GREY)


def frame_rgb(fig):
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()


class Mp4Writer:
    def __init__(self, path, w, h, fps):
        self.p = subprocess.Popen(
            [imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", "%dx%d" % (w, h), "-r", str(fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p",
             "-crf", "24", "-preset", "slow", "-movflags", "+faststart", path], stdin=subprocess.PIPE)

    def add(self, rgb):
        self.p.stdin.write(rgb.tobytes())

    def close(self):
        self.p.stdin.close()
        self.p.wait()


def mp4_to_gif(mp4, gif, width=720, fps=15):
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    vf = "fps=%d,scale=%d:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];" \
         "[b][p]paletteuse=dither=none" % (fps, width)
    subprocess.check_call([ff, "-y", "-loglevel", "error", "-i", mp4, "-vf", vf, "-loop", "0", gif])


def render_clip(name, make_frame, n_frames, fps=25, size=(720, 400), scale=2, gif_fps=15):
    """make_frame(i) -> fig (already updated). Streams to docs/media/<name>.mp4 and .gif."""
    os.makedirs(OUT, exist_ok=True)
    mp4 = os.path.join(OUT, name + ".mp4")
    w, h = size[0] * scale, size[1] * scale
    wr = Mp4Writer(mp4, w, h, fps)
    for i in range(n_frames):
        fig = make_frame(i)
        wr.add(frame_rgb(fig))
    wr.close()
    plt.close("all")
    gif = os.path.join(OUT, name + ".gif")
    mp4_to_gif(mp4, gif, size[0], gif_fps)
    print("%s: mp4 %.2f MB, gif %.2f MB" % (name, os.path.getsize(mp4) / 1e6, os.path.getsize(gif) / 1e6))
