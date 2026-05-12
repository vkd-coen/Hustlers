"""
Sumobot LiDAR + IMU Visualizer
Reads streaming data from STM32 over USB-serial (COM6 @ 115200)
and displays a real-time polar plot with IMU heading overlay.

Live-tunable sliders for: point size, point lifetime, max range,
quality filter, refresh rate.

Usage:
    pip install pyserial matplotlib numpy
    python visualizer.py

Press Ctrl+C in the terminal to stop.
"""

import serial
import re
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# ─── Configuration ────────────────────────────────────────────
SERIAL_PORT = "COM6"
BAUD_RATE   = 115200

# ─── Defaults (also used as initial slider values) ───────────
DEFAULT_POINT_SIZE     = 10
DEFAULT_POINT_LIFETIME = 0.25      # seconds
DEFAULT_MAX_DISTANCE   = 2000      # mm
DEFAULT_QUALITY_MIN    = 10
DEFAULT_UPDATE_HZ      = 10

# ─── Slider ranges (min, max) ────────────────────────────────
RANGE_POINT_SIZE     = (1,    40)
RANGE_POINT_LIFETIME = (0.05, 1.0)
RANGE_MAX_DISTANCE   = (500,  6000)
RANGE_QUALITY_MIN    = (0,    63)
RANGE_UPDATE_HZ      = (1,    30)

# ─── Live-tunable parameters (modified by sliders) ───────────
params = {
    "point_size":     DEFAULT_POINT_SIZE,
    "point_lifetime": DEFAULT_POINT_LIFETIME,
    "max_distance":   DEFAULT_MAX_DISTANCE,
    "quality_min":    DEFAULT_QUALITY_MIN,
    "update_hz":      DEFAULT_UPDATE_HZ,
}
params_lock = threading.Lock()

# ─── Shared state between serial thread and plot thread ──────
scan_dict  = {}   # key = int(angle), value = (angle, dist, quality, timestamp)
imu_state  = {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0, "temp": 0}
state_lock = threading.Lock()

# ─── Regex parsers for the STM32 output format ───────────────
LIDAR_RE = re.compile(
    r"LIDAR\s*\|\s*Sample=\d+\s*\|\s*Angle=([-\d.]+)\s*deg\s*\|\s*"
    r"Distance=([-\d.]+)\s*mm\s*\|\s*Quality=(\d+)"
)
IMU_RE = re.compile(
    r"IMU\s*\|\s*ACC mg:\s*X=([-\d]+)\s*Y=([-\d]+)\s*Z=([-\d]+)\s*\|\s*"
    r"GYRO dps:\s*X=([-\d.]+)\s*Y=([-\d.]+)\s*Z=([-\d.]+)\s*\|\s*"
    r"TEMP=([-\d.]+)\s*C"
)


def serial_reader():
    """Background thread: reads serial, parses LIDAR and IMU lines."""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} @ {BAUD_RATE}")
    except serial.SerialException as e:
        print(f"ERROR opening {SERIAL_PORT}: {e}")
        print("Check that the STM32 is connected and PuTTY is closed.")
        return

    buffer = ""
    while True:
        try:
            chunk = ser.read(256).decode("ascii", errors="ignore")
        except Exception as e:
            print(f"Serial read error: {e}")
            break

        if not chunk:
            continue

        buffer += chunk
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            m = LIDAR_RE.search(line)
            if m:
                angle = float(m.group(1))
                dist  = float(m.group(2))
                quality = int(m.group(3))
                with params_lock:
                    qmin  = params["quality_min"]
                    dmax  = params["max_distance"]
                if 0 <= angle < 360 and quality >= qmin and 0 < dist < dmax:
                    with state_lock:
                        scan_dict[int(angle)] = (angle, dist, quality, time.time())
                continue

            m = IMU_RE.search(line)
            if m:
                with state_lock:
                    imu_state["ax"]   = int(m.group(1))
                    imu_state["ay"]   = int(m.group(2))
                    imu_state["az"]   = int(m.group(3))
                    imu_state["gx"]   = float(m.group(4))
                    imu_state["gy"]   = float(m.group(5))
                    imu_state["gz"]   = float(m.group(6))
                    imu_state["temp"] = float(m.group(7))


def main():
    # Start serial reader thread
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    # ─── Plot setup ──────────────────────────────────────────
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(11, 11))
    fig.canvas.manager.set_window_title("Sumobot LiDAR + IMU Visualizer")

    # Reserve room at the bottom for sliders
    fig.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.28)

    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_ylim(0, params["max_distance"])
    ax.set_title("Sumobot view  —  LiDAR scan", color="white", pad=20, fontsize=14)
    ax.grid(color="#444", alpha=0.5)
    ax.tick_params(colors="#aaa")

    def update_yticks(maxd):
        if maxd <= 1000:
            ticks = [250, 500, 750, 1000]
        elif maxd <= 2500:
            ticks = [500, 1000, 1500, 2000, 2500]
        elif maxd <= 4000:
            ticks = [1000, 2000, 3000, 4000]
        else:
            ticks = [1000, 2000, 3000, 4000, 5000, 6000]
        ticks = [t for t in ticks if t <= maxd]
        ax.set_yticks(ticks)
        ax.set_yticklabels([f"{t/1000:.2g}m" for t in ticks])

    update_yticks(params["max_distance"])

    # Robot center marker
    ax.plot(0, 0, marker="o", color="#0ff", markersize=12, zorder=5)

    # Scatter plot
    scatter = ax.scatter([], [], s=params["point_size"], c=[], cmap="plasma",
                         vmin=0, vmax=63, alpha=0.9)

    # IMU info text
    info_text = fig.text(
        0.02, 0.97, "",
        color="#0ff", fontsize=10, family="monospace",
        verticalalignment="top",
    )

    # Heading arrow
    heading_line, = ax.plot([0, 0], [0, params["max_distance"] * 0.25],
                            color="#0f0", linewidth=2, alpha=0.7)

    # ─── Sliders ──────────────────────────────────────────────
    slider_color = "#222"
    slider_track = "#0ff"

    def make_slider(y, label, vmin, vmax, vinit, fmt="%.2f"):
        sax = fig.add_axes([0.15, y, 0.70, 0.025], facecolor=slider_color)
        s = Slider(sax, label, vmin, vmax, valinit=vinit, color=slider_track,
                   valfmt=fmt)
        s.label.set_color("#aaa")
        s.valtext.set_color("#0ff")
        return s

    s_size     = make_slider(0.20, "Point size",      *RANGE_POINT_SIZE,
                             DEFAULT_POINT_SIZE,     fmt="%d")
    s_lifetime = make_slider(0.16, "Point lifetime (s)", *RANGE_POINT_LIFETIME,
                             DEFAULT_POINT_LIFETIME, fmt="%.2f")
    s_maxdist  = make_slider(0.12, "Max distance (mm)", *RANGE_MAX_DISTANCE,
                             DEFAULT_MAX_DISTANCE,   fmt="%d")
    s_qmin     = make_slider(0.08, "Quality min",     *RANGE_QUALITY_MIN,
                             DEFAULT_QUALITY_MIN,    fmt="%d")
    s_hz       = make_slider(0.04, "Refresh rate (Hz)", *RANGE_UPDATE_HZ,
                             DEFAULT_UPDATE_HZ,      fmt="%d")

    def on_size(v):
        with params_lock:
            params["point_size"] = float(v)
        scatter.set_sizes(np.full(len(scatter.get_offsets()), float(v)))

    def on_lifetime(v):
        with params_lock:
            params["point_lifetime"] = float(v)

    def on_maxdist(v):
        with params_lock:
            params["max_distance"] = float(v)
        ax.set_ylim(0, float(v))
        update_yticks(float(v))
        heading_line.set_data([heading_line.get_xdata()[0]] * 2,
                              [0, float(v) * 0.25])

    def on_qmin(v):
        with params_lock:
            params["quality_min"] = float(v)

    def on_hz(v):
        with params_lock:
            params["update_hz"] = float(v)

    s_size.on_changed(on_size)
    s_lifetime.on_changed(on_lifetime)
    s_maxdist.on_changed(on_maxdist)
    s_qmin.on_changed(on_qmin)
    s_hz.on_changed(on_hz)

    plt.ion()
    plt.show()

    print("Visualizer running. Close the plot window to exit.")

    integrated_heading_deg = 0.0
    last_update = time.time()

    try:
        while plt.fignum_exists(fig.number):
            now = time.time()
            with params_lock:
                lifetime = params["point_lifetime"]
                size     = params["point_size"]
                maxd     = params["max_distance"]
                hz       = params["update_hz"]

            with state_lock:
                expired = [k for k, v in scan_dict.items() if now - v[3] > lifetime]
                for k in expired:
                    del scan_dict[k]

                points = list(scan_dict.values())
                imu    = dict(imu_state)

            if points:
                angles_rad = np.array([np.radians(p[0]) for p in points])
                dists      = np.array([p[1] for p in points])
                qualities  = np.array([p[2] for p in points])

                scatter.set_offsets(np.column_stack([angles_rad, dists]))
                scatter.set_array(qualities)
                scatter.set_sizes(np.full(len(points), size))
            else:
                scatter.set_offsets(np.empty((0, 2)))
                scatter.set_array(np.array([]))

            # Integrate gyro Z to estimate heading
            dt = now - last_update
            last_update = now
            integrated_heading_deg += imu["gz"] * dt
            integrated_heading_deg %= 360
            heading_rad = np.radians(integrated_heading_deg)
            heading_line.set_data([heading_rad, heading_rad], [0, maxd * 0.25])

            info_text.set_text(
                f"IMU\n"
                f"  Accel mg : X={imu['ax']:+5d}  Y={imu['ay']:+5d}  Z={imu['az']:+5d}\n"
                f"  Gyro dps : X={imu['gx']:+6.2f}  Y={imu['gy']:+6.2f}  Z={imu['gz']:+6.2f}\n"
                f"  Temp     : {imu['temp']:.2f} C\n"
                f"  Heading  : {integrated_heading_deg:6.1f} deg  (gyro-Z integrated)\n\n"
                f"LIDAR\n"
                f"  Live points    : {len(points)}\n"
                f"  Point lifetime : {lifetime * 1000:.0f} ms\n"
                f"  Range filter   : 0 - {int(maxd)} mm"
            )

            plt.pause(1.0 / hz)

    except KeyboardInterrupt:
        print("\nStopping visualizer.")
    finally:
        plt.close("all")


if __name__ == "__main__":
    main()


# """
# Sumobot LiDAR + IMU Visualizer
# Reads streaming data from STM32 over USB-serial (COM6 @ 115200)
# and displays a real-time polar plot with IMU heading overlay.

# Points expire after POINT_LIFETIME seconds — no ghosting trails.

# Usage:
#     pip install pyserial matplotlib numpy
#     python visualizer.py

# Press Ctrl+C in the terminal to stop.
# """

# import serial
# import re
# import threading
# import time

# import numpy as np
# import matplotlib.pyplot as plt

# # ─── Configuration ────────────────────────────────────────────
# SERIAL_PORT    = "COM6"
# BAUD_RATE      = 115200
# MAX_DISTANCE   = 2000     # mm — arena radius is 2m
# QUALITY_MIN    = 10       # filter out very low-quality points
# UPDATE_HZ      = 10       # plot refresh rate matches LiDAR scan rate
# POINT_LIFETIME = 0.6     # seconds — point disappears if not refreshed

# # ─── Shared state between serial thread and plot thread ───────
# scan_dict   = {}   # key = int(angle), value = (angle, dist, quality, timestamp)
# imu_state   = {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0, "temp": 0}
# state_lock  = threading.Lock()

# # ─── Regex parsers for the STM32 output format ────────────────
# LIDAR_RE = re.compile(
#     r"LIDAR\s*\|\s*Sample=\d+\s*\|\s*Angle=([-\d.]+)\s*deg\s*\|\s*"
#     r"Distance=([-\d.]+)\s*mm\s*\|\s*Quality=(\d+)"
# )
# IMU_RE = re.compile(
#     r"IMU\s*\|\s*ACC mg:\s*X=([-\d]+)\s*Y=([-\d]+)\s*Z=([-\d]+)\s*\|\s*"
#     r"GYRO dps:\s*X=([-\d.]+)\s*Y=([-\d.]+)\s*Z=([-\d.]+)\s*\|\s*"
#     r"TEMP=([-\d.]+)\s*C"
# )


# def serial_reader():
#     """Background thread: reads serial, parses LIDAR and IMU lines."""
#     try:
#         ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
#         print(f"Connected to {SERIAL_PORT} @ {BAUD_RATE}")
#     except serial.SerialException as e:
#         print(f"ERROR opening {SERIAL_PORT}: {e}")
#         print("Check that the STM32 is connected and PuTTY is closed.")
#         return

#     buffer = ""
#     while True:
#         try:
#             chunk = ser.read(256).decode("ascii", errors="ignore")
#         except Exception as e:
#             print(f"Serial read error: {e}")
#             break

#         if not chunk:
#             continue

#         buffer += chunk
#         while "\n" in buffer:
#             line, buffer = buffer.split("\n", 1)
#             line = line.strip()
#             if not line:
#                 continue

#             m = LIDAR_RE.search(line)
#             if m:
#                 angle = float(m.group(1))
#                 dist  = float(m.group(2))
#                 quality = int(m.group(3))
#                 # Filter: invalid angles (>=360), low quality, out-of-range
#                 if 0 <= angle < 360 and quality >= QUALITY_MIN and 0 < dist < MAX_DISTANCE:
#                     with state_lock:
#                         scan_dict[int(angle)] = (angle, dist, quality, time.time())
#                 continue

#             m = IMU_RE.search(line)
#             if m:
#                 with state_lock:
#                     imu_state["ax"]   = int(m.group(1))
#                     imu_state["ay"]   = int(m.group(2))
#                     imu_state["az"]   = int(m.group(3))
#                     imu_state["gx"]   = float(m.group(4))
#                     imu_state["gy"]   = float(m.group(5))
#                     imu_state["gz"]   = float(m.group(6))
#                     imu_state["temp"] = float(m.group(7))


# def main():
#     # Start serial reader thread
#     t = threading.Thread(target=serial_reader, daemon=True)
#     t.start()

#     # ─── Set up the polar plot ────────────────────────────────
#     plt.style.use("dark_background")
#     fig = plt.figure(figsize=(10, 10))
#     fig.canvas.manager.set_window_title("Sumobot LiDAR + IMU Visualizer")

#     ax = fig.add_subplot(111, projection="polar")
#     ax.set_theta_zero_location("N")     # 0 deg = up (forward)
#     ax.set_theta_direction(-1)           # clockwise
#     ax.set_ylim(0, MAX_DISTANCE)
#     ax.set_title("Sumobot view  —  LiDAR scan", color="white", pad=20, fontsize=14)
#     ax.grid(color="#444", alpha=0.5)
#     ax.tick_params(colors="#aaa")

#     # Range rings in meters
#     ax.set_yticks([500, 1000, 1500, 2000])
#     ax.set_yticklabels(["0.5m", "1m", "1.5m", "2m"])

#     # Robot center marker
#     ax.plot(0, 0, marker="o", color="#0ff", markersize=12, zorder=5)

#     # Scatter plot for LiDAR points (will be updated in loop)
#     scatter = ax.scatter([], [], s=10, c=[], cmap="plasma", vmin=0, vmax=63, alpha=0.9)

#     # IMU info text in corner
#     info_text = fig.text(
#         0.02, 0.96,
#         "",
#         color="#0ff", fontsize=10, family="monospace",
#         verticalalignment="top",
#     )

#     # IMU heading arrow (forward direction)
#     heading_line, = ax.plot([0, 0], [0, MAX_DISTANCE * 0.25], color="#0f0", linewidth=2, alpha=0.7)

#     plt.ion()
#     plt.show()

#     print("Visualizer running. Close the plot window to exit.")
#     print(f"Filtering: quality >= {QUALITY_MIN}, distance < {MAX_DISTANCE} mm")
#     print(f"Point lifetime: {POINT_LIFETIME} s")

#     integrated_heading_deg = 0.0
#     last_update = time.time()

#     try:
#         while plt.fignum_exists(fig.number):
#             now = time.time()

#             with state_lock:
#                 # Remove expired points so old data does not linger on screen
#                 expired = [k for k, v in scan_dict.items() if now - v[3] > POINT_LIFETIME]
#                 for k in expired:
#                     del scan_dict[k]

#                 points = list(scan_dict.values())
#                 imu    = dict(imu_state)

#             if points:
#                 angles_rad = np.array([np.radians(p[0]) for p in points])
#                 dists      = np.array([p[1] for p in points])
#                 qualities  = np.array([p[2] for p in points])

#                 scatter.set_offsets(np.column_stack([angles_rad, dists]))
#                 scatter.set_array(qualities)
#             else:
#                 scatter.set_offsets(np.empty((0, 2)))
#                 scatter.set_array(np.array([]))

#             # Integrate gyro Z to get a rough heading estimate (visual only)
#             dt = now - last_update
#             last_update = now
#             integrated_heading_deg += imu["gz"] * dt
#             integrated_heading_deg %= 360
#             heading_rad = np.radians(integrated_heading_deg)
#             heading_line.set_data([heading_rad, heading_rad], [0, MAX_DISTANCE * 0.25])

#             # Update info overlay
#             info_text.set_text(
#                 f"IMU\n"
#                 f"  Accel mg : X={imu['ax']:+5d}  Y={imu['ay']:+5d}  Z={imu['az']:+5d}\n"
#                 f"  Gyro dps : X={imu['gx']:+6.2f}  Y={imu['gy']:+6.2f}  Z={imu['gz']:+6.2f}\n"
#                 f"  Temp     : {imu['temp']:.2f} C\n"
#                 f"  Heading  : {integrated_heading_deg:6.1f} deg  (gyro-Z integrated)\n\n"
#                 f"LIDAR\n"
#                 f"  Live points    : {len(points)}\n"
#                 f"  Point lifetime : {POINT_LIFETIME * 1000:.0f} ms\n"
#                 f"  Range filter   : 0 - {MAX_DISTANCE} mm"
#             )

#             plt.pause(1.0 / UPDATE_HZ)

#     except KeyboardInterrupt:
#         print("\nStopping visualizer.")
#     finally:
#         plt.close("all")


# if __name__ == "__main__":
#     main()

# # """
# # Sumobot LiDAR + IMU Visualizer
# # Reads streaming data from STM32 over USB-serial (COM6 @ 115200)
# # and displays a real-time polar plot with IMU heading overlay.

# # Usage:
# #     pip install pyserial matplotlib numpy
# #     python visualizer.py

# # Press Ctrl+C in the terminal to stop.
# # """

# # import serial
# # import re
# # import threading
# # import time
# # from collections import deque

# # import numpy as np
# # import matplotlib.pyplot as plt
# # import matplotlib.patches as patches

# # # ─── Configuration ────────────────────────────────────────────
# # SERIAL_PORT  = "COM6"
# # BAUD_RATE    = 115200
# # MAX_DISTANCE = 2000      # mm — clip points beyond this for clarity
# # POINT_DECAY  = 180       # how many points to keep on screen
# # QUALITY_MIN  = 10        # filter out very low-quality points
# # UPDATE_HZ    = 10        # plot refresh rate

# # # ─── Shared state between serial thread and plot thread ───────
# # scan_points = deque(maxlen=POINT_DECAY)   # list of (angle_deg, dist_mm, quality)
# # imu_state   = {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0, "temp": 0}
# # state_lock  = threading.Lock()

# # # ─── Regex parsers for the STM32 output format ────────────────
# # LIDAR_RE = re.compile(
# #     r"LIDAR\s*\|\s*Sample=\d+\s*\|\s*Angle=([-\d.]+)\s*deg\s*\|\s*"
# #     r"Distance=([-\d.]+)\s*mm\s*\|\s*Quality=(\d+)"
# # )
# # IMU_RE = re.compile(
# #     r"IMU\s*\|\s*ACC mg:\s*X=([-\d]+)\s*Y=([-\d]+)\s*Z=([-\d]+)\s*\|\s*"
# #     r"GYRO dps:\s*X=([-\d.]+)\s*Y=([-\d.]+)\s*Z=([-\d.]+)\s*\|\s*"
# #     r"TEMP=([-\d.]+)\s*C"
# # )


# # def serial_reader():
# #     """Background thread: reads serial, parses LIDAR and IMU lines."""
# #     try:
# #         ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
# #         print(f"Connected to {SERIAL_PORT} @ {BAUD_RATE}")
# #     except serial.SerialException as e:
# #         print(f"ERROR opening {SERIAL_PORT}: {e}")
# #         print("Check that the STM32 is connected and PuTTY is closed.")
# #         return

# #     buffer = ""
# #     while True:
# #         try:
# #             chunk = ser.read(256).decode("ascii", errors="ignore")
# #         except Exception as e:
# #             print(f"Serial read error: {e}")
# #             break

# #         if not chunk:
# #             continue

# #         buffer += chunk
# #         # Lines may be separated by either \n or "LIDAR" appearing mid-stream
# #         # Split on newlines, process each
# #         while "\n" in buffer:
# #             line, buffer = buffer.split("\n", 1)
# #             line = line.strip()
# #             if not line:
# #                 continue

# #             m = LIDAR_RE.search(line)
# #             if m:
# #                 angle, dist, quality = float(m.group(1)), float(m.group(2)), int(m.group(3))
# #                 # Filter: invalid angles (>360), low quality, out-of-range
# #                 if 0 <= angle < 360 and quality >= QUALITY_MIN and 0 < dist < MAX_DISTANCE:
# #                     with state_lock:
# #                         scan_points.append((angle, dist, quality))
# #                 continue

# #             m = IMU_RE.search(line)
# #             if m:
# #                 with state_lock:
# #                     imu_state["ax"]   = int(m.group(1))
# #                     imu_state["ay"]   = int(m.group(2))
# #                     imu_state["az"]   = int(m.group(3))
# #                     imu_state["gx"]   = float(m.group(4))
# #                     imu_state["gy"]   = float(m.group(5))
# #                     imu_state["gz"]   = float(m.group(6))
# #                     imu_state["temp"] = float(m.group(7))


# # def main():
# #     # Start serial reader thread
# #     t = threading.Thread(target=serial_reader, daemon=True)
# #     t.start()

# #     # ─── Set up the polar plot ────────────────────────────────
# #     plt.style.use("dark_background")
# #     fig = plt.figure(figsize=(10, 10))
# #     fig.canvas.manager.set_window_title("Sumobot LiDAR + IMU Visualizer")

# #     ax = fig.add_subplot(111, projection="polar")
# #     ax.set_theta_zero_location("N")     # 0 deg = up (forward)
# #     ax.set_theta_direction(-1)           # clockwise
# #     ax.set_ylim(0, MAX_DISTANCE)
# #     ax.set_title("Sumobot view  —  LiDAR scan", color="white", pad=20, fontsize=14)
# #     ax.grid(color="#444", alpha=0.5)
# #     ax.tick_params(colors="#aaa")

# #     # Range rings in meters
# #     ax.set_yticks([500, 1000, 2000, 3000])
# #     ax.set_yticklabels(["0.5m", "1m", "2m", "3m"])

# #     # Robot center marker
# #     ax.plot(0, 0, marker="o", color="#0ff", markersize=12, zorder=5)

# #     # Scatter plot for LiDAR points (will be updated in loop)
# #     scatter = ax.scatter([], [], s=8, c=[], cmap="plasma", vmin=0, vmax=63, alpha=0.85)

# #     # IMU info text in corner
# #     info_text = fig.text(
# #         0.02, 0.96,
# #         "",
# #         color="#0ff", fontsize=10, family="monospace",
# #         verticalalignment="top",
# #     )

# #     # IMU heading arrow (forward direction)
# #     heading_line, = ax.plot([0, 0], [0, MAX_DISTANCE * 0.25], color="#0f0", linewidth=2, alpha=0.7)

# #     plt.ion()
# #     plt.show()

# #     print("Visualizer running. Close the plot window to exit.")
# #     print(f"Filtering: quality >= {QUALITY_MIN}, distance < {MAX_DISTANCE} mm")

# #     integrated_heading_deg = 0.0
# #     last_update = time.time()

# #     try:
# #         while plt.fignum_exists(fig.number):
# #             with state_lock:
# #                 points  = list(scan_points)
# #                 imu     = dict(imu_state)

# #             if points:
# #                 angles_rad = np.array([np.radians(p[0]) for p in points])
# #                 dists      = np.array([p[1] for p in points])
# #                 qualities  = np.array([p[2] for p in points])

# #                 scatter.set_offsets(np.column_stack([angles_rad, dists]))
# #                 scatter.set_array(qualities)

# #             # Integrate gyro Z to get a rough heading estimate (visual only)
# #             now = time.time()
# #             dt = now - last_update
# #             last_update = now
# #             integrated_heading_deg += imu["gz"] * dt
# #             integrated_heading_deg %= 360
# #             heading_rad = np.radians(integrated_heading_deg)
# #             heading_line.set_data([heading_rad, heading_rad], [0, MAX_DISTANCE * 0.25])

# #             # Update info overlay
# #             info_text.set_text(
# #                 f"IMU\n"
# #                 f"  Accel mg : X={imu['ax']:+5d}  Y={imu['ay']:+5d}  Z={imu['az']:+5d}\n"
# #                 f"  Gyro dps : X={imu['gx']:+6.2f}  Y={imu['gy']:+6.2f}  Z={imu['gz']:+6.2f}\n"
# #                 f"  Temp     : {imu['temp']:.2f} C\n"
# #                 f"  Heading  : {integrated_heading_deg:6.1f} deg  (gyro-Z integrated)\n\n"
# #                 f"LIDAR\n"
# #                 f"  Points displayed : {len(points)}\n"
# #                 f"  Range filter     : 0 - {MAX_DISTANCE} mm"
# #             )

# #             plt.pause(1.0 / UPDATE_HZ)

# #     except KeyboardInterrupt:
# #         print("\nStopping visualizer.")
# #     finally:
# #         plt.close("all")


# # if __name__ == "__main__":
# #     main()
