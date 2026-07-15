import queue
import threading
import time

try:
    import tkinter as tk
    from tkinter import ttk
except Exception:
    tk = None
    ttk = None


class CF2FlightMap:
    """
    Live map for Crazyflie estimated flight position.

    This displays the position estimate reported by the Crazyflie. With mocap
    disabled, accuracy is limited by the onboard estimator.
    """

    def __init__(
        self,
        flight_area_half_width_m,
        title="CF2 Live Flight Map",
        canvas_size_px=680,
        max_trail_points=1500,
        update_period_ms=50,
        prediction_horizon_s=2.0,
        prediction_step_s=0.10,
    ):
        self.flight_area_half_width_m = float(flight_area_half_width_m)
        self.title = title
        self.canvas_size_px = int(canvas_size_px)
        self.max_trail_points = int(max_trail_points)
        self.update_period_ms = int(update_period_ms)
        self.prediction_horizon_s = float(prediction_horizon_s)
        self.prediction_step_s = float(prediction_step_s)

        self._events = queue.Queue()
        self._thread = None
        self._running = False
        self._ui_ready = threading.Event()

        self._root = None
        self._canvas = None
        self._status_var = None
        self._position_var = None
        self._target_var = None
        self._error_var = None
        self._home_var = None

        self._positions = []
        self._current_position = None
        self._target_position = None
        self._home_position = None
        self._waypoints = []
        self._planned_trajectory = []
        self._last_redraw_s = 0.0

    def start(self):
        if tk is None:
            print("CF2FlightMap: tkinter is not available; live map disabled.")
            return

        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._run_ui, daemon=True)
        self._thread.start()
        self._ui_ready.wait(timeout=2.0)

    def close(self):
        if not self._running:
            return

        self._events.put(("close", None))
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
        self._thread = None

    def set_waypoints(self, waypoints):
        formatted = []
        for waypoint in waypoints:
            if len(waypoint) < 3:
                continue
            formatted.append((float(waypoint[0]), float(waypoint[1]), float(waypoint[2])))
        self._events.put(("waypoints", formatted))

    def set_planned_trajectory(self, trajectory):
        formatted = []
        for point in trajectory:
            if len(point) < 3:
                continue
            formatted.append((float(point[0]), float(point[1]), float(point[2])))
        self._events.put(("planned_trajectory", formatted))

    def set_target(self, x, y, z):
        self._events.put(("target", (float(x), float(y), float(z))))

    def set_home(self, x, y, z):
        self._events.put(("home", (float(x), float(y), float(z))))

    def update_position(self, x, y, z, host_time_s=None, vx=None, vy=None, vz=None):
        if host_time_s is None:
            host_time_s = time.time()
        self._events.put((
            "position",
            (
                float(x),
                float(y),
                float(z),
                float(host_time_s),
                self._optional_float(vx),
                self._optional_float(vy),
                self._optional_float(vz),
            ),
        ))

    def _run_ui(self):
        try:
            self._root = tk.Tk()
            self._root.title(self.title)
            self._root.protocol("WM_DELETE_WINDOW", self._request_close)
            self._build_ui()
            self._ui_ready.set()
            self._root.after(self.update_period_ms, self._drain_events)
            self._root.mainloop()
        except Exception as exc:
            print(f"CF2FlightMap: live map disabled: {exc}")
        finally:
            self._destroy_ui()
            self._running = False
            self._ui_ready.set()

    def _build_ui(self):
        self._root.configure(bg="#f5f7fb")

        main = ttk.Frame(self._root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")
        self._root.columnconfigure(0, weight=1)
        self._root.rowconfigure(0, weight=1)

        self._status_var = tk.StringVar(value="Waiting for position estimate...")
        self._position_var = tk.StringVar(value="Estimated position: --")
        self._target_var = tk.StringVar(value="Target: --")
        self._error_var = tk.StringVar(value="Target error: --")
        self._home_var = tk.StringVar(value="Home error: --")

        ttk.Label(main, textvariable=self._status_var, font=("Segoe UI", 11, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 6)
        )
        ttk.Label(main, textvariable=self._position_var).grid(row=1, column=0, sticky="w")
        ttk.Label(main, textvariable=self._target_var).grid(row=2, column=0, sticky="w")
        ttk.Label(main, textvariable=self._error_var).grid(row=1, column=1, sticky="e", padx=(18, 0))
        ttk.Label(main, textvariable=self._home_var).grid(row=2, column=1, sticky="e", padx=(18, 0))

        self._canvas = tk.Canvas(
            main,
            width=self.canvas_size_px,
            height=self.canvas_size_px,
            bg="#ffffff",
            highlightthickness=1,
            highlightbackground="#c9d1dc",
        )
        self._canvas.grid(row=3, column=0, columnspan=2, sticky="nsew", pady=(10, 0))

        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(3, weight=1)

        self._draw_static_map()

    def _request_close(self):
        self._events.put(("close", None))

    def _drain_events(self):
        should_close = False
        while True:
            try:
                event_name, payload = self._events.get_nowait()
            except queue.Empty:
                break

            if event_name == "close":
                should_close = True
            elif event_name == "waypoints":
                self._waypoints = payload
            elif event_name == "planned_trajectory":
                self._planned_trajectory = payload
            elif event_name == "target":
                self._target_position = payload
            elif event_name == "home":
                self._home_position = payload
            elif event_name == "position":
                self._current_position = payload
                if self._home_position is None:
                    self._home_position = payload[:3]
                self._positions.append(payload)
                if len(self._positions) > self.max_trail_points:
                    self._positions = self._positions[-self.max_trail_points:]

        if should_close:
            self._running = False
            self._destroy_ui()
            return

        now_s = time.time()
        if now_s - self._last_redraw_s >= self.update_period_ms / 1000.0:
            self._redraw()
            self._last_redraw_s = now_s

        if self._running and self._root is not None:
            self._root.after(self.update_period_ms, self._drain_events)

    def _destroy_ui(self):
        root = self._root

        self._status_var = None
        self._position_var = None
        self._target_var = None
        self._error_var = None
        self._home_var = None
        self._canvas = None
        self._root = None

        if root is not None:
            try:
                root.destroy()
            except tk.TclError:
                pass

    def _redraw(self):
        if self._canvas is None:
            return

        self._canvas.delete("dynamic")
        self._draw_planned_trajectory()
        self._draw_waypoints()
        self._draw_predicted_trajectory()
        self._draw_trail()
        self._draw_home()
        self._draw_target()
        self._draw_current_position()
        self._update_labels()

    def _draw_static_map(self):
        self._canvas.delete("static")
        size = self.canvas_size_px
        pad = 54
        left = pad
        top = pad
        right = size - pad
        bottom = size - pad

        self._canvas.create_rectangle(left, top, right, bottom, outline="#263238", width=2, tags="static")

        step_m = 0.10
        max_m = self.flight_area_half_width_m
        grid_count = int((2 * max_m) / step_m) + 1
        start_m = -grid_count * step_m / 2.0

        for index in range(grid_count + 1):
            value_m = start_m + index * step_m
            if value_m < -max_m or value_m > max_m:
                continue
            x0, y0 = self._to_canvas(value_m, -max_m)
            x1, y1 = self._to_canvas(value_m, max_m)
            self._canvas.create_line(x0, y0, x1, y1, fill="#e1e7ef", tags="static")

            x0, y0 = self._to_canvas(-max_m, value_m)
            x1, y1 = self._to_canvas(max_m, value_m)
            self._canvas.create_line(x0, y0, x1, y1, fill="#e1e7ef", tags="static")

        x0, y0 = self._to_canvas(-max_m, 0.0)
        x1, y1 = self._to_canvas(max_m, 0.0)
        self._canvas.create_line(x0, y0, x1, y1, fill="#7b8794", width=2, tags="static")

        x0, y0 = self._to_canvas(0.0, -max_m)
        x1, y1 = self._to_canvas(0.0, max_m)
        self._canvas.create_line(x0, y0, x1, y1, fill="#7b8794", width=2, tags="static")

        self._canvas.create_text(
            size / 2,
            16,
            text=self.title,
            fill="#263238",
            justify="center",
            width=size - 140,
            anchor="n",
            tags="static",
        )
        self._canvas.create_text(size - 24, size / 2, text="+X", fill="#263238", tags="static")
        self._canvas.create_text(size / 2, 62, text="+Y", fill="#263238", tags="static")
        self._draw_legend()

    def _draw_legend(self):
        left = 66
        top = self.canvas_size_px - 40
        items = [
            ("planned", "#80deea"),
            ("prediction", "#d81b60"),
            ("actual", "#1565c0"),
            ("target", "#fb8c00"),
            ("home", "#6a1b9a"),
        ]

        for index, (label, color) in enumerate(items):
            x = left + index * 112
            self._canvas.create_line(x, top, x + 28, top, fill=color, width=3, tags="static")
            self._canvas.create_text(x + 34, top, text=label, anchor="w", fill="#263238", tags="static")

    def _draw_planned_trajectory(self):
        if len(self._planned_trajectory) < 2:
            return

        points = [self._to_canvas(x, y) for x, y, _z in self._planned_trajectory]
        flattened = [coord for point in points for coord in point]
        self._canvas.create_line(
            *flattened,
            fill="#80deea",
            width=2,
            dash=(10, 6),
            tags="dynamic",
        )

    def _draw_predicted_trajectory(self):
        predicted = self._predict_future_positions()
        if len(predicted) < 2:
            return

        points = [self._to_canvas(x, y) for x, y, _z in predicted]
        flattened = [coord for point in points for coord in point]
        self._canvas.create_line(
            *flattened,
            fill="#d81b60",
            width=2,
            dash=(2, 4),
            tags="dynamic",
        )

    def _predict_future_positions(self):
        if self._target_position is None or len(self._positions) < 2:
            return []

        newest = self._positions[-1]
        position = [newest[0], newest[1], newest[2]]
        if len(newest) >= 7 and None not in newest[4:7]:
            velocity = [newest[4], newest[5], newest[6]]
        else:
            recent = [
                sample for sample in self._positions
                if newest[3] - sample[3] <= 0.75
            ]
            if len(recent) < 2:
                recent = self._positions[-2:]

            oldest = recent[0]
            elapsed_s = newest[3] - oldest[3]
            if elapsed_s <= 0:
                return []

            velocity = [
                (newest[0] - oldest[0]) / elapsed_s,
                (newest[1] - oldest[1]) / elapsed_s,
                (newest[2] - oldest[2]) / elapsed_s,
            ]
        target = list(self._target_position)

        predicted = [tuple(position)]
        step_s = max(0.02, self.prediction_step_s)
        steps = max(1, int(self.prediction_horizon_s / step_s))

        position_gain = 2.4
        velocity_damping = 1.7
        max_xy_speed_m_s = 0.8
        max_z_speed_m_s = 0.45

        for _ in range(steps):
            error = [
                target[0] - position[0],
                target[1] - position[1],
                target[2] - position[2],
            ]
            acceleration = [
                position_gain * error[0] - velocity_damping * velocity[0],
                position_gain * error[1] - velocity_damping * velocity[1],
                position_gain * error[2] - velocity_damping * velocity[2],
            ]

            velocity[0] += acceleration[0] * step_s
            velocity[1] += acceleration[1] * step_s
            velocity[2] += acceleration[2] * step_s

            xy_speed = (velocity[0] ** 2 + velocity[1] ** 2) ** 0.5
            if xy_speed > max_xy_speed_m_s:
                scale = max_xy_speed_m_s / xy_speed
                velocity[0] *= scale
                velocity[1] *= scale

            if abs(velocity[2]) > max_z_speed_m_s:
                velocity[2] = max_z_speed_m_s if velocity[2] > 0 else -max_z_speed_m_s

            position[0] += velocity[0] * step_s
            position[1] += velocity[1] * step_s
            position[2] += velocity[2] * step_s
            predicted.append(tuple(position))

        return predicted

    def _draw_waypoints(self):
        if not self._waypoints:
            return

        first_x, first_y, _first_z = self._waypoints[0]
        seen_positions = set()

        for index, (x, y, _z) in enumerate(self._waypoints, start=1):
            key = (round(x, 3), round(y, 3))
            if key in seen_positions:
                continue
            seen_positions.add(key)

            if index == 1 or self._same_xy((x, y), (first_x, first_y)):
                continue

            cx, cy = self._to_canvas(x, y)
            self._canvas.create_oval(
                cx - 5,
                cy - 5,
                cx + 5,
                cy + 5,
                fill="#ffffff",
                outline="#fb8c00",
                width=2,
                tags="dynamic",
            )

            label_dx = 14 if x >= 0 else -14
            label_dy = -14 if y >= 0 else 14
            anchor = "w" if label_dx > 0 else "e"
            self._draw_label(cx + label_dx, cy + label_dy, str(index), "#bf5f00", anchor=anchor)

    def _draw_trail(self):
        if len(self._positions) < 2:
            return

        points = [self._to_canvas(sample[0], sample[1]) for sample in self._positions]
        flattened = [coord for point in points for coord in point]
        self._canvas.create_line(*flattened, fill="#1565c0", width=3, tags="dynamic")

    def _draw_home(self):
        if self._home_position is None:
            return

        x, y, _z = self._home_position
        cx, cy = self._to_canvas(x, y)
        self._canvas.create_polygon(
            cx,
            cy - 10,
            cx + 10,
            cy,
            cx,
            cy + 10,
            cx - 10,
            cy,
            fill="#ffffff",
            outline="#6a1b9a",
            width=2,
            tags="dynamic",
        )
        self._draw_label(cx + 24, cy + 28, "HOME", "#6a1b9a", anchor="w")

    def _draw_target(self):
        if self._target_position is None:
            return

        x, y, _z = self._target_position
        cx, cy = self._to_canvas(x, y)

        if self._home_position is not None and self._same_xy((x, y), self._home_position[:2]):
            self._canvas.create_oval(cx - 16, cy - 16, cx + 16, cy + 16, outline="#fb8c00", width=2, tags="dynamic")
            return

        self._canvas.create_line(cx - 9, cy, cx + 9, cy, fill="#fb8c00", width=3, tags="dynamic")
        self._canvas.create_line(cx, cy - 9, cx, cy + 9, fill="#fb8c00", width=3, tags="dynamic")
        self._canvas.create_oval(cx - 11, cy - 11, cx + 11, cy + 11, outline="#fb8c00", width=2, tags="dynamic")

    def _draw_current_position(self):
        if self._current_position is None:
            return

        x, y, _z = self._current_position[:3]
        cx, cy = self._to_canvas(x, y)

        if self._target_position is not None:
            tx, ty, _tz = self._target_position
            tx_px, ty_px = self._to_canvas(tx, ty)
            self._canvas.create_line(cx, cy, tx_px, ty_px, fill="#90a4ae", dash=(3, 3), tags="dynamic")

        self._canvas.create_oval(cx - 8, cy - 8, cx + 8, cy + 8, fill="#2e7d32", outline="#1b5e20", width=2, tags="dynamic")

    def _same_xy(self, first_xy, second_xy, tolerance_m=0.025):
        dx = first_xy[0] - second_xy[0]
        dy = first_xy[1] - second_xy[1]
        return (dx ** 2 + dy ** 2) ** 0.5 <= tolerance_m

    def _draw_label(self, x, y, text, fill, anchor="center"):
        text_id = self._canvas.create_text(
            x,
            y,
            text=text,
            fill=fill,
            anchor=anchor,
            font=("Segoe UI", 9, "bold"),
            tags="dynamic",
        )
        bbox = self._canvas.bbox(text_id)
        if bbox is None:
            return

        pad = 3
        background_id = self._canvas.create_rectangle(
            bbox[0] - pad,
            bbox[1] - pad,
            bbox[2] + pad,
            bbox[3] + pad,
            fill="#ffffff",
            outline="",
            tags="dynamic",
        )
        self._canvas.tag_raise(text_id, background_id)

    def _update_labels(self):
        if self._current_position is None:
            self._status_var.set("Waiting for position estimate...")
            return

        x, y, z = self._current_position[:3]
        self._status_var.set("Live estimated position from Crazyflie telemetry")
        self._position_var.set(f"Estimated position: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")

        if self._target_position is None:
            self._target_var.set("Target: --")
            self._error_var.set("Target error: --")
        else:
            tx, ty, tz = self._target_position
            error_xy_m = ((x - tx) ** 2 + (y - ty) ** 2) ** 0.5
            error_z_m = abs(z - tz)
            self._target_var.set(f"Target: x={tx:.3f} m, y={ty:.3f} m, z={tz:.3f} m")
            self._error_var.set(f"Target error: xy={error_xy_m * 100:.1f} cm, z={error_z_m * 100:.1f} cm")

        if self._home_position is None:
            self._home_var.set("Home error: --")
        else:
            hx, hy, hz = self._home_position
            home_xy_m = ((x - hx) ** 2 + (y - hy) ** 2) ** 0.5
            home_z_m = abs(z - hz)
            self._home_var.set(f"Home error: xy={home_xy_m * 100:.1f} cm, z={home_z_m * 100:.1f} cm")

    def _to_canvas(self, x_m, y_m):
        size = self.canvas_size_px
        pad = 54
        usable = size - 2 * pad
        scale = usable / (2 * self.flight_area_half_width_m)
        center = size / 2
        return center + x_m * scale, center - y_m * scale

    def _optional_float(self, value):
        if value is None:
            return None
        return float(value)
