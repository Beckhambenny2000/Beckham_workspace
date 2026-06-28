import pyzed.sl as sl
import cv2
import numpy as np
from ultralytics import YOLO
import time
import socket
import threading
import itertools
import math
from dataclasses import dataclass
from typing import Optional, List, Tuple

# ── PyVista 3D Packing Visualizer ─────────────────────────────────────────────
try:
    import pyvista as pv
    PYVISTA_AVAILABLE = True
except ImportError:
    print("[WARNING] pyvista not installed. 3D visualizer disabled. Run: pip install pyvista")
    PYVISTA_AVAILABLE = False


# ── Visualizer config ──────────────────────────────────────────────────────────
WIN_SIZE   = (1600, 820)
BG_COLOR   = "#12122A"
BG_TOP     = "#1E1E3A"
VIEW1      = (-50,  22)
VIEW2      = ( 130, 22)
RENDER_GAP = 0.15
EPS        = 1e-6
ALLOW_ROTATION = True
RETRY_UNPACKED = True

ITEM_COLORS = [
    "#D94F3D", "#3D85C8", "#3DAD6E", "#E6A817", "#8B5CF6",
    "#E8602C", "#1AA3A3", "#C2375B", "#5B8A00", "#2C6FAC",
]

# ── Main-thread PyVista state ─────────────────────────────────────────────────
_pl         = None
_pl_pending = None


# ── Data class ────────────────────────────────────────────────────────────────
@dataclass
class Item:
    name: str
    w: float
    d: float
    h: float
    volume: float
    position: Optional[Tuple[float, float, float]] = None
    rotation: Optional[Tuple[float, float, float]] = None

    @property
    def placed_w(self): return self.rotation[0] if self.rotation else self.w
    @property
    def placed_d(self): return self.rotation[1] if self.rotation else self.d
    @property
    def placed_h(self): return self.rotation[2] if self.rotation else self.h

    def rotations(self):
        seen = set()
        for p in itertools.permutations([self.w, self.d, self.h]):
            if p not in seen:
                seen.add(p)
                yield p


# ── ExtremePoint Packer ────────────────────────────────────────────────────────
class ExtremePointPacker:
    def __init__(self, W, D, H):
        self.W, self.D, self.H = W, D, H
        self.packed: List[Item] = []
        self.unpacked: List[Item] = []

    def pack_all(self, items):
        def key(i): return (i.w * i.d * i.h, max(i.w * i.d, i.w * i.h, i.d * i.h))
        order  = sorted(items, key=key, reverse=True)
        failed = [it for it in order if not self._place(it)]
        if RETRY_UNPACKED and failed:
            self.unpacked = [it for it in failed if not self._place(it)]
        else:
            self.unpacked = failed

    def utilisation(self):
        return sum(i.placed_w * i.placed_d * i.placed_h for i in self.packed) / (self.W * self.D * self.H)

    def _place(self, item):
        rots = list(item.rotations()) if ALLOW_ROTATION else [(item.w, item.d, item.h)]
        best = None
        xs = sorted({0.0} | {p.position[0] for p in self.packed} | {p.position[0] + p.placed_w for p in self.packed})
        ys = sorted({0.0} | {p.position[1] for p in self.packed} | {p.position[1] + p.placed_d for p in self.packed})
        for rot in rots:
            iw, id_, ih = rot
            if iw > self.W + EPS or id_ > self.D + EPS or ih > self.H + EPS:
                continue
            for x in xs:
                if x + iw > self.W + EPS:
                    continue
                for y in ys:
                    if y + id_ > self.D + EPS:
                        continue
                    z = self._support_z(x, y, iw, id_)
                    if z + ih > self.H + EPS:
                        continue
                    if self._overlaps_any(x, y, z, iw, id_, ih):
                        continue
                    score = (round(z, 6), round(x, 6), round(y, 6), -round(iw * id_, 4))
                    if best is None or score < best[0]:
                        best = (score, x, y, z, rot)
        if best is None:
            return False
        _, x, y, z, rot = best
        item.position = (x, y, z)
        item.rotation = rot
        self.packed.append(item)
        return True

    def _support_z(self, x, y, iw, id_):
        z = 0.0
        for p in self.packed:
            px, py, pz = p.position
            pw, pd, ph = p.placed_w, p.placed_d, p.placed_h
            if px + pw > x + EPS and px < x + iw - EPS and py + pd > y + EPS and py < y + id_ - EPS:
                z = max(z, pz + ph)
        return z

    def _overlaps_any(self, x, y, z, iw, id_, ih):
        for p in self.packed:
            px, py, pz = p.position
            pw, pd, ph = p.placed_w, p.placed_d, p.placed_h
            if (x + iw > px + EPS and x < px + pw - EPS and
                    y + id_ > py + EPS and y < py + pd - EPS and
                    z + ih > pz + EPS and z < pz + ph - EPS):
                return True
        return False


# ── Visualizer helpers ─────────────────────────────────────────────────────────
def _fmt_vol(cm3):
    if cm3 >= 1_000_000: return f"{cm3 / 1_000_000:.2f} m3 ({cm3:,.0f} cm3)"
    if cm3 >= 1_000:     return f"{cm3 / 1_000:.2f} L ({cm3:,.0f} cm3)"
    return f"{cm3:,.0f} cm3"


def _hex2rgb(h):
    h = h.lstrip("#")
    return (int(h[0:2], 16) / 255, int(h[2:4], 16) / 255, int(h[4:6], 16) / 255)


def _cam_pos(W, D, H, azim, elev):
    cx, cy, cz = W / 2, D / 2, H / 2
    dist = max(W, D, H) * 3.5
    e, a = math.radians(elev), math.radians(azim)
    return [
        (cx + dist * math.cos(e) * math.cos(a),
         cy + dist * math.cos(e) * math.sin(a),
         cz + dist * math.sin(e)),
        (cx, cy, cz),
        (0, 0, 1),
    ]


def _build_scene(pl, packer, W, D, H, azim, elev, title, show_legend):
    pts = [(0,0,0),(W,0,0),(W,D,0),(0,D,0),(0,0,H),(W,0,H),(W,D,H),(0,D,H)]
    for a, b in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
        seg = pv.Line(pts[a], pts[b])
        pl.add_mesh(seg, color="#A0784A", line_width=2.0, opacity=0.8)

    legend_entries = []
    for idx, item in enumerate(packer.packed):
        chex = ITEM_COLORS[idx % len(ITEM_COLORS)]
        rgb  = _hex2rgb(chex)
        x, y, z = item.position
        iw, id_, ih = item.placed_w, item.placed_d, item.placed_h
        g  = RENDER_GAP
        bx, by, bz = x + g, y + g, z + g
        bw = max(iw - 2 * g, g)
        bd = max(id_ - 2 * g, g)
        bh = max(ih - 2 * g, g)

        mesh = pv.Box(bounds=(bx, bx + bw, by, by + bd, bz, bz + bh))
        pl.add_mesh(mesh, color=rgb, opacity=0.97, smooth_shading=False,
                    show_edges=True, edge_color="white", line_width=0.8)

        pl.add_point_labels(
            np.array([[x + iw / 2, y + id_ / 2, z + ih / 2]]),
            [f"#{idx+1} {item.name[:10]}"],
            font_size=7, text_color="white", bold=True,
            show_points=False, shape=None, always_visible=True,
        )

        legend_entries.append([
            f"#{idx+1}  {item.name}  "
            f"({iw:.1f}x{id_:.1f}x{ih:.1f} cm)  {_fmt_vol(item.volume)}",
            chex
        ])

    pl.add_text(title, position="upper_edge", font_size=10, color="#AAAACC")

    if show_legend and legend_entries:
        pl.add_legend(
            legend_entries, bcolor=None, border=False, loc="upper left",
            size=(0.45, min(0.01 * len(legend_entries) + 0.04, 0.35)),
            face="rectangle",
        )

    pl.camera_position = _cam_pos(W, D, H, azim, elev)


def viz_apply_pending():
    global _pl, _pl_pending

    if not PYVISTA_AVAILABLE:
        return

    if _pl is None and _pl_pending is None:
        return

    if _pl is None and _pl_pending is not None:
        box_name, box_dims, _, scan_num = _pl_pending
        BL, BW, BH = box_dims
        _pl = pv.Plotter(
            shape=(1, 2),
            window_size=list(WIN_SIZE),
            off_screen=False,
            title=f"Packing Visualizer - {box_name} Box",
        )
        _pl.set_background(BG_COLOR, top=BG_TOP)
        _pl.show(interactive_update=True, auto_close=False)

    if _pl_pending is not None:
        box_name, box_dims, item_dims_list, scan_num = _pl_pending
        _pl_pending = None

        BL, BW, BH = box_dims
        items = [
            Item(name=f"Item {i+1}", w=l, d=w, h=h, volume=l * w * h)
            for i, (l, w, h) in enumerate(item_dims_list)
        ]
        packer = ExtremePointPacker(BL, BW, BH)
        if items:
            packer.pack_all(items)

        util  = packer.utilisation() * 100
        W, D, H = BL, BW, BH
        cv_vol  = W * D * H
        pv_vol  = sum(i.placed_w * i.placed_d * i.placed_h for i in packer.packed)

        try:
            _pl.title = (
                f"Packing Visualizer - {box_name} Box ({BL}x{BW}x{BH} cm)  |  "
                f"Scan #{scan_num}  |  {util:.1f}% fill  |  "
                f"{len(packer.packed)}/{len(items)} item(s)"
            )

            _pl.renderers[0].clear()
            _pl.renderers[1].clear()

            _pl.subplot(0, 0)
            if not packer.packed:
                _pl.add_text(
                    "No items could be packed\n(items may exceed box dimensions)",
                    position="upper_left", font_size=11, color="red"
                )
            else:
                _build_scene(_pl, packer, W, D, H,
                             VIEW1[0], VIEW1[1], "Front-Right View", show_legend=True)

            _pl.add_text(
                f"Box: {box_name}  {W:.0f}x{D:.0f}x{H:.0f} cm  |  "
                f"Packed: {_fmt_vol(pv_vol)}  |  "
                f"Wasted: {_fmt_vol(cv_vol - pv_vol)}  |  "
                f"Fill: {util:.1f}%",
                position="lower_left", font_size=7, color="#CCCCEE",
            )
            if packer.unpacked:
                _pl.add_text(
                    f"Didn't fit: {', '.join(i.name for i in packer.unpacked)}",
                    position="lower_right", font_size=7, color="#FF6B6B",
                )

            _pl.subplot(0, 1)
            if packer.packed:
                _build_scene(_pl, packer, W, D, H,
                             VIEW2[0], VIEW2[1], "Back-Left View (Opposite)", show_legend=False)
            else:
                _pl.add_text("No items packed", position="upper_left",
                             font_size=11, color="red")

        except Exception as e:
            print(f"[VIZ] Redraw error: {e}")

    try:
        _pl.update(stime=1)
    except Exception:
        _pl = None


def request_pyvista_update(box_name, box_dims, item_dims_list, scan_num=0):
    global _pl_pending
    if not PYVISTA_AVAILABLE:
        return
    _pl_pending = (box_name, box_dims, list(item_dims_list), scan_num)


# ─────────────────────────────────────────────────────────────────────────────

model = YOLO("yolov8n.pt")

# Parameters
FRAME_AVERAGE           = 60
DEPTH_TOLERANCE         = 0.03
GRID_STEPS              = 20
TRACKING_DIST_THRESHOLD = 80
CONFIDENCE_THRESHOLD    = 0.08
DISPLAY_DEADZONE_CM     = 2.0

MAX_BOX_FRAME_COVERAGE = 0.60

# Box sizes (L, W, H) in cm
BOX_SIZES = {
    "Small":  (23.0, 16.0, 8.0),
    "Medium": (19.5, 14.5, 14.0),
    "Large":  (29.4, 19.4, 18.7),
}
BOX_VOLUME_TOLERANCE = 0.99
BOX_ORDER = ["Small", "Medium", "Large"]

CEILING_CM = 32.2 # max distance from camera to base

# ── TCP Server ────────────────────────────────────────────────────────────────
TCP_HOST         = "0.0.0.0"
TCP_PORT         = 5005
_tcp_running     = True
tcp_clients      = []
tcp_clients_lock = threading.Lock()


def tcp_server_thread():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(5)
    server.settimeout(1.0)
    print(f"TCP server listening on {TCP_HOST}:{TCP_PORT}")
    while _tcp_running:
        try:
            conn, addr = server.accept()
            print(f"TCP client connected: {addr}")
            with tcp_clients_lock:
                tcp_clients.append(conn)
        except socket.timeout:
            continue
        except OSError:
            break
    server.close()


def tcp_broadcast(message: str):
    data = (message + "\n").encode()
    with tcp_clients_lock:
        dead = []
        for c in tcp_clients:
            try:
                c.sendall(data)
            except OSError:
                dead.append(c)
        for c in dead:
            tcp_clients.remove(c)


# ── Standard helpers ──────────────────────────────────────────────────────────
def get_box_volumes():
    return {name: (d[0]*d[1]*d[2])*BOX_VOLUME_TOLERANCE for name, d in BOX_SIZES.items()}

BOX_VOLUMES = get_box_volumes()


def _count_dim_violations(item_dims, box_name):
    item_sorted = sorted(item_dims)
    box_sorted  = sorted(BOX_SIZES[box_name])
    return sum(1 for i in range(3) if item_sorted[i] > box_sorted[i])


def _item_fits_in_box(item_dims, box_name):
    violations = _count_dim_violations(item_dims, box_name)

    if violations == 0:
        return True, "Upright"

    if violations == 1:
        longest_item = max(item_dims)
        L, W, H = BOX_SIZES[box_name]
        box_diagonal = math.sqrt(L**2 + W**2 + H**2)
        if longest_item <= box_diagonal:
            return True, "Slanted"
        return False, "No fit"

    return False, "No fit"


def check_item_fit(w, h, l):
    for name in BOX_ORDER:
        fits, fit_type = _item_fits_in_box((w, h, l), name)
        if fits:
            return name, fit_type
    return None, "Rejected"


def get_overall_recommendation(item_fit_results, total_volume_cm3,
                                item_dims_list=None):
    if not item_fit_results:
        return None, "No items detected"
    if any(ft == "Rejected" for _, ft in item_fit_results):
        return None, "Items too large for all boxes - cannot recommend"

    for name in BOX_ORDER:

        if total_volume_cm3 > BOX_VOLUMES[name]:
            continue

        if item_dims_list:
            all_fit = True
            for dims in item_dims_list:
                fits, _ = _item_fits_in_box(dims, name)
                if not fits:
                    all_fit = False
                    break
            if not all_fit:
                continue

        return name, "OK"

    return None, "Item dimensions or total volume exceed all box sizes - cannot recommend"


def round_to_grid(v, grid=0.3):
    """Snap value to nearest grid increment (default 0.3 cm)."""
    return round(round(v / grid) * grid, 6)


def compute_dims_3d(points_3d):
    """
    Compute W, L, H purely from the 3D point cloud in camera space.

    - H: Z spread between the object top surface and the table surface
         (CEILING_CM * 10 mm). Uses real ZED world coords so off-centre
         objects are measured correctly — no perspective error.
    - W, L: PCA on real-world X, Y (mm) restricted to top-surface points
         only, which prevents side-face contamination from inflating dims.

    Works correctly for objects anywhere in the frame, not just directly
    below the camera.

    Tuning knobs:
      top_z_mm + 15.0  — increase if object top surface is noisy (more pts
                         included); decrease if side faces still leak in.
      percentile 5     — for top surface Z; increase to 10 if very noisy.
      percentile 2/98  — for W/L spread; widen to 1/99 for sparse point clouds.
    """
    if len(points_3d) < 10:
        return 0.0, 0.0, 0.0

    pts = np.array(points_3d, dtype=np.float64)  # units = mm

    # ── Height ───────────────────────────────────────────────────────────────
    # ZED top-down: smaller Z = closer to camera = top of object.
    # Table surface is at CEILING_CM * 10 mm from the camera.
    table_z_mm = CEILING_CM * 10.0
    top_z_mm   = np.percentile(pts[:, 2], 5)   # robust top surface (5th pct)
    h_cm       = max(0.0, (table_z_mm - top_z_mm) / 10.0)

    # ── Width & Length ───────────────────────────────────────────────────────
    # Filter to top-surface points only (within 15 mm of the top surface)
    # to exclude side faces that leak in when objects are off-centre.
    top_mask = pts[:, 2] <= (top_z_mm + 15.0)
    top_pts  = pts[top_mask] if top_mask.sum() >= 5 else pts

    xy  = top_pts[:, :2]
    cen = xy - np.mean(xy, axis=0)

    if cen.shape[0] < 2:
        return 0.0, 0.0, h_cm

    cov      = np.cov(cen.T)
    ev, evec = np.linalg.eigh(cov)
    evec     = evec[:, np.argsort(ev)[::-1]]
    proj     = cen @ evec

    # 2nd–98.9th percentile to reject stray outlier points
    ew = (np.percentile(proj[:, 0], 98.9) - np.percentile(proj[:, 0], 2)) / 10.0
    eh = (np.percentile(proj[:, 1], 98.9) - np.percentile(proj[:, 1], 2)) / 10.0

    return ew, eh, h_cm


def filter_out_container(boxes, fw, fh):
    fa = fw * fh
    out = []
    for b in boxes:
        x1, y1, x2, y2 = b.xyxy[0].cpu().numpy()
        if ((x2 - x1) * (y2 - y1)) / fa <= MAX_BOX_FRAME_COVERAGE:
            out.append(b)
    return out


# ── IoU deduplication ─────────────────────────────────────────────────────────
def iou(box_a, box_b):
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1 = max(ax1, bx1);  iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2);  iy2 = min(ay2, by2)
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    if inter == 0:
        return 0.0
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    return inter / (area_a + area_b - inter)

IOU_DEDUP_THRESHOLD = 0.45


# ── Pause overlay ─────────────────────────────────────────────────────────────
def draw_pause_overlay(frame, fit_text, fit_color, vol_text, elapsed, pause_dur):
    overlay = frame.copy()
    fh, fw  = frame.shape[:2]

    panel_x1, panel_y1 = fw // 6, fh // 4
    panel_x2, panel_y2 = fw * 5 // 6, fh * 3 // 4
    cv2.rectangle(overlay, (panel_x1, panel_y1), (panel_x2, panel_y2), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.78, frame, 0.22, 0, frame)

    cv2.rectangle(frame, (panel_x1, panel_y1), (panel_x2, panel_y2), fit_color, 3)

    cx = (panel_x1 + panel_x2) // 2

    title = "SCANNING COMPLETE"
    (tw, th), _ = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.85, 2)
    cv2.putText(frame, title,
                (cx - tw // 2, panel_y1 + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2, cv2.LINE_AA)

    (tw, th), _ = cv2.getTextSize(vol_text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
    cv2.putText(frame, vol_text,
                (cx - tw // 2, panel_y1 + 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2, cv2.LINE_AA)

    (tw, th), _ = cv2.getTextSize(fit_text, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)
    cv2.putText(frame, fit_text,
                (cx - tw // 2, panel_y1 + 125),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, fit_color, 2, cv2.LINE_AA)

    remaining   = max(0.0, pause_dur - elapsed)
    bar_x1      = panel_x1 + 20
    bar_x2      = panel_x2 - 20
    bar_y       = panel_y2 - 30
    bar_fill    = int((bar_x2 - bar_x1) * (remaining / pause_dur))
    cv2.rectangle(frame, (bar_x1, bar_y - 12), (bar_x2, bar_y + 12), (60, 60, 60), -1)
    cv2.rectangle(frame, (bar_x1, bar_y - 12),
                  (bar_x1 + bar_fill, bar_y + 12), fit_color, -1)
    cnt_text = f"Next scan in {remaining:.1f}s"
    (tw, th), _ = cv2.getTextSize(cnt_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    cv2.putText(frame, cnt_text,
                (cx - tw // 2, bar_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)


def main():
    global _tcp_running

    t = threading.Thread(target=tcp_server_thread, daemon=True)
    t.start()

    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.MILLIMETER
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Camera failed")
        exit()

    runtime     = sl.RuntimeParameters()
    image       = sl.Mat()
    point_cloud = sl.Mat()
    tracks      = {}

    print("Camera running... Press Q to exit")

    DETECT_DURATION = 3.0
    PAUSE_DURATION  = 1.5
    cycle_start     = time.time()
    in_pause        = False
    pause_frame     = None
    pause_rec       = ("", (255, 255, 255), "")

    last_frame_summary = None
    last_item_dims     = []
    scan_num           = 0

    while True:
        now_t   = time.time()
        elapsed = now_t - cycle_start

        viz_apply_pending()

        if in_pause:
            if elapsed >= PAUSE_DURATION:
                in_pause    = False
                cycle_start = now_t
                elapsed     = 0.0
        else:
            if elapsed >= DETECT_DURATION:
                scan_num += 1
                tcp_broadcast("SCAN_COMPLETE|1")
                if last_frame_summary is not None:
                    tcp_broadcast(last_frame_summary)

                viable_box_viz = None
                if last_item_dims:
                    fit_results_viz = [check_item_fit(l, w, h) for (l, w, h) in last_item_dims]
                    total_vol_viz   = sum(l * w * h for l, w, h in last_item_dims)
                    viable_box_viz, _ = get_overall_recommendation(
                        fit_results_viz, total_vol_viz, last_item_dims
                    )

                if viable_box_viz and last_item_dims:
                    request_pyvista_update(
                        viable_box_viz,
                        BOX_SIZES[viable_box_viz],
                        last_item_dims,
                        scan_num,
                    )

                in_pause    = True
                cycle_start = now_t
                elapsed     = 0.0

        if in_pause:
            if pause_frame is not None:
                display = pause_frame.copy()
                draw_pause_overlay(display, *pause_rec, elapsed, PAUSE_DURATION)
                cv2.imshow("ZED Object Dimensions", display)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
            continue

        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(image, sl.VIEW.LEFT)
        frame_rgb = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        results          = model(frame_rgb, verbose=False, conf=CONFIDENCE_THRESHOLD, imgsz=640)
        total_volume_cm3 = 0.0
        new_tracks       = {}
        any_rejected     = False
        item_fit_results = []
        current_item_dims = []
        fh, fw            = frame_rgb.shape[:2]

        all_boxes = []
        for result in results:
            all_boxes.extend(filter_out_container(result.boxes, fw, fh))

        all_boxes.sort(key=lambda b: float(b.conf[0]), reverse=True)
        kept_boxes = []
        for candidate in all_boxes:
            c_xyxy = candidate.xyxy[0].cpu().numpy()
            if not any(iou(c_xyxy, k.xyxy[0].cpu().numpy()) > IOU_DEDUP_THRESHOLD
                       for k in kept_boxes):
                kept_boxes.append(candidate)

        for box in kept_boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
            cx, cy          = (x1 + x2) // 2, (y1 + y2) // 2

            err, cp = point_cloud.get_value(cx, cy)
            if err != sl.ERROR_CODE.SUCCESS or not np.all(np.isfinite(cp[:3])):
                continue

            z_lo = cp[2] * (1.0 - DEPTH_TOLERANCE)
            z_hi = cp[2] * (1.0 + DEPTH_TOLERANCE)
            roi_h, roi_w = y2 - y1, x2 - x1
            depth_mask   = np.zeros((roi_h, roi_w), dtype=np.uint8)
            step_x       = max(1, roi_w // GRID_STEPS)
            step_y       = max(1, roi_h // GRID_STEPS)
            all_points   = {}

            for i in range(x1, x2, step_x):
                for j in range(y1, y2, step_y):
                    err, pt = point_cloud.get_value(i, j)
                    if err == sl.ERROR_CODE.SUCCESS and np.all(np.isfinite(pt[:3])):
                        mx = min(int((i - x1) * roi_w / (x2 - x1)), roi_w - 1)
                        my = min(int((j - y1) * roi_h / (y2 - y1)), roi_h - 1)
                        if z_lo <= pt[2] <= z_hi:
                            depth_mask[my, mx] = 255
                            all_points[(mx, my)] = pt[:3]

            dims_cm   = (0.0, 0.0, 0.0)
            points_3d = list(all_points.values())
            cp_pts    = cv2.findNonZero(depth_mask)

            if cp_pts is not None and len(cp_pts) >= 5:
                rect    = cv2.minAreaRect(cp_pts)
                corners = cv2.boxPoints(rect).astype(np.int32)
                rm      = np.zeros_like(depth_mask)
                cv2.fillPoly(rm, [corners], 255)
                rp3d = [p for (mx, my), p in all_points.items() if rm[my, mx] == 255]

                if len(rp3d) >= 10:
                    pw, ph, ph_h = compute_dims_3d(rp3d)
                    dims_cm = (pw, ph, ph_h)

                cf = corners.copy()
                cf[:, 0] += x1; cf[:, 1] += y1
                cv2.polylines(frame_rgb, [cf], True, (255, 255, 0), 2)

            elif len(points_3d) >= 10:
                pw, ph, ph_h = compute_dims_3d(points_3d)
                dims_cm = (pw, ph, ph_h)

            matched_key = None
            best_dist   = TRACKING_DIST_THRESHOLD
            for key in tracks:
                kx, ky = key
                d = np.hypot(cx - kx, cy - ky)
                if d < best_dist:
                    best_dist, matched_key = d, key

            prev    = tracks.get(matched_key, {"history": [], "display": None})
            history = prev["history"]
            history.append(dims_cm)
            if len(history) > FRAME_AVERAGE:
                history.pop(0)

            med  = np.median(history, axis=0)
            pd   = prev["display"]
            disp = med if pd is None else np.where(
                np.abs(med - np.array(pd)) > DISPLAY_DEADZONE_CM, med, pd)

            new_tracks[(cx, cy)] = {"history": history, "display": disp}

            l = round_to_grid(disp[0])
            w = round_to_grid(disp[1])
            h = round_to_grid(disp[2])


            fit_box, fit_type = check_item_fit(w, h, l)
            volume_cm3        = l * w * h
            total_volume_cm3 += volume_cm3
            item_fit_results.append((fit_box, fit_type))
            current_item_dims.append((l, w, h))

            if fit_type == "Upright":
                box_color = (0, 255, 0);   fit_label = f"Fits {fit_box} (Upright)"
            elif fit_type == "Slanted":
                box_color = (0, 165, 255); fit_label = f"Fits {fit_box} (Slanted)"
            else:
                box_color = (0, 0, 220);   fit_label = "Rejected - too large"
                any_rejected = True

            cv2.rectangle(frame_rgb, (x1, y1), (x2, y2), box_color, 2)
            cv2.circle(frame_rgb, (cx, cy), 4, (0, 0, 255), -1)

            cv2.putText(frame_rgb,
                        f"L:{l:.1f} W:{w:.1f} H:{h:.1f} cm | Vol:{volume_cm3:.1f}cm3",
                        (x1, y2 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
            cv2.putText(frame_rgb, fit_label,
                        (x1, y2 + 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

            tcp_broadcast(
                f"OBJECT|l:{l:.1f},w:{w:.1f},h:{h:.1f},vol:{volume_cm3:.1f}"
            )

        tracks = new_tracks

        if current_item_dims:
            last_item_dims = current_item_dims

        viable_box, reason = get_overall_recommendation(
            item_fit_results, total_volume_cm3, current_item_dims
        )
        if viable_box:
            bd        = BOX_SIZES[viable_box]
            fit_text  = (f"Recommended: {viable_box} "
                         f"({bd[0]}x{bd[1]}x{bd[2]} cm) "
                         f"| BoxVol: {BOX_VOLUMES[viable_box]:.1f} cm3")
            fit_color = {"Small": (0,255,0), "Medium": (0,165,255),
                         "Large": (0,0,255)}[viable_box]
        else:
            fit_text, fit_color = reason, (0, 0, 220)

        cv2.rectangle(frame_rgb, (0, 0), (frame_rgb.shape[1], 65), (0, 0, 0), -1)
        cv2.putText(frame_rgb,
                    f"Vol:{total_volume_cm3:.1f}cm3 | "
                    f"Objs:{len(item_fit_results)} | "
                    f"Conf:{CONFIDENCE_THRESHOLD:.0%}",
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(frame_rgb, fit_text,
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, fit_color, 2)
        if any_rejected:
            cv2.putText(frame_rgb, "WARNING: Item(s) too large for all boxes!",
                        (10, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 220), 2)

        frame_summary_msg = (
            f"FRAME|objs:{len(item_fit_results)},"
            f"total_vol:{total_volume_cm3:.1f},"
            f"recommended:{viable_box or 'None'}"
        )
        tcp_broadcast(frame_summary_msg)
        last_frame_summary = frame_summary_msg

        remaining_detect = DETECT_DURATION - elapsed
        if remaining_detect <= 0.15:
            pause_frame = frame_rgb.copy()
            vol_text    = (f"Total Volume: {total_volume_cm3:.1f} cm3  |  "
                           f"Objects: {len(item_fit_results)}")
            pause_rec   = (fit_text, fit_color, vol_text)

        scan_remaining = max(0.0, DETECT_DURATION - elapsed)
        scan_text      = f"Scanning... {scan_remaining:.1f}s"
        (stw, sth), _  = cv2.getTextSize(scan_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        sx = frame_rgb.shape[1] - stw - 10
        sy = frame_rgb.shape[0] - 10
        cv2.putText(frame_rgb, scan_text, (sx, sy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("ZED Object Dimensions", frame_rgb)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    _tcp_running = False
    if _pl is not None:
        try:
            _pl.close()
        except Exception:
            pass
    zed.close()
    cv2.destroyAllWindows()
    print("Session ended.")


if __name__ == "__main__":
    main()