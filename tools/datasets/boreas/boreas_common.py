"""Shared helpers for the Boreas validation tooling (numpy only)."""

import glob
import os
import struct
import zlib

import numpy as np

# --- times ------------------------------------------------------------------


def seconds_text_to_ns(text):
    """Exact integer nanoseconds from a decimal seconds string such as
    '1614710304.8466012'. float64 would lose ~0.2 us at this magnitude."""
    text = text.strip()
    if "e" in text or "E" in text:
        raise ValueError(f"unexpected exponent in time '{text}'")
    whole, _, frac = text.partition(".")
    frac = (frac + "000000000")[:9] if len(frac) <= 9 else frac
    ns = int(whole) * 1_000_000_000
    if len(frac) > 9:  # round to the nearest ns
        ns += int(round(int(frac) / 10 ** (len(frac) - 9)))
    else:
        ns += int(frac)
    return ns


def stamp_files(directory, ext):
    """Sorted (time_us, path) for files named <time_us><ext>."""
    out = []
    for p in glob.glob(os.path.join(directory, "*" + ext)):
        out.append((int(os.path.basename(p)[: -len(ext)]), p))
    out.sort()
    return out


# --- rotations --------------------------------------------------------------


def C_enu_sensor(roll, pitch, heading):
    """Boreas pose angles to the rotation that maps sensor-frame vectors into
    ENU. Boreas' Applanix angles compose as Rx(roll) Ry(pitch) Rz(heading)
    with the passive elementary rotations below (each maps vectors into the
    rotated frame). convert.py confirms this numerically against
    lidar_poses.csv and camera_poses.csv."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    ch, sh = np.cos(heading), np.sin(heading)
    n = np.size(roll)
    Rx = np.zeros((n, 3, 3))
    Rx[:, 0, 0] = 1
    Rx[:, 1, 1], Rx[:, 1, 2], Rx[:, 2, 1], Rx[:, 2, 2] = cr, sr, -sr, cr
    Ry = np.zeros((n, 3, 3))
    Ry[:, 1, 1] = 1
    Ry[:, 0, 0], Ry[:, 0, 2], Ry[:, 2, 0], Ry[:, 2, 2] = cp, -sp, sp, cp
    Rz = np.zeros((n, 3, 3))
    Rz[:, 2, 2] = 1
    Rz[:, 0, 0], Rz[:, 0, 1], Rz[:, 1, 0], Rz[:, 1, 1] = ch, sh, -sh, ch
    return Rx @ Ry @ Rz


def rot_to_quat(R):
    """(n,3,3) rotation matrices to (n,4) unit quaternions w,x,y,z (w >= 0)."""
    R = np.asarray(R).reshape(-1, 3, 3)
    q = np.empty((len(R), 4))
    for i, M in enumerate(R):
        tr = np.trace(M)
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2
            q[i] = [0.25 * s, (M[2, 1] - M[1, 2]) / s, (M[0, 2] - M[2, 0]) / s,
                    (M[1, 0] - M[0, 1]) / s]
        elif M[0, 0] > M[1, 1] and M[0, 0] > M[2, 2]:
            s = np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2]) * 2
            q[i] = [(M[2, 1] - M[1, 2]) / s, 0.25 * s, (M[0, 1] + M[1, 0]) / s,
                    (M[0, 2] + M[2, 0]) / s]
        elif M[1, 1] > M[2, 2]:
            s = np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2]) * 2
            q[i] = [(M[0, 2] - M[2, 0]) / s, (M[0, 1] + M[1, 0]) / s, 0.25 * s,
                    (M[1, 2] + M[2, 1]) / s]
        else:
            s = np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1]) * 2
            q[i] = [(M[1, 0] - M[0, 1]) / s, (M[0, 2] + M[2, 0]) / s,
                    (M[1, 2] + M[2, 1]) / s, 0.25 * s]
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    q[q[:, 0] < 0] *= -1
    return q


def quat_to_rot(q):
    q = np.asarray(q, dtype=float).reshape(-1, 4)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - w * z)
    R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y)
    R[:, 2, 1] = 2 * (y * z + w * x)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def rot_angle_deg(Ra, Rb):
    """Angle of Ra^T Rb in degrees, for (n,3,3) stacks."""
    M = np.einsum("nji,njk->nik", Ra, Rb)
    c = np.clip((np.trace(M, axis1=1, axis2=2) - 1) / 2, -1, 1)
    # arccos loses precision near 0; use the antisymmetric part as well
    s = 0.5 * np.linalg.norm(np.stack([M[:, 2, 1] - M[:, 1, 2], M[:, 0, 2] - M[:, 2, 0],
                                       M[:, 1, 0] - M[:, 0, 1]], 1), axis=1)
    return np.degrees(np.arctan2(s, c))


def axis_angle(axis, deg):
    a = np.radians(deg)
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


# --- reference pose interpolation (numpy, independent of the engine) --------


def interp_poses(t_ns, pos, R, tq_ns):
    """Linear position and slerp rotation between the bracketing samples.
    Used only for the convention checks in convert.py."""
    t = np.asarray(t_ns, dtype=np.int64)
    tq = np.asarray(tq_ns, dtype=np.int64)
    k = np.searchsorted(t, tq, side="right") - 1
    if np.any(k < 0) or np.any(k >= len(t) - 1):
        raise ValueError("query time outside the pose samples")
    a = ((tq - t[k]) / (t[k + 1] - t[k]).astype(float))[:, None]
    p = pos[k] + a * (pos[k + 1] - pos[k])
    q0 = rot_to_quat(R[k])
    q1 = rot_to_quat(R[k + 1])
    d = np.sum(q0 * q1, 1)
    q1[d < 0] *= -1
    d = np.abs(d)
    th = np.arccos(np.clip(d, -1, 1))[:, None]
    small = th[:, 0] < 1e-9
    s = np.sin(th)
    s[small] = 1
    w0 = np.where(small[:, None], 1 - a, np.sin((1 - a) * th) / s)
    w1 = np.where(small[:, None], a, np.sin(a * th) / s)
    return p, quat_to_rot(w0 * q0 + w1 * q1)


# --- files ------------------------------------------------------------------


def load_matrix(path):
    return np.loadtxt(path)


def read_boreas_lidar(path):
    """(n,6) float32: x, y, z, intensity, laser_id, t (s, relative to the file
    time)."""
    return np.fromfile(path, dtype="<f4").reshape(-1, 6)


def read_pose_csv(path, time_unit):
    """Boreas pose CSV (gps_post_process.csv in seconds, *_poses.csv in
    microseconds) -> dict of arrays. Times are exact integer ns."""
    with open(path, encoding="utf-8") as f:
        header = f.readline().strip().split(",")
        rows = [line.rstrip("\n").split(",") for line in f if line.strip()]
    col = {h: i for i, h in enumerate(header)}
    if time_unit == "s":
        t = np.array([seconds_text_to_ns(r[0]) for r in rows], dtype=np.int64)
    else:
        t = np.array([int(r[0]) * 1000 for r in rows], dtype=np.int64)
    num = np.array([[float(x) for x in r[1:]] for r in rows])
    out = {"t_ns": t}
    for h, i in col.items():
        if i > 0:
            out[h] = num[:, i - 1]
    return out


def png_size(path):
    with open(path, "rb") as f:
        head = f.read(24)
    if head[:8] != b"\x89PNG\r\n\x1a\n" or head[12:16] != b"IHDR":
        raise ValueError(f"{path}: not a PNG")
    return struct.unpack(">II", head[16:24])


def write_png_rgb(path, img):
    """8-bit RGB PNG, no filtering (stdlib zlib only)."""
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w, _ = img.shape
    raw = np.zeros((h, 1 + 3 * w), np.uint8)
    raw[:, 1:] = img.reshape(h, 3 * w)

    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    data = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw.tobytes(), 6)) + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(data)


# --- statistics -------------------------------------------------------------


def nearest_rank(v, p):
    v = np.sort(np.asarray(v, dtype=float))
    if len(v) == 0:
        return float("nan")
    k = int(np.ceil(p / 100.0 * len(v)))
    return float(v[min(max(k, 1), len(v)) - 1])


def stats(v):
    v = np.asarray(v, dtype=float)
    if len(v) == 0:
        return {"n": 0}
    return {"n": int(len(v)), "median": float(np.median(v)), "p95": nearest_rank(v, 95),
            "max": float(np.max(v)), "mean": float(np.mean(v))}
