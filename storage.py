import csv
import re

import numpy as np

from engine import LOOPS, KEYS

PARAM_COLS = [g + "." + k for g in LOOPS for k in KEYS]


class History:
    def __init__(self, capacity=20000, channels=4):
        self.capacity = capacity
        self.data = np.empty((capacity, 1 + channels))
        self.params = [()] * capacity
        self.end = self.size = 0

    def reset(self):
        self.end = self.size = 0

    def add(self, t, values, params):
        row = self.data[self.end]
        row[:] = np.nan
        row[0] = t
        row[1:1+len(values)] = values
        self.params[self.end] = params
        self.end = (self.end + 1) % self.capacity
        self.size = min(self.capacity, self.size + 1)

    def slices(self):
        if self.size == self.capacity:
            return (slice(self.end, self.capacity), slice(0, self.end))
        return (slice(0, self.end),)

    def view(self, copy=False):
        slices = self.slices()
        a = np.concatenate([self.data[s] for s in slices]) if len(slices) == 2 else self.data[slices[0]]
        return a.copy() if copy else a

    def snapshots(self):
        return [p for s in self.slices() for p in self.params[s]]


def csv_header(names, references=()):
    return ["time_s", *[f"CH{i+1}_{name}" for i, name in enumerate(names)],
            *[f"REF_{name}" for name in references], *PARAM_COLS]


def csv_row(t, values, params, references=()):
    cols = [float(x) if np.isfinite(x) else "" for x in values]
    return [round(t, 6), *cols, *([""] * (4 - len(cols))),
            *[float(x) if np.isfinite(x) else "" for x in references], *params]


def _parse_channel_header(raw_header, fallback):
    text = raw_header.strip()
    m = re.match(r"^CH[1-4]_(.+)$", text, flags=re.IGNORECASE)
    if m:
        text = m.group(1).strip()
    unit = ""
    um = re.match(r"^(.+?)\s*\[([^\]]+)\]\s*$", text)
    if um:
        text, unit = um.group(1).strip(), um.group(2).strip()[:16]
    return (text or fallback)[:32], unit


def load_csv(path, max_rows=20000):
    """Load a PIDTuner CSV (or generic time + 1..4 channel CSV) for offline replay.

    Returns dict with keys: data (Nx9 ndarray), count, aliases, units, references, params.
    """
    with open(path, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.reader(f)
        try:
            header = next(reader)
        except StopIteration:
            raise ValueError("CSV 文件为空")
        rows = [r for r in reader if any(cell.strip() for cell in r)]

    if not rows:
        raise ValueError("CSV 文件没有数据行")
    if len(rows) > max_rows:
        rows = rows[-max_rows:]

    # Detect PIDTuner native columns vs generic columns
    ch_cols = []
    aliases = ["", "", "", ""]
    units = ["", "", "", ""]
    ref_cols = []
    param_indices = {}

    for idx, col_name in enumerate(header):
        stripped = col_name.strip()
        if idx == 0:
            continue
        if re.match(r"^CH([1-4])_", stripped, flags=re.IGNORECASE):
            slot = int(stripped[2]) - 1
            ch_cols.append((slot, idx))
            aliases[slot], units[slot] = _parse_channel_header(stripped, f"CH{slot+1}")
        elif stripped.upper().startswith("REF_") and len(ref_cols) < 4:
            ref_cols.append((stripped[4:].strip()[:32] or f"参考线 {len(ref_cols)+1}", idx))
        elif stripped in PARAM_COLS:
            param_indices[stripped] = idx

    # Generic CSV fallback: col 0 = time (or sample index), cols 1..4 = CH1..CH4
    if not ch_cols:
        for slot in range(min(4, max(0, len(header) - 1))):
            ch_cols.append((slot, slot + 1))
            aliases[slot], units[slot] = _parse_channel_header(header[slot + 1], f"CH{slot+1}")

    data = np.full((len(rows), 9), np.nan, dtype=float)
    params = []
    valid_rows = 0
    max_channel_seen = 0

    for r_i, row in enumerate(rows):
        try:
            t = float(row[0]) if row[0].strip() else float(r_i) * 0.01
        except ValueError:
            continue
        if not np.isfinite(t):
            continue
        data[valid_rows, 0] = t
        for slot, col_idx in ch_cols:
            if col_idx < len(row) and row[col_idx].strip():
                try:
                    val = float(row[col_idx])
                    if np.isfinite(val):
                        data[valid_rows, 1 + slot] = val
                        max_channel_seen = max(max_channel_seen, slot + 1)
                except ValueError:
                    pass
        for ref_slot, (_, col_idx) in enumerate(ref_cols):
            if col_idx < len(row) and row[col_idx].strip():
                try:
                    val = float(row[col_idx])
                    if np.isfinite(val):
                        data[valid_rows, 5 + ref_slot] = val
                except ValueError:
                    pass
        if param_indices:
            snap = []
            for p_col in PARAM_COLS:
                ci = param_indices.get(p_col)
                cell = row[ci].strip() if ci is not None and ci < len(row) else ""
                if cell:
                    try:
                        snap.append(float(cell))
                    except ValueError:
                        snap.append("")
                else:
                    snap.append("")
            params.append(tuple(snap))
        else:
            params.append(("",) * len(PARAM_COLS))
        valid_rows += 1

    if valid_rows == 0 or max_channel_seen == 0:
        raise ValueError("未在 CSV 中解析到有效的数值通道")

    data = data[:valid_rows]
    references = []
    for ref_slot, (r_name, _) in enumerate(ref_cols):
        col_vals = data[:, 5 + ref_slot]
        finite_vals = col_vals[np.isfinite(col_vals)]
        last_val = float(finite_vals[-1]) if len(finite_vals) else 0.0
        references.append((r_name, last_val))

    return {
        "data": data,
        "count": max_channel_seen,
        "aliases": aliases,
        "units": units,
        "references": references,
        "params": params,
    }
