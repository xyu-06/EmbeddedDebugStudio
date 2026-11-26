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


def csv_row(t, values, params, references=(), ch_cols=None):
    cols = [float(x) if np.isfinite(x) else "" for x in values]
    width = ch_cols if ch_cols is not None else max(4, len(cols))
    padded = cols[:width] + [""] * max(0, width - len(cols))
    return [round(t, 6), *padded,
            *[float(x) if np.isfinite(x) else "" for x in references], *params]


def _parse_channel_header(raw_header, fallback):
    text = raw_header.strip()
    m = re.match(r"^CH([1-8])_(.+)$", text, flags=re.IGNORECASE)
    if m:
        text = m.group(2).strip()
    unit = ""
    um = re.match(r"^(.+?)\s*\[([^\]]+)\]\s*$", text)
    if um:
        text, unit = um.group(1).strip(), um.group(2).strip()[:16]
    return (text or fallback)[:32], unit


def inspect_csv(path):
    """Stream-scan a CSV file without retaining rows in RAM; returns metadata and time span."""
    total_rows = 0
    t_start = None
    t_end = None
    with open(path, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.reader(f)
        try:
            header = next(reader)
        except StopIteration:
            raise ValueError("CSV 文件为空")
        for r_i, row in enumerate(reader):
            if not any(cell.strip() for cell in row):
                continue
            try:
                t_val = float(row[0]) if row[0].strip() else float(r_i) * 0.01
            except ValueError:
                continue
            if not np.isfinite(t_val):
                continue
            total_rows += 1
            if t_start is None:
                t_start = t_val
            t_end = t_val
    if total_rows == 0:
        raise ValueError("CSV 文件没有数据行")
    return {
        "header": header,
        "total_rows": total_rows,
        "t_start": t_start,
        "t_end": t_end,
        "duration_s": max(0.0, float(t_end - t_start)),
    }


def load_csv(path, max_rows=20000, t_start=None, t_end=None):
    """Stream-load a PIDTuner CSV (or generic time + 1..8 channel CSV) for offline replay.

    Supports optional time window [t_start, t_end] and bounded deque(maxlen=max_rows)
    so arbitrarily large recordings do not blow up RAM.
    """
    from collections import deque

    selected = deque(maxlen=max(1, int(max_rows)))
    total_rows = 0
    matched_rows = 0
    file_t_start = None
    file_t_end = None

    with open(path, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.reader(f)
        try:
            header = next(reader)
        except StopIteration:
            raise ValueError("CSV 文件为空")
        for r_i, row in enumerate(reader):
            if not any(cell.strip() for cell in row):
                continue
            try:
                t_val = float(row[0]) if row[0].strip() else float(r_i) * 0.01
            except ValueError:
                continue
            if not np.isfinite(t_val):
                continue
            total_rows += 1
            if file_t_start is None:
                file_t_start = t_val
            file_t_end = t_val
            if t_start is not None and t_val < float(t_start) - 1e-9:
                continue
            if t_end is not None and t_val > float(t_end) + 1e-9:
                continue
            matched_rows += 1
            selected.append((t_val, row))

    if total_rows == 0:
        raise ValueError("CSV 文件没有数据行")
    if not selected:
        raise ValueError(f"在时间段 [{t_start}, {t_end}] 内没有有效数据行")

    rows = list(selected)

    # Detect PIDTuner native columns vs generic columns
    ch_cols = []
    aliases = [""] * 8
    units = [""] * 8
    ref_cols = []
    param_indices = {}

    for idx, col_name in enumerate(header):
        stripped = col_name.strip()
        if idx == 0:
            continue
        m = re.match(r"^CH([1-8])_", stripped, flags=re.IGNORECASE)
        if m:
            slot = int(m.group(1)) - 1
            ch_cols.append((slot, idx))
            aliases[slot], units[slot] = _parse_channel_header(stripped, f"CH{slot+1}")
        elif stripped.upper().startswith("REF_") and len(ref_cols) < 4:
            ref_cols.append((stripped[4:].strip()[:32] or f"参考线 {len(ref_cols)+1}", idx))
        elif stripped in PARAM_COLS:
            param_indices[stripped] = idx

    # Generic CSV fallback: col 0 = time (or sample index), cols 1..8 = CH1..CH8
    if not ch_cols:
        for slot in range(min(8, max(0, len(header) - 1))):
            ch_cols.append((slot, slot + 1))
            aliases[slot], units[slot] = _parse_channel_header(header[slot + 1], f"CH{slot+1}")

    data = np.full((len(rows), 13), np.nan, dtype=float)
    params = []
    valid_rows = 0
    max_channel_seen = 0

    for r_i, (t, row) in enumerate(rows):
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
                        data[valid_rows, 9 + ref_slot] = val
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
        col_vals = data[:, 9 + ref_slot]
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
        "total_rows": total_rows,
        "matched_rows": matched_rows,
        "loaded_rows": valid_rows,
        "truncated": matched_rows > valid_rows or matched_rows < total_rows,
        "window_start": float(data[0, 0]),
        "window_end": float(data[-1, 0]),
        "file_t_start": float(file_t_start),
        "file_t_end": float(file_t_end),
    }
