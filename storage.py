import csv

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

