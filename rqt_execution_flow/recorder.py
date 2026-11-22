"""Record raw events and play them back at any speed.

A recording keeps the raw event tuples coming out of ``RosMonitor`` (their
wall-clock time is at index 1) and the ROS graph snapshots seen while
recording.  Replaying feeds them into a fresh ``Tracker`` in time order, so
the replay shows exactly what the live view would have shown.

No ROS or Qt imports, so it can be unit tested on its own.
"""

import json
from dataclasses import fields

from .tracker import GraphSnapshot

FORMAT = 'rqt_execution_flow.recording'
VERSION = 1
MAX_EVENTS = 500_000     # roughly 100-200 MB of RAM; recording stops there


def graph_to_dict(g: GraphSnapshot) -> dict:
    out = {}
    for f in fields(g):
        value = getattr(g, f.name)
        if isinstance(value, set):
            out[f.name] = sorted(value)
        else:
            out[f.name] = {k: sorted(v) if isinstance(v, set) else v
                           for k, v in sorted(value.items())}
    return out


def graph_from_dict(d: dict) -> GraphSnapshot:
    g = GraphSnapshot()
    for f in fields(g):
        if f.name not in d:
            continue
        value = d[f.name]
        current = getattr(g, f.name)
        if isinstance(current, set):
            setattr(g, f.name, set(value))
        elif f.name.endswith('_types'):
            setattr(g, f.name, dict(value))
        else:
            setattr(g, f.name, {k: set(v) for k, v in value.items()})
    return g


class Recording:

    def __init__(self, start: float):
        self.start = start
        self.end = start
        self.events = []         # raw event tuples, time at index 1
        self.graphs = []         # [(t, GraphSnapshot)]
        self._last_graph = None

    @property
    def duration(self) -> float:
        return self.end - self.start

    @property
    def full(self) -> bool:
        return len(self.events) >= MAX_EVENTS

    def add_event(self, ev):
        if not self.full:
            self.events.append(ev)
            self.end = max(self.end, ev[1])

    def add_graph(self, t, graph: GraphSnapshot):
        as_dict = graph_to_dict(graph)
        if as_dict != self._last_graph:
            self._last_graph = as_dict
            self.graphs.append((t, graph))
            self.end = max(self.end, t)

    def finish(self, t):
        """Call when recording stops: sorts events, sets the end time."""
        self.events.sort(key=lambda e: e[1])   # stable, keeps same-time order
        self.end = max(self.end, t)

    # ------------------------------------------------------------- files
    def save(self, path):
        data = {
            'format': FORMAT,
            'version': VERSION,
            'start': self.start,
            'end': self.end,
            'graphs': [[t, graph_to_dict(g)] for t, g in self.graphs],
            'events': [list(e) for e in self.events],
        }
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, separators=(',', ':'))

    @classmethod
    def load(cls, path) -> 'Recording':
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
        if data.get('format') != FORMAT:
            raise ValueError('not an rqt_execution_flow recording')
        if data.get('version', 0) > VERSION:
            raise ValueError('recording was made by a newer version of this plugin')
        rec = cls(float(data['start']))
        rec.graphs = [(float(t), graph_from_dict(g)) for t, g in data['graphs']]
        rec.events = [tuple(e) for e in data['events']]
        rec.finish(float(data['end']))
        return rec


class Player:
    """Walks through a recording.  ``pos`` is the current recorded time."""

    def __init__(self, recording: Recording):
        self.rec = recording
        self.reset()

    def reset(self):
        self.pos = self.rec.start
        self._ei = 0
        self._gi = 0

    @property
    def offset(self) -> float:
        return self.pos - self.rec.start

    @property
    def at_end(self) -> bool:
        return self._ei >= len(self.rec.events) and self.pos >= self.rec.end

    def skip_to(self, t):
        """Jump forward without returning anything (used to skip quiet time)."""
        self.pos = max(self.pos, min(t, self.rec.end))

    def next_event_time(self):
        if self._ei < len(self.rec.events):
            return self.rec.events[self._ei][1]
        return None

    def advance_to(self, t):
        """Everything recorded up to time ``t``, in order.

        Returns a list of ``('graph', GraphSnapshot)`` and ``('event', tuple)``.
        A graph snapshot comes before events with the same time.
        """
        t = min(t, self.rec.end)
        out = []
        events, graphs = self.rec.events, self.rec.graphs
        while True:
            next_g = graphs[self._gi][0] if self._gi < len(graphs) else None
            next_e = events[self._ei][1] if self._ei < len(events) else None
            if next_g is not None and next_g <= t and (next_e is None or next_g <= next_e):
                out.append(('graph', graphs[self._gi][1]))
                self._gi += 1
            elif next_e is not None and next_e <= t:
                out.append(('event', events[self._ei]))
                self._ei += 1
            else:
                break
        self.pos = max(self.pos, t)
        return out

    def step(self):
        """Jump to the next event and return it (plus any graph change before it)."""
        t = self.next_event_time()
        if t is None:
            self.pos = self.rec.end
            return []
        return self.advance_to(t)
