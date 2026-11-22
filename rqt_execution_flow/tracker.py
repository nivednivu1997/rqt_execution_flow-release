"""Turn raw ROS events into "operations" (action goals and service calls).

This module has no ROS or Qt imports so it can be unit tested on its own.

Input events are plain tuples produced by ``monitor.RosMonitor``:

* ``('goal_status', t, action, goal_id_hex, status_code[, baseline])``
* ``('feedback', t, action, goal_id_hex, feedback_text_or_None)``
* ``('service_event', t, service, event_code, client_node_or_None,
  client_id, sequence_number, payload_text_or_None, stamp)``

``Tracker.handle()`` returns a list of ``UiEvent`` objects telling the UI what
to draw: new rows, packets to animate, timeline log lines, and so on.

Parent/child links ("this service call was made while that goal was running")
are *inferred*: ROS 2 Jazzy does not carry a call id from a goal to the calls
its server makes.  The rule is: if the caller of a new operation is itself the
server of an operation that is running right now, the running one is the parent.
"""

import itertools
import math
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

ACTION = 'action'
SERVICE = 'service'

UNKNOWN_NODE = '(unknown)'

GOAL_STATUS = {
    0: 'UNKNOWN',
    1: 'ACCEPTED',
    2: 'EXECUTING',
    3: 'CANCELING',
    4: 'SUCCEEDED',
    5: 'CANCELED',
    6: 'ABORTED',
}
TERMINAL_STATUS = {'SUCCEEDED', 'CANCELED', 'ABORTED'}

REQUEST_SENT, REQUEST_RECEIVED, RESPONSE_SENT, RESPONSE_RECEIVED = 0, 1, 2, 3
SERVICE_EVENT = {
    REQUEST_SENT: 'REQUEST SENT',
    REQUEST_RECEIVED: 'REQUEST RECEIVED',
    RESPONSE_SENT: 'RESPONSE SENT',
    RESPONSE_RECEIVED: 'RESPONSE RECEIVED',
}

# Seconds between two packets of the same operation, so they don't overlap.
PACKET_SPACING = 0.35
# Minimum seconds between two animated feedback packets of the same goal.
FEEDBACK_PACKET_INTERVAL = 0.15


@dataclass
class GraphSnapshot:
    """Who serves and who calls what, as seen in the ROS graph."""

    action_servers: Dict[str, Set[str]] = field(default_factory=dict)
    action_clients: Dict[str, Set[str]] = field(default_factory=dict)
    action_types: Dict[str, str] = field(default_factory=dict)
    service_servers: Dict[str, Set[str]] = field(default_factory=dict)
    service_clients: Dict[str, Set[str]] = field(default_factory=dict)
    service_types: Dict[str, str] = field(default_factory=dict)
    # services that publish introspection events (<service>/_service_event)
    introspected: Set[str] = field(default_factory=set)


@dataclass(eq=False)
class Operation:
    uid: int
    kind: str                      # ACTION or SERVICE
    name: str                      # action / service name
    key: tuple
    start: float
    short_id: str = ''             # goal id prefix or "seq N"
    type: str = ''
    client: Optional[str] = None
    client_note: str = ''
    server: Optional[str] = None
    server_note: str = ''
    status: str = ''
    end: Optional[float] = None
    outcome: str = ''              # '', 'ok', 'failed', 'canceled'
    cancel_requested: bool = False
    feedback_count: int = 0
    last_feedback: str = ''
    request_text: str = ''
    response_text: str = ''
    stamps: Dict[int, float] = field(default_factory=dict)
    history: List[Tuple[float, str]] = field(default_factory=list)
    parent: Optional['Operation'] = None
    parent_note: str = ''
    children: List['Operation'] = field(default_factory=list)
    service_phase: int = -1
    partial: bool = False          # we only saw the end, so the start time is unknown
    next_packet_time: float = 0.0
    last_feedback_packet: float = 0.0

    @property
    def active(self) -> bool:
        return self.end is None

    @property
    def label(self) -> str:
        return f'{self.name} #{self.short_id}' if self.short_id else self.name

    def duration(self, now: float) -> float:
        return (self.end if self.end is not None else now) - self.start

    def edge_key(self) -> tuple:
        return (self.kind, self.name, self.client or UNKNOWN_NODE, self.server or UNKNOWN_NODE)


@dataclass
class CallStats:
    """Counts and timings for one action or service name, since the last clear."""

    name: str
    kind: str
    type: str = ''
    calls: int = 0
    ok: int = 0
    canceled: int = 0
    failed: int = 0
    durations: 'OrderedDict[int, float]' = field(default_factory=OrderedDict)

    MAX_SAMPLES = 5000

    def record(self, uid: int, seconds: float):
        self.durations[uid] = seconds
        self.durations.move_to_end(uid)
        while len(self.durations) > self.MAX_SAMPLES:
            self.durations.popitem(last=False)

    def timing(self, slow_limit: float) -> dict:
        """avg, p95 and max in seconds (None without samples), and how many were slow."""
        values = sorted(self.durations.values())
        if not values:
            return {'avg': None, 'p95': None, 'max': None, 'slow': 0}
        return {
            'avg': sum(values) / len(values),
            # nearest-rank percentile: never below the average with few samples
            'p95': values[max(0, math.ceil(0.95 * len(values)) - 1)],
            'max': values[-1],
            'slow': sum(1 for v in values if v > slow_limit),
        }


@dataclass
class UiEvent:
    kind: str                      # 'new', 'update', 'packet', 'log', 'removed'
    op: Operation
    packet: str = ''               # packet type for kind == 'packet'
    delay: float = 0.0
    text: str = ''
    verbose: bool = False          # log lines hidden unless the user wants them
    t: float = 0.0


class Tracker:

    def __init__(self, max_roots: int = 400):
        self.graph = GraphSnapshot()
        self.max_roots = max_roots
        self.clear()

    # ------------------------------------------------------------------ api
    def clear(self):
        self.by_key: Dict[tuple, Operation] = {}
        self.by_uid: Dict[int, Operation] = {}
        self.roots: List[Operation] = []
        self.active: List[Operation] = []
        self._unresolved: Dict[int, int] = {}   # op uid -> graph updates tried
        self.stats: Dict[str, CallStats] = {}   # action / service name -> stats
        self._uid = itertools.count(1)

    def set_graph(self, graph: GraphSnapshot) -> List[Operation]:
        """Store the new graph. Returns running calls whose caller it now reveals.

        Right after startup a goal can arrive before discovery has told us who
        can send it; this fills the caller in once the graph catches up.
        """
        self.graph = graph
        changed = []
        for uid in list(self._unresolved):
            op = self.by_uid.get(uid)
            tries = self._unresolved[uid] + 1
            if op is None or (op.client and op.server and op.type) or tries > 5:
                del self._unresolved[uid]
                continue
            self._unresolved[uid] = tries
            is_action = op.kind == ACTION
            if not op.type:
                types = graph.action_types if is_action else graph.service_types
                op.type = types.get(op.name, '')
                if op.type and op.name in self.stats:
                    self.stats[op.name].type = op.type
            if op.server is None:
                servers = graph.action_servers if is_action else graph.service_servers
                server, _ = self._pick(servers.get(op.name), 'server')
                if server is not None:
                    op.server, op.server_note = server, 'found after the call started'
            if op.client is None:
                clients = graph.action_clients if is_action else graph.service_clients
                client, note = self._pick_client(clients.get(op.name), op.server)
                if client is not None:
                    op.client, op.client_note = client, note or 'found after the call started'
            if op.client is not None or op.server is not None:
                changed.append(op)
                self._attach_late(op)
                changed.extend(self._adopt_orphans(op))
            if op.client and op.server and op.type:
                del self._unresolved[uid]
        return changed

    def handle(self, ev: tuple) -> List[UiEvent]:
        kind = ev[0]
        if kind == 'goal_status':
            return self._on_goal_status(*ev[1:])
        if kind == 'feedback':
            return self._on_feedback(*ev[1:])
        if kind == 'service_event':
            return self._on_service_event(*ev[1:])
        return []

    # -------------------------------------------------------------- actions
    def _on_goal_status(self, t, action, goal_id, code, baseline=False) -> List[UiEvent]:
        status = GOAL_STATUS.get(code, str(code))
        out: List[UiEvent] = []
        op = self.by_key.get((ACTION, action, goal_id))
        if op is None:
            if status == 'UNKNOWN':
                return out
            if baseline and status in TERMINAL_STATUS:
                return out   # finished before we started watching
            op = self._new_action_op(t, action, goal_id, out)
        if op.status == status:
            return out
        prev = op.status
        op.status = status
        op.history.append((t, status))

        if status == 'ACCEPTED':
            self._packet(op, 'accepted', t, out)
            self._log(op, t, 'Goal accepted', out)
        elif status == 'EXECUTING':
            if prev == '':
                # first seen while already executing: show the accept too
                self._packet(op, 'accepted', t, out)
            self._log(op, t, 'Executing', out)
        elif status == 'CANCELING':
            op.cancel_requested = True
            self._packet(op, 'cancel', t, out)
            self._log(op, t, 'Cancel requested', out)
        elif status in TERMINAL_STATUS and op.end is None:
            if status == 'CANCELED' and not op.cancel_requested:
                # we missed the CANCELING state, still show the cancel
                op.cancel_requested = True
                self._packet(op, 'cancel', t, out)
                self._log(op, t, 'Cancel requested', out)
            packet = {'SUCCEEDED': 'result_ok', 'CANCELED': 'result_canceled',
                      'ABORTED': 'result_failed'}[status]
            outcome = {'SUCCEEDED': 'ok', 'CANCELED': 'canceled', 'ABORTED': 'failed'}[status]
            self._packet(op, packet, t, out)
            self._finish(op, t, outcome)
            self._log(op, t, f'Result: {status}', out)
        out.append(UiEvent('update', op, t=t))
        return out

    def _on_feedback(self, t, action, goal_id, text) -> List[UiEvent]:
        out: List[UiEvent] = []
        op = self.by_key.get((ACTION, action, goal_id))
        if op is None:
            op = self._new_action_op(t, action, goal_id, out)
            op.status = 'EXECUTING'
            op.partial = True
            op.history.append((t, 'EXECUTING (seen via feedback)'))
        if op.end is not None:
            return out
        op.feedback_count += 1
        if text:
            op.last_feedback = text
        if op.feedback_count == 1:
            self._log(op, t, 'Feedback started', out)
        else:
            self._log(op, t, f'Feedback #{op.feedback_count}', out, verbose=True)
        if t - op.last_feedback_packet >= FEEDBACK_PACKET_INTERVAL:
            op.last_feedback_packet = t
            # feedback flows continuously, it does not wait for other packets
            out.append(UiEvent('packet', op, packet='feedback',
                               delay=max(0.0, op.next_packet_time - t), t=t))
        out.append(UiEvent('update', op, t=t))
        return out

    def _new_action_op(self, t, action, goal_id, out) -> Operation:
        server, server_note = self._pick(self.graph.action_servers.get(action), 'server')
        client, client_note = self._pick_client(self.graph.action_clients.get(action), server)
        op = self._new_op(ACTION, action, (ACTION, action, goal_id), t, out,
                          short_id=goal_id[:6], client=client, client_note=client_note,
                          server=server, server_note=server_note,
                          type=self.graph.action_types.get(action, ''))
        self._packet(op, 'goal', t, out)
        self._log(op, t, 'Goal sent', out)
        return op

    # ------------------------------------------------------------- services
    def _on_service_event(self, t, service, code, client_node, client_id, seq,
                          payload, stamp=0.0) -> List[UiEvent]:
        out: List[UiEvent] = []
        key = (SERVICE, service, client_id, seq)
        op = self.by_key.get(key)
        if op is None:
            server, server_note = self._pick(self.graph.service_servers.get(service), 'server')
            if client_node:
                client, client_note = client_node, ''
            else:
                client, client_note = self._pick_client(
                    self.graph.service_clients.get(service), server)
            op = self._new_op(SERVICE, service, key, t, out, short_id=f'seq {seq}',
                              client=client, client_note=client_note,
                              server=server, server_note=server_note,
                              type=self.graph.service_types.get(service, ''))
            if code in (RESPONSE_SENT, RESPONSE_RECEIVED):
                op.partial = True   # the request happened before we could see it
            self._packet(op, 'request', t, out)
            self._log(op, t, 'Request sent', out)
        label = SERVICE_EVENT.get(code, str(code))
        # client and server publish separately, so their events can arrive out
        # of order: keep the earliest time as the start, history sorted
        if t < op.start:
            op.start = t
        if code in (REQUEST_SENT, REQUEST_RECEIVED):
            op.partial = False
        if op.end is not None and not op.partial:
            # a late request event changed the start of a finished call
            self.stats[op.name].record(op.uid, op.end - op.start)
        op.history.append((t, label))
        op.history.sort(key=lambda h: h[0])
        if stamp:
            op.stamps[code] = stamp
        if payload:
            if code in (REQUEST_SENT, REQUEST_RECEIVED):
                op.request_text = payload
            else:
                op.response_text = payload
        if code > op.service_phase:
            op.service_phase = code
            op.status = label
        if code == REQUEST_RECEIVED:
            self._log(op, t, 'Request received by server', out, verbose=True)
        if code in (RESPONSE_SENT, RESPONSE_RECEIVED) and op.end is None:
            self._packet(op, 'response', t, out)
            self._finish(op, t, 'ok')
            self._log(op, t, 'Response ' + ('received' if code == RESPONSE_RECEIVED
                                            else 'sent'), out)
        out.append(UiEvent('update', op, t=t))
        return out

    # -------------------------------------------------------------- helpers
    def _new_op(self, kind, name, key, t, out, **fields) -> Operation:
        st = self.stats.setdefault(name, CallStats(name, kind))
        st.calls += 1
        if fields.get('type'):
            st.type = fields['type']
        op = Operation(uid=next(self._uid), kind=kind, name=name, key=key, start=t,
                       next_packet_time=t, **fields)
        self._infer_parent(op)
        self.by_key[key] = op
        self.by_uid[op.uid] = op
        self.active.append(op)
        if not (op.client and op.server and op.type):
            # discovery may not be finished yet: retry on the next few graph updates
            self._unresolved[op.uid] = 0
        if op.parent is not None:
            op.parent.children.append(op)
        else:
            self.roots.append(op)
        out.append(UiEvent('new', op, t=t))
        for child in self._adopt_orphans(op):
            out.append(UiEvent('update', child, t=t))
        self._prune(out)
        return op

    def _adopt_orphans(self, parent: Operation) -> List[Operation]:
        """A call we learned about late may have children already shown as roots.

        Happens right after startup, when a goal's status arrives after the calls
        its server made.  Adopt top-level calls made by this call's server while
        it was running.
        """
        if not parent.server:
            return []
        adopted = []
        for r in list(self.roots):
            if (r is not parent and r.client == parent.server and r.start >= parent.start
                    and (parent.end is None or r.start <= parent.end)):
                self.roots.remove(r)
                r.parent = parent
                r.parent_note = (f'{r.client} was handling {parent.label} when this call '
                                 f'started (inferred, linked after both were seen)')
                parent.children.append(r)
                adopted.append(r)
        parent.children.sort(key=lambda c: c.start)
        return adopted

    def _attach_late(self, op: Operation):
        """Find a parent for a root whose caller became known only now."""
        if op.parent is not None or not op.client:
            return
        candidates = [o for o in self.by_uid.values()
                      if o is not op and o.server == op.client and o.start <= op.start
                      and (o.end is None or o.end >= op.start)]
        if not candidates:
            return
        parent = max(candidates, key=lambda o: o.start)
        if op in self.roots:
            self.roots.remove(op)
        op.parent = parent
        op.parent_note = (f'{op.client} was handling {parent.label} when this call '
                          f'started (inferred)')
        parent.children.append(op)
        parent.children.sort(key=lambda c: c.start)

    def _infer_parent(self, op: Operation):
        if not op.client:
            return
        candidates = [o for o in self.active
                      if o.server == op.client and o is not op]
        if not candidates:
            return
        parent = max(candidates, key=lambda o: o.start)
        op.parent = parent
        op.parent_note = (f'{op.client} was handling {parent.label} '
                          f'when this call started (inferred)')

    def _pick(self, nodes, what) -> Tuple[Optional[str], str]:
        nodes = sorted(nodes or ())
        if len(nodes) == 1:
            return nodes[0], ''
        if not nodes:
            return None, f'no {what} visible in the ROS graph'
        return None, f'several {what}s: ' + ', '.join(nodes)

    def _pick_client(self, nodes, server) -> Tuple[Optional[str], str]:
        nodes = sorted(n for n in (nodes or ()) if n != server)
        if len(nodes) == 1:
            return nodes[0], ''
        if not nodes:
            return None, 'caller not visible in the ROS graph (e.g. a short-lived CLI node)'
        busy = sorted({o.server for o in self.active if o.server in nodes})
        if len(busy) == 1:
            return busy[0], (f'guessed: {len(nodes)} possible callers, only '
                             f'{busy[0]} was busy at the time')
        return None, 'several possible callers: ' + ', '.join(nodes)

    def _packet(self, op, packet, t, out):
        start = max(t, op.next_packet_time)
        op.next_packet_time = start + PACKET_SPACING
        out.append(UiEvent('packet', op, packet=packet, delay=start - t, t=t))

    def _log(self, op, t, text, out, verbose=False):
        out.append(UiEvent('log', op, text=text, verbose=verbose, t=t))

    def _finish(self, op, t, outcome):
        op.end = t
        op.outcome = outcome
        if op in self.active:
            self.active.remove(op)
        st = self.stats[op.name]
        if outcome == 'ok':
            st.ok += 1
        elif outcome == 'canceled':
            st.canceled += 1
        elif outcome == 'failed':
            st.failed += 1
        if not op.partial:
            st.record(op.uid, op.end - op.start)

    def _subtree_active(self, op) -> bool:
        return op.active or any(self._subtree_active(c) for c in op.children)

    def _drop(self, op):
        for c in op.children:
            self._drop(c)
        self.by_key.pop(op.key, None)
        self.by_uid.pop(op.uid, None)
        if op in self.active:
            self.active.remove(op)

    def _prune(self, out):
        while len(self.roots) > self.max_roots:
            victim = next((r for r in self.roots if not self._subtree_active(r)), None)
            if victim is None:
                return
            self.roots.remove(victim)
            self._drop(victim)
            out.append(UiEvent('removed', victim))
