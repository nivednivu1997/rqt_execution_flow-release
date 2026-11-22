"""ROS side: discover actions and services and listen to their hidden topics.

Read-only by design: this module only creates subscriptions.  It never creates
action clients, service clients or publishers.

What can be seen on Jazzy without touching the application:

* action goal status  -> ``<action>/_action/status``   (accepted, executing,
  canceling, succeeded, canceled, aborted)
* action feedback     -> ``<action>/_action/feedback``

Service calls are only visible when the application enabled service
introspection, which publishes ``<service>/_service_event``.

Callbacks run on the ROS executor thread.  Events are pushed into a
``queue.Queue`` that the Qt thread drains.
"""

import importlib
import threading
import time

from rclpy.action.graph import (get_action_client_names_and_types_by_node,
                                get_action_server_names_and_types_by_node)
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from .introspection import is_internal_service
from .tracker import GraphSnapshot

STATUS_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Best effort matches both reliable and best-effort publishers.
STREAM_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                        reliability=ReliabilityPolicy.BEST_EFFORT)

EVENT_SUFFIX = '/_service_event'
# Status messages that arrive this soon after subscribing describe old goals.
BASELINE_WINDOW = 1.5
TEXT_INTERVAL = 0.2   # seconds between two feedback-to-text conversions per goal
MAX_CLOCK_SKEW = 5.0  # if a sender's clock is further off than this, use arrival time


def _event_time(info):
    """When the message was *sent* (seconds), so late delivery doesn't distort timing."""
    now = time.time()
    try:
        sent = info['source_timestamp'] * 1e-9
    except (KeyError, TypeError):
        return now
    if sent <= 0 or abs(now - sent) > MAX_CLOCK_SKEW:
        return now
    return sent


def _full_name(name, ns):
    return ns.rstrip('/') + '/' + name


def _import_interface(type_str):
    """'pkg/action/Name' -> class, also handles 'pkg/srv/Name_Event'."""
    pkg, kind, name = type_str.split('/')
    module = importlib.import_module(f'{pkg}.{kind}')
    if hasattr(module, name):
        return getattr(module, name)
    if name.endswith('_Event'):
        return getattr(module, name[:-len('_Event')]).Event
    raise ImportError(type_str)


def _to_text(msg):
    try:
        from rosidl_runtime_py import message_to_yaml
        text = message_to_yaml(msg, truncate_length=40)
    except Exception:  # noqa: BLE001 - best effort display only
        text = str(msg)
    return text if len(text) < 4000 else text[:4000] + '\n...'


class RosMonitor:

    def __init__(self, node, out_queue):
        self._node = node
        self._q = out_queue
        self._self_name = node.get_fully_qualified_name()
        self._lock = threading.Lock()
        self._action_subs = {}        # action -> [subscriptions]
        self._service_subs = {}       # service -> subscription
        self._subscribed_at = {}      # action -> time
        self._status_cache = {}       # action -> {goal_id: code}
        self._rebaseline = set()      # actions whose next status skips old results
        self._text_time = {}          # goal_id -> last feedback text time
        self._gid_to_node = {}        # bytes -> node
        self._gid_miss = {}           # service -> last refresh time
        self.problems = {}            # name -> human readable problem
        self.duplicate_nodes = []     # node names used by more than one node
        self.graph = GraphSnapshot()

    # ---------------------------------------------------------- discovery
    def refresh(self) -> GraphSnapshot:
        g = GraphSnapshot()
        node = self._node
        seen, dupes = set(), set()
        for name, ns in node.get_node_names_and_namespaces():
            full = _full_name(name, ns)
            if full in seen:
                # two nodes with one name: ROS mixes up their goals and replies
                dupes.add(full)
                continue
            seen.add(full)
            if full == self._self_name:
                continue
            try:
                a_srv = get_action_server_names_and_types_by_node(node, name, ns)
                a_cli = get_action_client_names_and_types_by_node(node, name, ns)
                s_srv = node.get_service_names_and_types_by_node(name, ns)
                s_cli = node.get_client_names_and_types_by_node(name, ns)
            except Exception:  # noqa: BLE001 - node vanished between calls
                continue
            for aname, types in a_srv:
                g.action_servers.setdefault(aname, set()).add(full)
                g.action_types[aname] = types[0]
            raw_clients = {sname for sname, _ in s_cli}
            for aname, types in a_cli:
                g.action_types.setdefault(aname, types[0])
                # ROS counts any node that listens to an action's feedback as a
                # "client", including watchers like this plugin or
                # `ros2 action echo`. Only nodes that can send goals are callers.
                if aname + '/_action/send_goal' in raw_clients:
                    g.action_clients.setdefault(aname, set()).add(full)
            for sname, types in s_srv:
                if not is_internal_service(sname):
                    g.service_servers.setdefault(sname, set()).add(full)
                    g.service_types[sname] = types[0]
            for sname, types in s_cli:
                if not is_internal_service(sname):
                    g.service_clients.setdefault(sname, set()).add(full)
                    g.service_types.setdefault(sname, types[0])

        event_types = {}
        for topic, types in node.get_topic_names_and_types():
            if topic.endswith(EVENT_SUFFIX) and types:
                service = topic[:-len(EVENT_SUFFIX)]
                if not is_internal_service(service):
                    event_types[service] = types[0]
        g.introspected = set(event_types)
        self.duplicate_nodes = sorted(dupes)

        for action, atype in g.action_types.items():
            if action not in self._action_subs:
                self._subscribe_action(action, atype)
        for service, etype in event_types.items():
            if service not in self._service_subs:
                self._subscribe_service(service, etype)
        self.graph = g
        return g

    # ------------------------------------------------------ subscriptions
    def _subscribe_action(self, action, atype):
        from action_msgs.msg import GoalStatusArray
        subs = []
        self._subscribed_at[action] = time.time()
        subs.append(self._node.create_subscription(
            GoalStatusArray, action + '/_action/status',
            lambda msg, info, a=action: self._on_status(a, msg, info), STATUS_QOS))
        try:
            fb_type = _import_interface(atype).Impl.FeedbackMessage
            subs.append(self._node.create_subscription(
                fb_type, action + '/_action/feedback',
                lambda msg, info, a=action: self._on_feedback(a, msg, info), STREAM_QOS))
        except Exception as e:  # noqa: BLE001
            self.problems[action] = (f'cannot load type {atype} ({e}); '
                                     'feedback is not shown. Source the workspace '
                                     'that defines it before starting rqt.')
        self._action_subs[action] = subs

    def _subscribe_service(self, service, etype):
        try:
            msg_type = _import_interface(etype)
        except Exception as e:  # noqa: BLE001
            self.problems[service] = (f'cannot load type {etype} ({e}); calls are not '
                                      'shown. Source the workspace that defines it.')
            self._service_subs[service] = None
            return
        self._service_subs[service] = self._node.create_subscription(
            msg_type, service + EVENT_SUFFIX,
            lambda msg, info, s=service: self._on_service_event(s, msg, info), STREAM_QOS)

    def rebaseline(self):
        """Re-send the state of running goals on the next status message.

        Used when the display was rebuilt (e.g. back to live after a replay):
        running goals show up again, goals that finished long ago don't.
        """
        with self._lock:
            self._status_cache.clear()
            self._rebaseline = set(self._action_subs)

    def shutdown(self):
        for subs in self._action_subs.values():
            for s in subs:
                self._node.destroy_subscription(s)
        for s in self._service_subs.values():
            if s is not None:
                self._node.destroy_subscription(s)
        self._action_subs.clear()
        self._service_subs.clear()

    # ----------------------------------------------------------- callbacks
    def _on_status(self, action, msg, info=None):
        t = _event_time(info)
        with self._lock:
            baseline = (time.time() - self._subscribed_at.get(action, 0) < BASELINE_WINDOW
                        or action in self._rebaseline)
            self._rebaseline.discard(action)
            cache = self._status_cache.setdefault(action, {})
            present = set()
            for st in msg.status_list:
                gid = bytes(st.goal_info.goal_id.uuid).hex()
                present.add(gid)
                code = st.status
                if cache.get(gid) == code:
                    continue
                cache[gid] = code
                # 'baseline' lets the tracker drop goals that finished before we
                # started watching, while still finishing goals it already shows
                self._q.put(('goal_status', t, action, gid, code, baseline))
            for gid in list(cache):
                if gid not in present:
                    del cache[gid]

    def _on_feedback(self, action, msg, info=None):
        t = _event_time(info)
        gid = bytes(msg.goal_id.uuid).hex()
        text = None
        if t - self._text_time.get(gid, 0.0) > TEXT_INTERVAL:
            if len(self._text_time) > 5000:
                self._text_time.clear()
            self._text_time[gid] = t
            text = _to_text(msg.feedback)
        self._q.put(('feedback', t, action, gid, text))

    def _on_service_event(self, service, msg, msg_info=None):
        t = _event_time(msg_info)
        info = msg.info
        gid = bytes(bytearray(int(x) for x in info.client_gid))
        node = self._resolve_client(service, gid)
        client_id = node or gid.hex()
        payload = None
        if len(msg.request):
            payload = _to_text(msg.request[0])
        elif len(msg.response):
            payload = _to_text(msg.response[0])
        stamp = info.stamp.sec + info.stamp.nanosec * 1e-9
        self._q.put(('service_event', t, service, int(info.event_type), node, client_id,
                     int(info.sequence_number), payload, stamp))

    def _resolve_client(self, service, gid):
        """Map the client gid in a service event to the node that owns the client.

        Client-side events carry the gid of the client's request writer, server-side
        events the gid of the client's response reader.  Both are in the graph.
        """
        node = self._gid_to_node.get(gid)
        if node is not None:
            return node
        now = time.time()
        if now - self._gid_miss.get(service, 0.0) < 1.0:
            return None
        self._gid_miss[service] = now
        try:
            endpoints = list(self._node.get_publishers_info_by_topic(
                'rq' + service + 'Request', no_mangle=True))
            endpoints += list(self._node.get_subscriptions_info_by_topic(
                'rr' + service + 'Reply', no_mangle=True))
        except Exception:  # noqa: BLE001
            return None
        for ep in endpoints:
            if not ep.node_name or 'UNKNOWN' in ep.node_name:
                continue   # discovery not finished: try again on a later event
            key = bytes(bytearray(int(x) for x in ep.endpoint_gid))[:16]
            self._gid_to_node[key] = _full_name(ep.node_name, ep.node_namespace)
        return self._gid_to_node.get(gid)
