"""Main widget: call tree, animated flow view, event timeline and details."""

import html
import os
import queue
import threading
import time
from collections import Counter

import rclpy
from python_qt_binding.QtCore import Qt, QTimer
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from python_qt_binding.QtGui import QBrush, QColor, QFont
from python_qt_binding.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox,
                                         QDoubleSpinBox, QFileDialog, QFrame, QHBoxLayout,
                                         QHeaderView, QLabel, QLineEdit, QMessageBox,
                                         QPushButton, QSlider, QSplitter, QTabWidget,
                                         QTextBrowser, QTreeWidget, QTreeWidgetItem,
                                         QVBoxLayout, QWidget)

from .flow_scene import PACKETS, ROLE_COLORS, ROLE_LABELS, ROLE_ORDER, FlowScene, FlowView
from .monitor import RosMonitor
from .recorder import Player, Recording
from .tracker import ACTION, GOAL_STATUS, UNKNOWN_NODE, Tracker, UiEvent

STATUS_COLORS = {
    'ACCEPTED': '#2563eb', 'EXECUTING': '#2563eb', 'CANCELING': '#ea580c',
    'SUCCEEDED': '#15803d', 'CANCELED': '#d97706', 'ABORTED': '#dc2626',
    'REQUEST SENT': '#9333ea', 'REQUEST RECEIVED': '#9333ea',
    'RESPONSE SENT': '#15803d', 'RESPONSE RECEIVED': '#15803d',
}
LOG_COLORS = {
    'Goal sent': '#2563eb', 'Goal accepted': '#2563eb', 'Executing': '#2563eb',
    'Cancel requested': '#ea580c', 'Result: SUCCEEDED': '#15803d',
    'Result: CANCELED': '#d97706', 'Result: ABORTED': '#dc2626',
    'Request sent': '#9333ea', 'Response received': '#15803d', 'Response sent': '#15803d',
    'Feedback started': '#16a34a',
}
# shorter service states for the tree column; full names stay in the details
SHORT_STATUS = {'REQUEST SENT': 'REQUESTED', 'REQUEST RECEIVED': 'IN SERVER',
                'RESPONSE SENT': 'RESPONDED', 'RESPONSE RECEIVED': 'DONE'}
COL_CALL, COL_STATUS, COL_PATH, COL_DUR, COL_FB = range(5)
MAX_LOG_ROWS = 3000
GRAPH_PERIOD_MS = 1500
FRAME_MS = 33
SPEEDS = [(0.1, '0.1\u00D7'), (0.25, '0.25\u00D7'), (0.5, '0.5\u00D7'), (1.0, '1\u00D7'),
          (2.0, '2\u00D7')]
DEFAULT_SPEED = 0.25
STEP_ANIMATION = 1.6        # seconds of animation played after "Step"
GOAL_CODE = {name: code for code, name in GOAL_STATUS.items()}
FILE_FILTER = 'Execution flow recordings (*.flowrec.json);;All files (*)'
RECORDINGS_DIR = os.path.join(
    os.environ.get('ROS_HOME') or os.path.join(os.path.expanduser('~'), '.ros'),
    'rqt_execution_flow', 'recordings')
QUIET_GAP = 0.4     # recorded seconds of nothing happening that replay skips over
DEFAULT_SLOW = 5.0  # seconds after which a call counts as slow
SLOW_RED = '#dc2626'
SHOW_MODES = [('all', 'All calls'), ('running', 'Running'), ('ok', 'Succeeded'),
              ('canceled', 'Canceled'), ('failed', 'Failed'), ('slow', 'Slow'),
              ('problems', 'Problems')]
STATS_COLS = [
    ('Action / service', ''), ('Kind', ''), ('Calls', 'Calls since start or Clear'),
    ('Running', 'Calls running right now'), ('OK', 'Succeeded or answered'),
    ('Canceled', ''), ('Failed', 'Aborted goals. Services have no failure state.'),
    ('Slow', 'Finished calls that took longer than the slow limit'),
    ('Avg', 'Average duration'), ('p95', '95% of calls were faster than this'),
    ('Max', 'Slowest call')]


class _SortItem(QTreeWidgetItem):
    """Sorts by the value stored in UserRole, so numbers sort as numbers."""

    def __lt__(self, other):
        tree = self.treeWidget()
        col = tree.sortColumn() if tree is not None else 0
        a, b = self.data(col, Qt.UserRole), other.data(col, Qt.UserRole)
        if a is not None and b is not None:
            try:
                return a < b
            except TypeError:
                pass
        return self.text(col) < other.text(col)


def _seconds(v):
    if v is None:
        return '\u2013'
    return f'{v * 1000:.0f} ms' if v < 1.0 else f'{v:.2f} s'


def _clock(t):
    return time.strftime('%H:%M:%S', time.localtime(t)) + f'.{int((t % 1) * 1000):03d}'


def _short(node):
    return node.rsplit('/', 1)[-1] if node else '?'


class ExecutionFlowWidget(QWidget):

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.setObjectName('ExecutionFlowWidget')
        self.setWindowTitle('Execution Flow')
        self._events = queue.Queue()
        self._start_listener(node)
        self._monitor = RosMonitor(self._node, self._events)
        self._tracker = Tracker()
        self._items = {}            # op uid -> QTreeWidgetItem
        self._seen_edges = set()    # edges that had traffic
        self._paused = False
        self._selected_uid = None
        self._details_dirty = False
        self._node_filter = None     # node clicked in the flow view
        self._slow_logged = set()    # uids already reported as slow
        self._stats_items = {}       # name -> statistics row
        self._last_stats = 0.0
        self._mode = 'live'          # 'live' or 'replay'
        self._recording = None       # Recording while the record button is on
        self._last_recording = None
        self._player = None
        self._replay_playing = False
        self._speed = DEFAULT_SPEED
        self._anim_budget = 0.0
        self._last_frame = time.monotonic()
        self._last_slow = 0.0
        self._build_ui()
        self._scene.on_node_clicked = self._on_node_clicked

        self._graph_timer = QTimer(self)
        self._graph_timer.timeout.connect(self._refresh_graph)
        self._graph_timer.start(GRAPH_PERIOD_MS)
        self._frame_timer = QTimer(self)
        self._frame_timer.timeout.connect(self._on_frame)
        self._frame_timer.start(FRAME_MS)
        QTimer.singleShot(100, self._refresh_graph)

    # ============================================================ listener
    def _start_listener(self, host_node):
        """Listen with our own hidden node and a simple executor in our own thread.

        rqt spins its node with a multi-threaded executor one step at a time, which
        can deliver messages late and out of order.  Our own loop avoids that.
        """
        context = host_node.context
        name = f'_execution_flow_{os.getpid()}_{id(self) % 100000}'
        self._node = rclpy.create_node(name, context=context)
        self._executor = SingleThreadedExecutor(context=context)
        self._executor.add_node(self._node)
        self._spinning = True
        self._spin_thread = threading.Thread(target=self._spin, name='execution_flow_spin',
                                             daemon=True)
        self._spin_thread.start()

    def _spin(self):
        while self._spinning and self._node.context.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except (ExternalShutdownException, rclpy.executors.ShutdownException):
                break
            except Exception:  # noqa: BLE001 - keep listening after a bad message
                time.sleep(0.01)

    # ================================================================== UI
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)

        bar = QHBoxLayout()
        self._live = QLabel()
        self._live.setMinimumWidth(80)
        bar.addWidget(self._live)
        self._pause_btn = QPushButton('Pause')
        self._pause_btn.setCheckable(True)
        self._pause_btn.setToolTip('Freeze the display. Events keep being collected.')
        self._pause_btn.toggled.connect(self._set_paused)
        bar.addWidget(self._pause_btn)
        clear_btn = QPushButton('Clear')
        clear_btn.setToolTip('Forget all recorded calls')
        clear_btn.clicked.connect(self.clear)
        bar.addWidget(clear_btn)
        fit_btn = QPushButton('Fit view')
        fit_btn.setToolTip('Zoom to show the whole graph and keep it fitted')
        fit_btn.clicked.connect(lambda: self._view.fit(keep=True))
        bar.addWidget(fit_btn)
        relayout_btn = QPushButton('Re-layout')
        relayout_btn.setToolTip('Undo manual node moves and arrange the graph again')
        relayout_btn.clicked.connect(self._relayout)
        bar.addWidget(relayout_btn)
        self._filter = QLineEdit()
        self._filter.setPlaceholderText('Filter by action, service or node name')
        self._filter.setClearButtonEnabled(True)
        self._filter.textChanged.connect(self._on_filter_changed)
        bar.addWidget(self._filter, 1)
        self._hide_idle = QCheckBox('Only links with traffic')
        self._hide_idle.setToolTip('Hide actions and services that have not been called yet')
        self._hide_idle.toggled.connect(lambda _: self._refresh_topology())
        bar.addWidget(self._hide_idle)
        self._log_fb = QCheckBox('Log every feedback')
        bar.addWidget(self._log_fb)
        root.addLayout(bar)
        rec_bar = self._build_record_bar()
        self._node_chip = QPushButton()
        self._node_chip.setVisible(False)
        self._node_chip.setStyleSheet(
            'QPushButton { background: #fef3c7; border: 1px solid #f59e0b; '
            'border-radius: 4px; padding: 2px 8px; }')
        self._node_chip.clicked.connect(lambda: self._set_node_filter(None))
        rec_bar.addWidget(self._node_chip)
        rec_bar.addWidget(QLabel('Show:'))
        self._show_box = QComboBox()
        for key, label in SHOW_MODES:
            self._show_box.addItem(label, key)
        self._show_box.setToolTip('Show only some calls in the tree and timeline.\n'
                                  'Problems = canceled, failed or slow.')
        self._show_box.currentIndexChanged.connect(lambda _: self._apply_filters())
        rec_bar.addWidget(self._show_box)
        rec_bar.addWidget(QLabel('Slow after'))
        self._slow_limit = QDoubleSpinBox()
        self._slow_limit.setRange(0.1, 3600.0)
        self._slow_limit.setDecimals(1)
        self._slow_limit.setSingleStep(0.5)
        self._slow_limit.setSuffix(' s')
        self._slow_limit.setValue(DEFAULT_SLOW)
        self._slow_limit.setToolTip('Calls running longer than this turn red')
        self._slow_limit.valueChanged.connect(lambda _: self._on_slow_limit_changed())
        rec_bar.addWidget(self._slow_limit)
        root.addLayout(rec_bar)
        # replay controls get their own row so the window never has to grow wider
        root.addWidget(self._replay_box)

        legend = '  '.join(
            f'<span style="color:{c}; font-size:14px;">'
            f'{"&#9632;" if shape == "square" else ("&#9675;" if shape == "ring" else "&#9679;")}'
            f'</span>&nbsp;{label}'
            for c, _, _, shape, label in PACKETS.values())
        info = QHBoxLayout()
        self._legend = QLabel(legend + '&nbsp;&nbsp;&nbsp;<span style="color:#64748b">'
                              'Dashed moving link = call in progress. Drag nodes to move them, '
                              'scroll to zoom.</span>')
        self._legend.setTextFormat(Qt.RichText)
        info.addWidget(self._legend, 1)
        self._warning = QLabel()
        self._warning.setTextFormat(Qt.RichText)
        self._warning.setVisible(False)
        info.addWidget(self._warning)
        root.addLayout(info)

        # call tree
        self._tree = QTreeWidget()
        self._tree.setColumnCount(5)
        self._tree.setHeaderLabels(['Call', 'Status', 'Caller → Server', 'Duration',
                                    'Feedback'])
        self._tree.setUniformRowHeights(True)
        self._tree.setSelectionMode(QAbstractItemView.SingleSelection)
        self._tree.itemSelectionChanged.connect(self._on_tree_selection)
        header = self._tree.header()
        header.setSectionResizeMode(COL_CALL, QHeaderView.Interactive)
        header.resizeSection(COL_CALL, 250)
        header.resizeSection(COL_STATUS, 115)
        header.resizeSection(COL_PATH, 190)
        header.resizeSection(COL_DUR, 60)

        # flow view
        self._scene = FlowScene(self)
        self._view = FlowView(self._scene)

        # timeline
        self._log = QTreeWidget()
        self._log.setRootIsDecorated(False)
        self._log.setUniformRowHeights(True)
        self._log.setColumnCount(5)
        self._log.setHeaderLabels(['Time', 'Event', 'Call', 'Caller → Server', 'Detail'])
        self._log.header().resizeSection(0, 110)
        self._log.header().resizeSection(1, 150)
        self._log.header().resizeSection(2, 250)
        self._log.header().resizeSection(3, 260)
        self._log.itemClicked.connect(self._on_log_clicked)

        self._details = QTextBrowser()
        self._details.setOpenLinks(False)
        self._details.setHtml(self._details_placeholder())

        self._left_split = QSplitter(Qt.Vertical)
        self._tree_box = self._titled('Call tree (live)', self._tree)
        self._left_split.addWidget(self._tree_box)
        self._left_split.addWidget(self._titled('Details', self._details))
        self._left_split.setSizes([600, 340])

        self._right_split = QSplitter(Qt.Vertical)
        flow_box = self._titled('Flow', self._view)
        key = '&nbsp;&nbsp;'.join(
            f'<span style="background:{ROLE_COLORS[r]}; color:white;">&nbsp;'
            f'{ROLE_LABELS[r][0]}&nbsp;</span>' for r in ROLE_ORDER)
        flow_box.title_label.setText(
            f'<b>Flow</b>&nbsp;&nbsp;&nbsp;<span style="color:#64748b">Node roles:</span>'
            f'&nbsp;&nbsp;{key}')
        self._right_split.addWidget(flow_box)
        self._stats = QTreeWidget()
        self._stats.setRootIsDecorated(False)
        self._stats.setUniformRowHeights(True)
        self._stats.setColumnCount(len(STATS_COLS))
        self._stats.setHeaderLabels([c for c, _ in STATS_COLS])
        for col, (_, tip) in enumerate(STATS_COLS):
            if tip:
                self._stats.headerItem().setToolTip(col, tip)
        self._stats.header().resizeSection(0, 230)
        for col in range(1, len(STATS_COLS)):
            self._stats.header().resizeSection(col, 84)
        self._stats.setSortingEnabled(True)
        self._stats.sortByColumn(2, Qt.DescendingOrder)
        self._stats.setToolTip('Double-click a row to filter the call tree by that name')
        self._stats.itemDoubleClicked.connect(
            lambda item, _col: self._filter.setText(item.text(0)))
        self._bottom_tabs = QTabWidget()
        self._bottom_tabs.addTab(self._log, 'Event timeline')
        self._bottom_tabs.addTab(self._stats, 'Statistics')
        self._bottom_tabs.currentChanged.connect(lambda _: self._update_stats())
        self._right_split.addWidget(self._bottom_tabs)
        self._right_split.setSizes([600, 340])

        self._main_split = QSplitter(Qt.Horizontal)
        self._main_split.addWidget(self._left_split)
        self._main_split.addWidget(self._right_split)
        self._main_split.setSizes([620, 980])
        root.addWidget(self._main_split, 1)

        self._status = QLabel()
        self._status.setStyleSheet('color: #64748b;')
        root.addWidget(self._status)
        self._update_live_label()

    def _build_record_bar(self):
        bar = QHBoxLayout()
        self._rec_btn = QPushButton('\u23FA Record')
        self._rec_btn.setCheckable(True)
        self._rec_btn.setToolTip('Record everything that happens, then replay it slowly')
        self._rec_btn.toggled.connect(self._on_record_toggled)
        bar.addWidget(self._rec_btn)
        self._rec_label = QLabel('Not recording')
        self._rec_label.setStyleSheet('color: #64748b;')
        bar.addWidget(self._rec_label)
        self._replay_last_btn = QPushButton('Replay last recording')
        self._replay_last_btn.setEnabled(False)
        self._replay_last_btn.clicked.connect(
            lambda: self._enter_replay(self._last_recording))
        bar.addWidget(self._replay_last_btn)
        self._open_btn = QPushButton('Open\u2026')
        self._open_btn.setToolTip('Open a saved recording')
        self._open_btn.clicked.connect(self._open_recording)
        bar.addWidget(self._open_btn)
        self._save_btn = QPushButton('Save copy\u2026')
        self._save_btn.setToolTip('Recordings are saved automatically in '
                                  f'{RECORDINGS_DIR}.\nUse this to save a copy elsewhere.')
        self._save_btn.setEnabled(False)
        self._save_btn.clicked.connect(self._save_recording)
        bar.addWidget(self._save_btn)

        # replay controls, only visible while replaying
        self._replay_box = QFrame()
        self._replay_box.setFrameShape(QFrame.StyledPanel)
        self._replay_box.setStyleSheet('QFrame { background: #eef2ff; border-radius: 4px; }')
        rb = QHBoxLayout(self._replay_box)
        rb.setContentsMargins(6, 2, 6, 2)
        restart = QPushButton('\u23EE')
        restart.setToolTip('Back to the start of the recording')
        restart.clicked.connect(lambda: self._seek(0.0))
        rb.addWidget(restart)
        self._play_btn = QPushButton('\u25B6 Play')
        self._play_btn.setToolTip('Play / pause the replay')
        self._play_btn.clicked.connect(lambda: self._set_replay_playing(not self._replay_playing))
        rb.addWidget(self._play_btn)
        step = QPushButton('Step \u23ED')
        step.setToolTip('Pause and go to the next event only')
        step.clicked.connect(self._step)
        rb.addWidget(step)
        rb.addWidget(QLabel('Speed'))
        self._speed_box = QComboBox()
        for value, label in SPEEDS:
            self._speed_box.addItem(label, value)
        self._speed_box.setCurrentIndex([v for v, _ in SPEEDS].index(DEFAULT_SPEED))
        self._speed_box.currentIndexChanged.connect(
            lambda i: setattr(self, '_speed', float(self._speed_box.itemData(i))))
        rb.addWidget(self._speed_box)
        self._skip_quiet = QCheckBox('Skip quiet time')
        self._skip_quiet.setChecked(True)
        self._skip_quiet.setToolTip('Jump over moments where nothing is running, '
                                    'so slow replays don\'t look stuck')
        rb.addWidget(self._skip_quiet)
        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, 1000)
        self._slider.setMinimumWidth(200)
        self._slider.setToolTip('Drag or click to jump to a moment in the recording')
        self._slider.valueChanged.connect(self._on_slider_changed)
        self._slider.sliderReleased.connect(
            lambda: self._seek(self._slider_offset(self._slider.value())))
        self._slider.sliderMoved.connect(
            lambda v: self._set_time_label(self._slider_offset(v)))
        rb.addWidget(self._slider, 1)
        self._time_label = QLabel()
        self._time_label.setMinimumWidth(200)
        rb.addWidget(self._time_label)
        back = QPushButton('Back to live')
        back.clicked.connect(self._exit_replay)
        rb.addWidget(back)
        self._replay_box.setVisible(False)
        bar.addStretch(1)
        return bar

    @staticmethod
    def _titled(title, widget):
        box = QWidget()
        lay = QVBoxLayout(box)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        label = QLabel(f'<b>{title}</b>')
        box.title_label = label
        lay.addWidget(label)
        lay.addWidget(widget, 1)
        return box

    @staticmethod
    def _details_placeholder():
        return ('<p style="color:#64748b">Select a call in the tree or the timeline to see '
                'its details.</p><p style="color:#64748b">This tool only watches. It never '
                'sends goals, cancels or service requests.</p>')

    # ============================================================ ROS graph
    def _refresh_graph(self):
        try:
            graph = self._monitor.refresh()
        except Exception as e:  # noqa: BLE001 - keep the GUI alive
            self._status.setText(f'Graph refresh failed: {e}')
            return
        if self._recording is not None:
            self._recording.add_graph(time.time(), graph)
        if self._mode != 'live':
            return   # the replay drives the graph
        self._set_graph(graph)

        n_act = len(graph.action_servers)
        n_srv = len(graph.service_servers)
        hidden = sorted(s for s in graph.service_servers if s not in graph.introspected)
        dupes = self._monitor.duplicate_nodes
        self._status.setText(
            f'Watching {n_act} action(s) and {len(graph.introspected)} of {n_srv} service(s). '
            f'Read-only: this tool never calls, sends goals or cancels.')
        problems = self._monitor.problems
        if dupes:
            self._warning.setText(
                f'<b style="color:#dc2626">&#9888; {len(dupes)} node name(s) are used by more '
                f'than one node: {html.escape(", ".join(dupes[:3]))}</b>')
            self._warning.setToolTip(
                'Two nodes with the same name usually means a program was started twice.\n'
                'Goals and replies then go to the wrong copy, and calls can hang.\n'
                'Stop the extra copy.\n\n' + '\n'.join(dupes))
            self._warning.setVisible(True)
        elif problems:
            self._warning.setText(
                f'<b style="color:#dc2626">&#9888; Can\'t load {len(problems)} message type(s). '
                f'Source the workspace that defines them, then restart.</b>')
            self._warning.setToolTip(
                'The plugin needs your interface packages to show feedback and service calls.\n'
                'Run  source <your_ws>/install/setup.bash  before starting rqt.\n\n'
                + '\n'.join(f'{name}: {text.split(";")[0]}'
                             for name, text in sorted(problems.items())))
            self._warning.setVisible(True)
        elif hidden:
            self._warning.setText(
                f'<span style="color:#b45309">&#9888; {len(hidden)} service(s) have '
                f'introspection off, so their calls are invisible</span>')
            tip = ('Enable service introspection in the nodes that serve or call them.\n'
                   'Python:  from rqt_execution_flow.introspection import '
                   'enable_service_introspection\n'
                   '         enable_service_introspection(node)\n\n' + '\n'.join(hidden[:40]))
            if len(hidden) > 40:
                tip += f'\n... and {len(hidden) - 40} more'
            self._warning.setToolTip(tip)
            self._warning.setVisible(True)
        else:
            self._warning.setVisible(False)

    def _refresh_topology(self):
        g = self._tracker.graph
        specs, isolated, roles = {}, set(), {}

        def add_role(node, role):
            roles.setdefault(node, Counter())[role] += 1

        for kind, servers, clients in ((ACTION, g.action_servers, g.action_clients),
                                       ('service', g.service_servers, g.service_clients)):
            for name, srv_nodes in servers.items():
                introspected = kind == ACTION or name in g.introspected
                for s in srv_nodes:
                    add_role(s, f'{kind}_server')
                    callers = [c for c in clients.get(name, ()) if c != s]
                    if not callers:
                        isolated.add(s)
                    for c in callers:
                        specs[(kind, name, c, s)] = introspected
            for name, cl_nodes in clients.items():
                for c in cl_nodes:
                    add_role(c, f'{kind}_client')
        for key in self._seen_edges:
            specs.setdefault(key, True)

        if self._hide_idle.isChecked():
            specs = {k: v for k, v in specs.items() if k in self._seen_edges}
            isolated = set()
        text = self._filter.text().strip().lower()
        if text:
            specs = {k: v for k, v in specs.items()
                     if any(text in part.lower() for part in k[1:])}
            isolated = {n for n in isolated if text in n.lower()}
        isolated -= {k[2] for k in specs} | {k[3] for k in specs}

        roles = {node: dict(c) for node, c in roles.items()}
        roles[UNKNOWN_NODE] = 'caller not in graph'
        if self._scene.sync(specs, isolated, roles):
            self._view.auto_fit()

    def _relayout(self):
        for n in self._scene.nodes.values():
            n.pinned = False
        self._scene.layout()
        self._view.fit(keep=True)

    # ============================================================ frames
    def _on_frame(self):
        now = time.monotonic()
        dt = min(0.2, now - self._last_frame)
        self._last_frame = now
        if self._mode == 'live':
            if not self._paused:
                self._drain_live()
                self._scene.tick(dt)
        else:
            self._drain_discard()
            if self._replay_playing:
                self._maybe_skip_quiet()
                target = self._player.pos + dt * self._speed
                self._feed(self._player.advance_to(target), animate=True)
                self._scene.tick(dt * self._speed)
                if self._player.at_end:
                    self._set_replay_playing(False)
                    self._anim_budget = STEP_ANIMATION   # let the last packets land
            elif self._anim_budget > 0:
                step_dt = dt * max(self._speed, 0.5)
                self._anim_budget -= step_dt
                self._scene.tick(step_dt)
            self._update_replay_position()
        if now - self._last_slow > 0.25:
            self._last_slow = now
            self._slow_update()

    def _drain_live(self):
        backlog = self._events.qsize()
        animate = backlog < 300   # after a long pause, skip the animation
        for _ in range(min(backlog, 3000)):
            try:
                ev = self._events.get_nowait()
            except queue.Empty:
                break
            if self._recording is not None:
                self._recording.add_event(ev)
            for ui in self._tracker.handle(ev):
                self._apply(ui, animate)
        if self._recording is not None and self._recording.full:
            self._rec_btn.setChecked(False)
            self._rec_label.setText('Recording stopped: it reached the size limit')

    def _drain_discard(self):
        # live events are not shown while replaying
        for _ in range(self._events.qsize()):
            try:
                self._events.get_nowait()
            except queue.Empty:
                break

    def _feed(self, items, animate):
        for kind, item in items:
            if kind == 'graph':
                self._set_graph(item)
            else:
                for ui in self._tracker.handle(item):
                    self._apply(ui, animate)

    def _set_graph(self, graph):
        for op in self._tracker.set_graph(graph):
            self._seen_edges.add(op.edge_key())
            self._update_item(op)
        # drop "(unknown)" links that no recorded call uses any more
        used = {op.edge_key() for op in self._tracker.by_uid.values()}
        self._seen_edges = {k for k in self._seen_edges if UNKNOWN_NODE not in k or k in used}
        self._refresh_topology()

    def _maybe_skip_quiet(self):
        if not self._skip_quiet.isChecked() or self._tracker.active or self._scene.packets:
            return
        nxt = self._player.next_event_time()
        if nxt is not None and nxt - self._player.pos > QUIET_GAP:
            self._player.skip_to(nxt - 0.1)

    def _now(self):
        """Current time: wall clock when live, recorded time when replaying."""
        if self._mode == 'replay' and self._player is not None:
            return self._player.pos
        return time.time()

    def _apply(self, ui, animate):
        op = ui.op
        if ui.kind == 'new':
            key = op.edge_key()
            self._seen_edges.add(key)
            if self._scene.ensure_edge(key):
                self._refresh_topology()
                self._view.auto_fit()
            self._add_item(op)
        elif ui.kind == 'update':
            self._update_item(op)
            if op.uid == self._selected_uid:
                self._details_dirty = True
        elif ui.kind == 'packet':
            if animate:
                self._scene.emit_packet(op.edge_key(), ui.packet, ui.delay)
        elif ui.kind == 'log':
            if not ui.verbose or self._log_fb.isChecked():
                self._add_log(ui)
        elif ui.kind == 'removed':
            self._remove_item(op)

    def _slow_update(self):
        self._update_live_label()
        active = self._tracker.active
        counts = Counter(op.edge_key() for op in active)
        busy = {op.server for op in active if op.server}
        now = self._now()
        limit = self._slow_limit.value()
        slow_keys = set()
        for op in list(active):
            slow = self._is_slow(op, now)
            if slow:
                slow_keys.add(op.edge_key())
            item = self._items.get(op.uid)
            if item is not None:
                item.setText(COL_DUR, self._duration_text(op, now))
                self._style_slow(item, op, now)
            if slow and op.uid not in self._slow_logged:
                self._slow_logged.add(op.uid)
                self._add_log(UiEvent('log', op, text=f'Slow: running over {limit:g} s', t=now))
                if item is not None:
                    self._apply_tree_filter_to(item)
        self._scene.set_activity(counts, busy, slow_keys)
        if time.monotonic() - self._last_stats > 1.0:
            self._update_stats()
        if self._details_dirty or (
                self._selected_uid is not None and
                self._tracker.by_uid.get(self._selected_uid) in active):
            self._details_dirty = False
            self._show_details(self._selected_uid)

    def _update_live_label(self):
        blink = int(time.monotonic() * 2) % 2 == 0
        if self._mode == 'replay':
            state = '\u25B6' if self._replay_playing else '&#10073;&#10073;'
            self._live.setText(f'<b style="color:#4338ca">{state} REPLAY</b>')
            return
        if self._recording is not None:
            dot = '#dc2626' if blink else '#fca5a5'
            self._live.setText(f'<b style="color:#dc2626"><span style="color:{dot}">'
                               f'&#9210;</span> REC</b>')
            rec = self._recording
            self._rec_label.setText(f'Recording: {time.time() - rec.start:.1f} s, '
                                    f'{len(rec.events)} events')
            return
        if self._paused:
            self._live.setText('<b style="color:#64748b">&#10073;&#10073; PAUSED</b>')
        else:
            dot = '#dc2626' if blink else '#fca5a5'
            self._live.setText(f'<b><span style="color:{dot}">&#9679;</span> LIVE</b>')

    def _set_paused(self, paused):
        self._paused = paused
        self._pause_btn.setText('Resume' if paused else 'Pause')
        self._update_live_label()

    # ============================================================ call tree
    def _add_item(self, op):
        parent_item = self._items.get(op.parent.uid) if op.parent is not None else None
        item = QTreeWidgetItem()
        icon = '▶' if op.kind == ACTION else '⇄'
        item.setText(COL_CALL, f'{icon} {op.name}  #{op.short_id}')
        item.setData(COL_CALL, Qt.UserRole, op.uid)
        item.setToolTip(COL_CALL, op.type or op.name)
        item.setForeground(COL_CALL, QBrush(QColor('#2563eb' if op.kind == ACTION
                                                   else '#9333ea')))
        if parent_item is not None:
            parent_item.addChild(item)
            parent_item.setExpanded(True)
        else:
            self._tree.addTopLevelItem(item)
        self._items[op.uid] = item
        self._update_item(op)
        self._apply_tree_filter_to(item)

    def _update_item(self, op):
        item = self._items.get(op.uid)
        if item is None:
            return
        self._place_item(op, item)
        status = op.status
        if op.kind == ACTION and op.cancel_requested and op.active:
            status = 'CANCELING'
        item.setText(COL_STATUS, SHORT_STATUS.get(status, status))
        item.setToolTip(COL_STATUS, status)
        item.setForeground(COL_STATUS, QBrush(QColor(STATUS_COLORS.get(status, '#334155'))))
        caller = _short(op.client) if op.client else '?'
        item.setText(COL_PATH, f'{caller} → {_short(op.server) if op.server else "?"}')
        item.setToolTip(COL_PATH, f'{op.client or op.client_note} → '
                                  f'{op.server or op.server_note}')
        item.setText(COL_DUR, self._duration_text(op, self._now()))
        item.setText(COL_FB, str(op.feedback_count) if op.kind == ACTION else '')
        font = QFont(item.font(COL_CALL))
        font.setBold(op.active)
        for col in range(5):
            item.setFont(col, font)
        self._style_slow(item, op, self._now())
        if self._filters_active(self._filters()):
            self._apply_tree_filter_to(item)

    @staticmethod
    def _duration_text(op, now):
        if op.partial:
            return '?'
        return f'{op.duration(now):.1f} s'

    def _place_item(self, op, item):
        """Move a row under its parent if the parent was found after the row was added."""
        want = self._items.get(op.parent.uid) if op.parent is not None else None
        if want is None or item.parent() is want:
            return
        current = item.parent()
        if current is not None:
            current.removeChild(item)
        else:
            idx = self._tree.indexOfTopLevelItem(item)
            if idx < 0:
                return
            self._tree.takeTopLevelItem(idx)
        siblings = op.parent.children
        pos = siblings.index(op) if op in siblings else want.childCount()
        want.insertChild(min(pos, want.childCount()), item)
        want.setExpanded(True)
        item.setExpanded(True)

    def _remove_item(self, op):
        item = self._items.pop(op.uid, None)
        for child in op.children:
            self._remove_item(child)
        if item is None:
            return
        parent = item.parent()
        if parent is not None:
            parent.removeChild(item)
        else:
            idx = self._tree.indexOfTopLevelItem(item)
            if idx >= 0:
                self._tree.takeTopLevelItem(idx)

    def _on_filter_changed(self, _):
        self._refresh_topology()
        self._apply_filters()

    # ---- slow calls
    def _is_slow(self, op, now):
        return not op.partial and op.duration(now) > self._slow_limit.value()

    def _style_slow(self, item, op, now):
        slow = self._is_slow(op, now)
        background = QBrush(QColor('#fee2e2')) if (slow and op.active) else None
        for col in range(5):
            item.setData(col, Qt.BackgroundRole, background)
        item.setData(COL_DUR, Qt.ForegroundRole, QBrush(QColor(SLOW_RED)) if slow else None)
        item.setToolTip(COL_DUR, f'Longer than the slow limit of '
                                 f'{self._slow_limit.value():g} s' if slow else '')

    def _on_slow_limit_changed(self):
        now = self._now()
        for uid, item in list(self._items.items()):
            op = self._tracker.by_uid.get(uid)
            if op is not None:
                self._style_slow(item, op, now)
        self._apply_filters()
        self._update_stats()

    # ---- node focus
    def _on_node_clicked(self, name):
        self._set_node_filter(None if name == self._node_filter else name)

    def _set_node_filter(self, name):
        self._node_filter = name
        self._scene.set_focus(name)
        if name:
            self._node_chip.setText(f'Node: {_short(name)}  \u2715')
            self._node_chip.setToolTip(f'Showing only calls made or served by {name}.\n'
                                       'Click here, or the node again, to show all.')
        self._node_chip.setVisible(bool(name))
        self._apply_filters()

    # ---- filtering the tree and the timeline
    def _filters(self):
        return (self._filter.text().strip().lower(), self._node_filter,
                self._show_box.currentData() or 'all', self._now())

    @staticmethod
    def _filters_active(f):
        return bool(f[0] or f[1] or f[2] != 'all')

    def _matches(self, op, f):
        text, node, mode, now = f
        if text and not any(text in (s or '').lower() for s in (op.name, op.client, op.server)):
            return False
        if node and node not in (op.client, op.server):
            return False
        if mode == 'all':
            return True
        slow = self._is_slow(op, now)
        return {'running': op.active, 'ok': op.outcome == 'ok',
                'canceled': op.outcome == 'canceled', 'failed': op.outcome == 'failed',
                'slow': slow,
                'problems': slow or op.outcome in ('canceled', 'failed')}.get(mode, True)

    def _visible(self, op, f):
        """A row stays visible if it, or one of its children, matches."""
        return self._matches(op, f) or any(self._visible(c, f) for c in op.children)

    def _apply_filters(self):
        self._apply_tree_filter()
        self._apply_log_filter()

    def _apply_tree_filter(self):
        f = self._filters()
        for uid in list(self._items):
            self._apply_tree_filter_to(self._items[uid], f)

    def _apply_tree_filter_to(self, item, f=None):
        uid = item.data(COL_CALL, Qt.UserRole)
        op = self._tracker.by_uid.get(uid)
        if op is None:
            return
        f = f or self._filters()
        if not self._filters_active(f):
            item.setHidden(False)
            return
        item.setHidden(not self._visible(op, f))
        # parents must stay visible if a child became visible
        p = item.parent()
        while p is not None:
            pop = self._tracker.by_uid.get(p.data(COL_CALL, Qt.UserRole))
            if pop is not None:
                p.setHidden(not self._visible(pop, f))
            p = p.parent()

    def _apply_log_filter(self):
        f = self._filters()
        active = self._filters_active(f)
        for i in range(self._log.topLevelItemCount()):
            row = self._log.topLevelItem(i)
            row.setHidden(self._log_hidden(row, f, active))

    def _log_hidden(self, row, f, active):
        if not active:
            return False
        op = self._tracker.by_uid.get(row.data(0, Qt.UserRole))
        return op is None or not self._matches(op, f)

    # ---- statistics
    def _update_stats(self):
        self._last_stats = time.monotonic()
        if self._bottom_tabs.currentWidget() is not self._stats:
            return
        limit = self._slow_limit.value()
        running = Counter(op.name for op in self._tracker.active)
        self._stats.setSortingEnabled(False)
        for name, st in self._tracker.stats.items():
            item = self._stats_items.get(name)
            if item is None:
                item = _SortItem()
                self._stats.addTopLevelItem(item)
                self._stats_items[name] = item
            tm = st.timing(limit)
            is_action = st.kind == ACTION
            values = [
                (name, name), ('Action' if is_action else 'Service', st.kind),
                (str(st.calls), st.calls), (str(running[name]), running[name]),
                (str(st.ok), st.ok), (str(st.canceled), st.canceled),
                (str(st.failed) if is_action else '\u2013', st.failed if is_action else -1),
                (str(tm['slow']), tm['slow']),
                (_seconds(tm['avg']), tm['avg'] if tm['avg'] is not None else -1.0),
                (_seconds(tm['p95']), tm['p95'] if tm['p95'] is not None else -1.0),
                (_seconds(tm['max']), tm['max'] if tm['max'] is not None else -1.0)]
            for col, (text, key) in enumerate(values):
                item.setText(col, text)
                item.setData(col, Qt.UserRole, key)
            item.setToolTip(0, st.type or name)
            item.setForeground(0, QBrush(QColor('#2563eb' if is_action else '#9333ea')))
            for col, bad, color in ((5, st.canceled, '#d97706'), (6, st.failed, SLOW_RED),
                                    (7, tm['slow'], SLOW_RED)):
                item.setData(col, Qt.ForegroundRole, QBrush(QColor(color)) if bad else None)
        self._stats.setSortingEnabled(True)

    def _on_tree_selection(self):
        items = self._tree.selectedItems()
        if not items:
            return
        uid = items[0].data(COL_CALL, Qt.UserRole)
        self._select(uid, from_tree=True)

    def _select(self, uid, from_tree=False):
        self._selected_uid = uid
        op = self._tracker.by_uid.get(uid)
        self._scene.set_highlight(op.edge_key() if op else None)
        if not from_tree and uid in self._items:
            self._tree.setCurrentItem(self._items[uid])
            self._tree.scrollToItem(self._items[uid])
        self._show_details(uid)

    # ============================================================ timeline
    def _add_log(self, ui):
        op = ui.op
        item = QTreeWidgetItem([
            _clock(ui.t), ui.text, op.label,
            f'{_short(op.client) if op.client else "?"} → '
            f'{_short(op.server) if op.server else "?"}',
            self._log_detail(op, ui.text, ui.t)])
        item.setData(0, Qt.UserRole, op.uid)
        color = LOG_COLORS.get(ui.text)
        if color is None and ui.text.startswith('Feedback'):
            color = '#16a34a'
        if color:
            item.setForeground(1, QBrush(QColor(color)))
        if color is None and ui.text.startswith('Slow'):
            item.setForeground(1, QBrush(QColor(SLOW_RED)))
        scrollbar = self._log.verticalScrollBar()
        at_bottom = scrollbar.value() >= scrollbar.maximum() - 4
        self._log.addTopLevelItem(item)
        f = self._filters()
        if self._filters_active(f):
            item.setHidden(self._log_hidden(item, f, True))
        while self._log.topLevelItemCount() > MAX_LOG_ROWS:
            self._log.takeTopLevelItem(0)
        if at_bottom:
            self._log.scrollToBottom()

    @staticmethod
    def _log_detail(op, text, t):
        if text.startswith('Result') or text.startswith('Response'):
            if op.partial:
                return 'start not seen'
            return f'after {op.duration(t):.2f} s'
        if text == 'Goal sent' and op.parent is not None:
            return f'inside {op.parent.label}'
        if text == 'Request sent' and op.parent is not None:
            return f'inside {op.parent.label}'
        return ''

    def _on_log_clicked(self, item, _col):
        self._select(item.data(0, Qt.UserRole))

    # ============================================================ details
    def _show_details(self, uid):
        op = self._tracker.by_uid.get(uid)
        if op is None:
            self._details.setHtml(self._details_placeholder())
            return
        e = html.escape
        now = self._now()
        rows = [
            ('Kind', 'Action goal' if op.kind == ACTION else 'Service call'),
            ('Type', op.type or '?'),
            ('Status', f'<b style="color:{STATUS_COLORS.get(op.status, "#334155")}">'
                       f'{e(op.status)}</b>' + (' (cancel requested)'
                                                 if op.cancel_requested else '')),
            ('Caller', e(op.client or '?') + (f' <i style="color:#64748b">({e(op.client_note)})'
                                              f'</i>' if op.client_note else '')),
            ('Server', e(op.server or '?') + (f' <i style="color:#64748b">({e(op.server_note)})'
                                              f'</i>' if op.server_note else '')),
            ('Goal id' if op.kind == ACTION else 'Call', e(op.key[-1] if op.kind == ACTION
                                                            else op.short_id)),
            ('Started', _clock(op.start)),
            ('Duration', 'unknown: it started before the plugin could see it'
             if op.partial else
             f'{op.duration(now):.3f} s' + ('' if op.end else ' (running)')),
        ]
        if op.kind == ACTION:
            rows.append(('Feedback messages', str(op.feedback_count)))
        else:
            st = op.stamps
            if 0 in st and 3 in st:
                rows.append(('Round trip (client clock)', f'{(st[3] - st[0]) * 1000:.1f} ms'))
            if 1 in st and 2 in st:
                rows.append(('Server work (server clock)', f'{(st[2] - st[1]) * 1000:.1f} ms'))
        if op.parent is not None:
            rows.append(('Called from', f'{e(op.parent.label)}<br><i style="color:#64748b">'
                                        f'{e(op.parent_note)}</i>'))
        if op.children:
            rows.append(('Calls made', ', '.join(e(c.label) for c in op.children[:12])))
        body = ''.join(f'<tr><td style="color:#64748b; padding-right:10px">{k}</td>'
                       f'<td>{v}</td></tr>' for k, v in rows)
        hist = ''.join(f'<tr><td style="color:#64748b; padding-right:10px">'
                       f'+{t - op.start:.3f} s</td><td>{e(s)}</td></tr>'
                       for t, s in op.history[-30:])
        icon = '&#9654;' if op.kind == ACTION else '&#8644;'
        out = (f'<h3 style="margin:2px 0">{icon} {e(op.name)}</h3>'
               f'<table cellspacing="0">{body}</table>'
               f'<p style="margin:8px 0 2px 0"><b>History</b></p>'
               f'<table cellspacing="0">{hist}</table>')
        for title, text in (('Last feedback', op.last_feedback),
                            ('Request', op.request_text), ('Response', op.response_text)):
            if text:
                out += (f'<p style="margin:8px 0 2px 0"><b>{title}</b></p>'
                        f'<pre style="margin:0">{e(text)}</pre>')
        if op.kind != ACTION and not op.request_text and not op.response_text:
            out += ('<p style="color:#64748b">No request/response data. The node enabled '
                    'introspection with timing only (METADATA).</p>')
        scroll = self._details.verticalScrollBar().value()
        self._details.setHtml(out)
        self._details.verticalScrollBar().setValue(scroll)

    # ============================================================ misc
    def clear(self):
        self._reset_display()
        self._refresh_topology()

    def _reset_display(self):
        self._tracker.clear()
        self._slow_logged.clear()
        self._stats.clear()
        self._stats_items.clear()
        self._tree.clear()
        self._log.clear()
        self._items.clear()
        self._seen_edges.clear()
        self._scene.clear_packets()
        self._selected_uid = None
        self._scene.set_highlight(None)
        self._details.setHtml(self._details_placeholder())

    def shutdown(self):
        self._recording = None
        self._graph_timer.stop()
        self._frame_timer.stop()
        self._spinning = False
        self._spin_thread.join(timeout=2.0)
        self._monitor.shutdown()
        self._executor.shutdown()
        self._node.destroy_node()

    def save_settings(self, settings):
        settings.set_value('hide_idle', self._hide_idle.isChecked())
        settings.set_value('show', self._show_box.currentData())
        settings.set_value('slow_limit', self._slow_limit.value())
        settings.set_value('log_feedback', self._log_fb.isChecked())
        settings.set_value('replay_speed', self._speed)
        settings.set_value('filter', self._filter.text())
        settings.set_value('main_split', self._main_split.saveState())
        settings.set_value('left_split', self._left_split.saveState())
        settings.set_value('right_split', self._right_split.saveState())

    def restore_settings(self, settings):
        def flag(name):
            v = settings.value(name, False)
            return v in (True, 'true', 'True', 1, '1')
        self._hide_idle.setChecked(flag('hide_idle'))
        mode = settings.value('show', None) or ('running' if flag('hide_done') else 'all')
        idx = self._show_box.findData(mode)
        if idx >= 0:
            self._show_box.setCurrentIndex(idx)
        try:
            self._slow_limit.setValue(float(settings.value('slow_limit', DEFAULT_SLOW)))
        except (TypeError, ValueError):
            pass
        self._log_fb.setChecked(flag('log_feedback'))
        self._filter.setText(settings.value('filter', '') or '')
        try:
            speed = float(settings.value('replay_speed', DEFAULT_SPEED))
            self._speed_box.setCurrentIndex([v for v, _ in SPEEDS].index(speed))
        except (TypeError, ValueError):
            pass
        for name, split in (('main_split', self._main_split), ('left_split', self._left_split),
                            ('right_split', self._right_split)):
            state = settings.value(name)
            if state is not None:
                try:
                    split.restoreState(state)
                except TypeError:
                    pass

    # ============================================================ recording
    def _on_record_toggled(self, on):
        if on:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        self._drain_live()   # events from before the button press belong to "before"
        t0 = time.time()
        rec = Recording(t0)
        rec.add_graph(t0, self._monitor.graph)
        # Calls already running: store their current state so the replay
        # starts with them (and their children get the right parent).
        for op in sorted(self._tracker.active, key=lambda o: o.start):
            rec.add_event(self._snapshot_event(op, t0))
        self._recording = rec
        self._rec_btn.setText('\u23F9 Stop recording')
        self._rec_btn.setStyleSheet('color: #dc2626; font-weight: bold;')
        self._replay_last_btn.setEnabled(False)
        self._open_btn.setEnabled(False)
        self._save_btn.setEnabled(False)

    @staticmethod
    def _snapshot_event(op, t):
        if op.kind == ACTION:
            code = GOAL_CODE.get('CANCELING' if op.cancel_requested else op.status, 2)
            return ('goal_status', t, op.name, op.key[2], code)
        client_node = op.client if not op.client_note else None
        return ('service_event', t, op.name, max(op.service_phase, 0), client_node,
                op.key[2], op.key[3], op.request_text or None, 0.0)

    def _stop_recording(self):
        rec, self._recording = self._recording, None
        self._rec_btn.setText('\u23FA Record')
        self._rec_btn.setStyleSheet('')
        self._open_btn.setEnabled(True)
        if rec is None:
            return
        self._drain_live()
        rec.finish(time.time())
        if not rec.events:
            self._rec_label.setText('Nothing happened while recording')
            self._update_record_buttons()
            return
        self._last_recording = rec
        self._autosave(rec)
        self._enter_replay(rec)

    def _autosave(self, rec):
        name = time.strftime('flow_%Y%m%d_%H%M%S.flowrec.json', time.localtime(rec.start))
        path = os.path.join(RECORDINGS_DIR, name)
        try:
            os.makedirs(RECORDINGS_DIR, exist_ok=True)
            rec.save(path)
            rec.path = path
        except OSError as e:
            rec.path = None
            QMessageBox.warning(self, 'Recording', f'Could not save the recording to\n'
                                                   f'{path}\n\n{e}\n\nUse Save\u2026 to pick '
                                                   f'another place.')

    def _update_record_buttons(self):
        has = self._last_recording is not None
        replaying = self._mode == 'replay'
        self._replay_last_btn.setEnabled(has and not replaying)
        self._save_btn.setEnabled(has or replaying)
        self._rec_btn.setEnabled(not replaying)

    def _save_recording(self):
        rec = self._player.rec if self._mode == 'replay' else self._last_recording
        if rec is None:
            return
        name = time.strftime('flow_%Y%m%d_%H%M%S.flowrec.json', time.localtime(rec.start))
        path, _ = QFileDialog.getSaveFileName(self, 'Save a copy of the recording',
                                              os.path.join(os.path.expanduser('~'), name),
                                              FILE_FILTER)
        if not path:
            return
        try:
            rec.save(path)
        except OSError as e:
            QMessageBox.warning(self, 'Save recording', f'Could not save:\n{e}')
            return
        self._rec_label.setText(f'Saved a copy as {os.path.basename(path)}')
        self._rec_label.setToolTip(path)

    def _open_recording(self):
        start_dir = RECORDINGS_DIR if os.path.isdir(RECORDINGS_DIR) else ''
        path, _ = QFileDialog.getOpenFileName(self, 'Open recording', start_dir, FILE_FILTER)
        if not path:
            return
        try:
            rec = Recording.load(path)
        except (OSError, ValueError, KeyError, TypeError) as e:
            QMessageBox.warning(self, 'Open recording', f'Could not open {path}:\n{e}')
            return
        rec.path = path
        self._last_recording = rec
        self._enter_replay(rec)

    # ============================================================ replay
    def _enter_replay(self, rec):
        if rec is None:
            return
        self._mode = 'replay'
        self._player = Player(rec)
        self._anim_budget = 0.0
        self._reset_display()
        self._feed(self._player.advance_to(rec.start), animate=False)
        self._refresh_topology()
        self._replay_box.setVisible(True)
        self._tree_box.title_label.setText('<b style="color:#4338ca">Call tree (replay)</b>')
        self._pause_btn.setEnabled(False)
        path = getattr(rec, 'path', None)
        # only the file name: a long path would force the window wider
        where = f'  \u2022  saved as {os.path.basename(path)}' if path else '  \u2022  not saved'
        self._rec_label.setText(f'Recording: {rec.duration:.1f} s, {len(rec.events)} events'
                                f'{where}')
        self._rec_label.setToolTip(path or 'Use Save\u2026 to keep this recording')
        self._rec_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._update_record_buttons()
        self._set_replay_playing(False)
        self._update_replay_position()
        self._status.setText('Replaying a recording. Live events are not shown until you go '
                             'back to live.')

    def _exit_replay(self):
        self._mode = 'live'
        self._player = None
        self._replay_playing = False
        self._reset_display()
        self._monitor.rebaseline()   # running goals reappear, old results don't
        self._set_graph(self._monitor.graph)
        self._replay_box.setVisible(False)
        self._tree_box.title_label.setText('<b>Call tree (live)</b>')
        self._pause_btn.setEnabled(True)
        self._update_record_buttons()
        self._update_live_label()
        self._refresh_graph()

    def _set_replay_playing(self, playing):
        if self._player is None:
            return
        if playing and self._player.at_end:
            self._seek(0.0)
        self._replay_playing = playing
        self._play_btn.setText('\u275A\u275A Pause' if playing else '\u25B6 Play')
        self._update_live_label()

    def _step(self):
        if self._player is None:
            return
        self._set_replay_playing(False)
        items = self._player.step()
        self._feed(items, animate=True)
        self._anim_budget = STEP_ANIMATION
        self._update_replay_position()

    def _seek(self, offset):
        if self._player is None:
            return
        target = self._player.rec.start + max(0.0, offset)
        if target < self._player.pos:
            self._reset_display()
            self._player.reset()
        self._feed(self._player.advance_to(target), animate=False)
        self._scene.clear_packets()
        self._anim_budget = 0.0
        self._update_replay_position()
        self._slow_update()

    def _slider_offset(self, value):
        return value / 1000.0 * self._player.rec.duration if self._player else 0.0

    def _on_slider_changed(self, value):
        # clicks on the track and keyboard moves; drags seek on release
        if not self._slider.isSliderDown():
            self._seek(self._slider_offset(value))

    def _update_replay_position(self):
        if self._player is None:
            return
        rec = self._player.rec
        offset = self._player.offset
        if not self._slider.isSliderDown():
            self._slider.blockSignals(True)
            self._slider.setValue(int(1000 * offset / rec.duration) if rec.duration else 0)
            self._slider.blockSignals(False)
            self._set_time_label(offset)

    def _set_time_label(self, offset):
        rec = self._player.rec if self._player else None
        if rec is None:
            return
        self._time_label.setText(f'{offset:6.2f} / {rec.duration:.2f} s   '
                                 f'({_clock(rec.start + offset)})')
