"""Animated flow view: ROS nodes as boxes, actions/services as curved links,
and packets that travel along the links like a network animation.

Packets going from caller to server: goal, cancel, service request.
Packets coming back: accepted, feedback, result, service response.
"""

from collections import defaultdict

from python_qt_binding.QtCore import QPointF, QRectF, Qt
from python_qt_binding.QtGui import (QBrush, QColor, QFont, QFontMetrics,
                                     QPainter, QPainterPath, QPen, QPixmap, QPolygonF)
from python_qt_binding.QtWidgets import (QGraphicsItem, QGraphicsPathItem,
                                         QGraphicsScene, QGraphicsSimpleTextItem,
                                         QGraphicsView)

from .schedule import PacketScheduler
from .tracker import ACTION

ACTION_COLOR = QColor('#2563eb')
SERVICE_COLOR = QColor('#9333ea')
IDLE_COLOR = QColor('#94a3b8')
SLOW_COLOR = QColor('#dc2626')
FOCUS_COLOR = QColor('#f59e0b')
FADED = 0.22
TEXT_COLOR = QColor('#0f172a')
MUTED_TEXT = QColor('#64748b')

# packet type: (color, direction (+1 caller->server, -1 back), radius, shape, label)
PACKETS = {
    'goal':            ('#2563eb', +1, 8.5, 'circle', 'Goal'),
    'accepted':        ('#2563eb', -1, 6.0, 'ring', 'Accepted'),
    'feedback':        ('#16a34a', -1, 5.5, 'circle', 'Feedback'),
    'cancel':          ('#ea580c', +1, 9.5, 'cross', 'Cancel'),
    'result_ok':       ('#15803d', -1, 8.5, 'square', 'Succeeded'),
    'result_canceled': ('#d97706', -1, 8.5, 'square', 'Canceled'),
    'result_failed':   ('#dc2626', -1, 8.5, 'square', 'Aborted'),
    'request':         ('#9333ea', +1, 7.5, 'circle', 'Request'),
    'response':        ('#c026d3', -1, 7.5, 'square', 'Response'),
}
FLASH = {'result_ok': '#16a34a', 'result_canceled': '#d97706',
         'result_failed': '#dc2626', 'response': '#c026d3'}
# Node roles: blues for actions, purples/rose for services; servers are the darker shade.
ROLE_ORDER = ('action_server', 'action_client', 'service_server', 'service_client')
ROLE_COLORS = {
    'action_server': '#1e40af',
    'action_client': '#0e7490',
    'service_server': '#6b21a8',
    'service_client': '#be185d',
}
ROLE_LABELS = {   # (full, short) - short is used when the full tags don't fit
    'action_server': ('action server', 'A-srv'),
    'action_client': ('action client', 'A-cli'),
    'service_server': ('service server', 'S-srv'),
    'service_client': ('service client', 'S-cli'),
}
TRAVEL_TIME = 0.9
FEEDBACK_TRAVEL_TIME = 0.75

NODE_W, NODE_H = 190.0, 52.0
COLUMN_GAP, ROW_GAP = 380.0, 110.0


class NodeItem(QGraphicsItem):

    def __init__(self, name):
        super().__init__()
        self.name = name
        self.edges = []
        self.busy = False
        self.pinned = False
        self.roles = {}     # role -> count, or a short note for pseudo nodes
        self.focused = False
        self._press_pos = None
        self.setFlags(QGraphicsItem.ItemIsMovable | QGraphicsItem.ItemIsSelectable
                      | QGraphicsItem.ItemSendsGeometryChanges)
        self.setZValue(3)
        self.setCursor(Qt.OpenHandCursor)
        self.setToolTip(name)

    def boundingRect(self):
        return QRectF(-NODE_W / 2 - 4, -NODE_H / 2 - 4, NODE_W + 8, NODE_H + 8)

    def paint(self, p, option, widget=None):
        p.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(-NODE_W / 2, -NODE_H / 2, NODE_W, NODE_H)
        if self.busy:
            glow = QColor(ACTION_COLOR)
            glow.setAlpha(45)
            p.setPen(Qt.NoPen)
            p.setBrush(glow)
            p.drawRoundedRect(rect.adjusted(-4, -4, 4, 4), 11, 11)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor('#eff6ff') if self.busy else QColor('#ffffff'))
        p.drawRoundedRect(rect, 8, 8)

        # role stripe on the left edge, one band per role
        roles = [r for r in ROLE_ORDER if isinstance(self.roles, dict) and self.roles.get(r)]
        if roles:
            clip = QPainterPath()
            clip.addRoundedRect(rect, 8, 8)
            p.save()
            p.setClipPath(clip)
            band = NODE_H / len(roles)
            for i, role in enumerate(roles):
                p.fillRect(QRectF(rect.left(), rect.top() + i * band, 7, band + 0.5),
                           QColor(ROLE_COLORS[role]))
            p.restore()

        border = TEXT_COLOR if self.isSelected() else (ACTION_COLOR if self.busy else IDLE_COLOR)
        width = 2.2 if (self.busy or self.isSelected()) else 1.2
        if self.focused:
            border, width = FOCUS_COLOR, 3.2
        p.setPen(QPen(border, width))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(rect, 8, 8)

        left = 15 if roles else 10
        title = QFont()
        title.setBold(True)
        title.setPointSizeF(9.5)
        p.setFont(title)
        p.setPen(TEXT_COLOR)
        text_rect = rect.adjusted(left, 5, -10, -NODE_H / 2)
        name = QFontMetrics(title).elidedText(self.name, Qt.ElideLeft, int(text_rect.width()))
        p.drawText(text_rect, Qt.AlignLeft | Qt.AlignVCenter, name)

        sub_rect = rect.adjusted(left, NODE_H / 2, -8, -5)
        if roles:
            self._paint_role_tags(p, roles, sub_rect)
        elif self.roles:
            small = QFont()
            small.setPointSizeF(8)
            p.setFont(small)
            p.setPen(MUTED_TEXT)
            p.drawText(sub_rect, Qt.AlignLeft | Qt.AlignVCenter, str(self.roles))

    def _paint_role_tags(self, p, roles, area):
        """Small coloured tags such as 'action server', 'service client x2'."""
        tag_font = QFont()
        tag_font.setPointSizeF(7.2)
        tag_font.setBold(True)
        fm = QFontMetrics(tag_font)

        def texts(short):
            out = []
            for r in roles:
                label = ROLE_LABELS[r][1 if short else 0]
                count = self.roles[r]
                out.append(label + (f' \u00D7{count}' if count > 1 else ''))
            return out

        labels = texts(short=False)
        if sum(fm.horizontalAdvance(t) + 12 for t in labels) > area.width():
            labels = texts(short=True)
        p.setFont(tag_font)
        x = area.left()
        h = 14.0
        y = area.center().y() - h / 2
        for role, text in zip(roles, labels):
            w = fm.horizontalAdvance(text) + 8
            if x + w > area.right():
                break
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(ROLE_COLORS[role]))
            p.drawRoundedRect(QRectF(x, y, w, h), 4, 4)
            p.setPen(QColor('#ffffff'))
            p.drawText(QRectF(x, y, w, h), Qt.AlignCenter, text)
            x += w + 4

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionHasChanged:
            for e in self.edges:
                e.adjust()
        return super().itemChange(change, value)

    def mouseMoveEvent(self, event):
        self.pinned = True   # user placed it, layout leaves it alone
        super().mouseMoveEvent(event)

    def mousePressEvent(self, event):
        self._press_pos = event.scenePos()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        # a click (not a drag) focuses the view on this node
        if self._press_pos is not None and \
                (event.scenePos() - self._press_pos).manhattanLength() < 4 and self.scene():
            self.scene().node_clicked(self.name)
        self._press_pos = None
        super().mouseReleaseEvent(event)


class EdgeLabel(QGraphicsItem):

    def __init__(self, parent):
        super().__init__(parent)
        self.text = ''
        self.color = IDLE_COLOR
        self.setZValue(4)
        self._font = QFont()
        self._font.setPointSizeF(8.5)

    def set_text(self, text, color):
        if text != self.text or color != self.color:
            self.prepareGeometryChange()
            self.text, self.color = text, color
            self.update()

    def boundingRect(self):
        w = QFontMetrics(self._font).horizontalAdvance(self.text) + 12
        return QRectF(-w / 2, -9, w, 18)

    def paint(self, p, option, widget=None):
        p.setRenderHint(QPainter.Antialiasing)
        r = self.boundingRect()
        bg = QColor('#ffffff')
        bg.setAlpha(225)
        p.setPen(QPen(self.color, 1))
        p.setBrush(bg)
        p.drawRoundedRect(r, 9, 9)
        p.setFont(self._font)
        p.setPen(self.color)
        p.drawText(r, Qt.AlignCenter, self.text)


class EdgeItem(QGraphicsPathItem):

    def __init__(self, key, src, dst):
        super().__init__()
        self.key = key
        self.kind, self.name = key[0], key[1]
        self.src, self.dst = src, dst
        self.lane, self.lanes = 0, 1
        self.introspected = True
        self.active_count = 0
        self.ever_active = False
        self.highlight = False
        self.flash_color = None
        self.flash_left = 0.0
        self.pending_flash = None
        self.slow = False
        self.phase = 0.0
        self.setZValue(1)
        self.label = EdgeLabel(self)
        src.edges.append(self)
        dst.edges.append(self)
        self.adjust()
        self.refresh_label()

    def current_color(self):
        if self.flash_color is not None and self.flash_left > 0:
            return QColor(self.flash_color)
        if self.active_count > 0 and self.slow:
            return QColor(SLOW_COLOR)
        return QColor(self.base_color if self.active_count > 0 else IDLE_COLOR)

    def refresh_label(self):
        prefix = '\u25B6 ' if self.kind == ACTION else '\u21C4 '
        text = prefix + self.name
        if self.active_count > 1:
            text += f'  \u00D7{self.active_count}'
        if not self.introspected and self.kind != ACTION:
            text += '  (introspection off)'
        lit = self.active_count > 0 or self.flash_left > 0
        self.label.set_text(text, self.current_color() if lit else MUTED_TEXT)

    @property
    def base_color(self):
        return ACTION_COLOR if self.kind == ACTION else SERVICE_COLOR

    def detach(self):
        for n in (self.src, self.dst):
            if self in n.edges:
                n.edges.remove(self)

    def adjust(self):
        a, b = self.src.pos(), self.dst.pos()
        offset = (self.lane - (self.lanes - 1) / 2.0) * 34.0
        path = QPainterPath()
        span = abs(b.x() - a.x()) / COLUMN_GAP
        if b.x() > a.x() + NODE_W * 0.6 and span > 1.5 and abs(b.y() - a.y()) < NODE_H:
            # skips a column: arc over the nodes in between instead of hiding behind them
            p1 = QPointF(a.x(), a.y() - NODE_H / 2)
            p2 = QPointF(b.x(), b.y() - NODE_H / 2)
            lift = 70.0 + 45.0 * span + abs(offset)
            path.moveTo(p1)
            path.cubicTo(QPointF(p1.x() + 40, p1.y() - lift),
                         QPointF(p2.x() - 40, p2.y() - lift), p2)
        elif b.x() > a.x() + NODE_W * 0.6:
            p1 = QPointF(a.x() + NODE_W / 2, a.y())
            p2 = QPointF(b.x() - NODE_W / 2, b.y())
            dx = max(70.0, (p2.x() - p1.x()) * 0.5)
            path.moveTo(p1)
            path.cubicTo(QPointF(p1.x() + dx, p1.y() + offset),
                         QPointF(p2.x() - dx, p2.y() + offset), p2)
        elif b.x() < a.x() - NODE_W * 0.6:
            p1 = QPointF(a.x() - NODE_W / 2, a.y())
            p2 = QPointF(b.x() + NODE_W / 2, b.y())
            dx = max(70.0, (p1.x() - p2.x()) * 0.5)
            path.moveTo(p1)
            path.cubicTo(QPointF(p1.x() - dx, p1.y() + offset),
                         QPointF(p2.x() + dx, p2.y() + offset), p2)
        else:
            # same column: loop out to the right
            p1 = QPointF(a.x() + NODE_W / 2, a.y())
            p2 = QPointF(b.x() + NODE_W / 2, b.y())
            bulge = 90.0 + abs(offset) * 1.5
            path.moveTo(p1)
            path.cubicTo(QPointF(p1.x() + bulge, p1.y()), QPointF(p2.x() + bulge, p2.y()), p2)
        self.setPath(path)
        mid = path.pointAtPercent(0.5)
        # sit just above the line so packets stay visible
        self.label.setPos(mid.x(), mid.y() - 15)

    def boundingRect(self):
        return super().boundingRect().adjusted(-14, -14, 14, 14)

    def paint(self, p, option, widget=None):
        p.setRenderHint(QPainter.Antialiasing)
        active = self.active_count > 0
        color = self.current_color()
        if self.highlight:
            hl = QColor('#facc15')
            hl.setAlpha(150)
            p.setPen(QPen(hl, 10, Qt.SolidLine, Qt.RoundCap))
            p.drawPath(self.path())
        width = 3.0 if active else (2.4 if self.flash_left > 0 else 1.4)
        pen = QPen(color, width, Qt.SolidLine, Qt.RoundCap)
        if active:
            # marching dashes = "work in progress"
            pen.setDashPattern([4, 3])
            pen.setDashOffset(-self.phase)
        elif not self.introspected and self.kind != ACTION:
            pen.setStyle(Qt.DotLine)
        elif not self.ever_active:
            pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawPath(self.path())

        # arrow head at the server end
        path = self.path()
        tip = path.pointAtPercent(1.0)
        back = path.pointAtPercent(0.96)
        d = tip - back
        length = (d.x() ** 2 + d.y() ** 2) ** 0.5 or 1.0
        ux, uy = d.x() / length, d.y() / length
        size = 9.0
        left = QPointF(tip.x() - ux * size - uy * size * 0.55, tip.y() - uy * size + ux * size * 0.55)
        right = QPointF(tip.x() - ux * size + uy * size * 0.55, tip.y() - uy * size - ux * size * 0.55)
        p.setPen(Qt.NoPen)
        p.setBrush(color)
        p.drawPolygon(QPolygonF([tip, left, right]))

        prefix = '▶ ' if self.kind == ACTION else '⇄ '
        text = prefix + self.name
        if self.active_count > 1:
            text += f'  ×{self.active_count}'
        if not self.introspected and self.kind != ACTION:
            text += '  (introspection off)'
        self.label.set_text(text, color if (active or self.flash_left > 0) else MUTED_TEXT)


class PacketItem(QGraphicsItem):

    def __init__(self, edge, ptype, delay, duration):
        super().__init__()
        self.edge = edge
        self.ptype = ptype
        color, self.direction, self.r, self.shape, _ = PACKETS[ptype]
        self.color = QColor(color)
        self.wait = delay
        self.t = 0.0
        self.duration = duration
        self.setZValue(6)
        self.setVisible(False)

    def boundingRect(self):
        r = self.r * 2.4
        return QRectF(-r, -r, 2 * r, 2 * r)

    def advance_by(self, dt):
        """Move along the edge. Returns False when the packet arrived."""
        if self.wait > 0:
            self.wait -= dt
            if self.wait > 0:
                return True
            dt = -self.wait
        self.setVisible(True)
        self.t += dt / self.duration
        if self.t >= 1.0:
            return False
        s = self.t * self.t * (3 - 2 * self.t)   # ease in/out
        pct = s if self.direction > 0 else 1.0 - s
        self.setPos(self.edge.path().pointAtPercent(pct))
        return True

    def paint(self, p, option, widget=None):
        p.setRenderHint(QPainter.Antialiasing)
        r = self.r
        glow = QColor(self.color)
        glow.setAlpha(55)
        p.setPen(Qt.NoPen)
        p.setBrush(glow)
        p.drawEllipse(QPointF(0, 0), r * 2.1, r * 2.1)
        if self.shape == 'circle':
            p.setBrush(self.color)
            p.setPen(QPen(QColor('#ffffff'), 1.2))
            p.drawEllipse(QPointF(0, 0), r, r)
        elif self.shape == 'ring':
            p.setBrush(QColor('#ffffff'))
            p.setPen(QPen(self.color, 2.2))
            p.drawEllipse(QPointF(0, 0), r, r)
        elif self.shape == 'square':
            p.setBrush(self.color)
            p.setPen(QPen(QColor('#ffffff'), 1.2))
            p.drawRoundedRect(QRectF(-r, -r, 2 * r, 2 * r), 2.5, 2.5)
        elif self.shape == 'cross':
            p.setBrush(self.color)
            p.setPen(QPen(QColor('#ffffff'), 1.2))
            p.drawEllipse(QPointF(0, 0), r, r)
            p.setPen(QPen(QColor('#ffffff'), 2.2, Qt.SolidLine, Qt.RoundCap))
            k = r * 0.45
            p.drawLine(QPointF(-k, -k), QPointF(k, k))
            p.drawLine(QPointF(-k, k), QPointF(k, -k))


class FlowScene(QGraphicsScene):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.nodes = {}
        self.edges = {}
        self.packets = []
        self.isolated = set()
        self.clock = 0.0                      # animation time, advanced by tick()
        self.scheduler = PacketScheduler()    # keeps cause and effect in the animation
        self.on_node_clicked = None           # callback(name), set by the widget
        self.focus = None                     # node name the view is focused on
        tile = QPixmap(28, 28)
        tile.fill(QColor('#f8fafc'))
        painter = QPainter(tile)
        painter.setPen(QPen(QColor('#e2e8f0'), 1.6))
        painter.drawPoint(0, 0)
        painter.end()
        self.setBackgroundBrush(QBrush(tile))
        self._placeholder = QGraphicsSimpleTextItem(
            'Waiting for actions and services...\n\n'
            'Start your system, or try:  ros2 run rqt_execution_flow execution_flow_demo')
        self._placeholder.setBrush(MUTED_TEXT)
        self.addItem(self._placeholder)

    # ------------------------------------------------------------ topology
    def sync(self, specs, isolated, roles):
        """specs: {edge_key: introspected}; isolated: nodes to show without links."""
        changed = False
        for key, edge in list(self.edges.items()):
            if key not in specs and edge.active_count == 0 and not self._has_packets(edge):
                self._remove_edge(key)
                changed = True
        for key, introspected in specs.items():
            if key not in self.edges:
                self._add_edge(key)
                changed = True
            edge = self.edges[key]
            if edge.introspected != introspected:
                edge.introspected = introspected
                edge.refresh_label()
                edge.update()
        wanted = {k[2] for k in self.edges} | {k[3] for k in self.edges} | set(isolated)
        for name in list(self.nodes):
            if name not in wanted:
                self.removeItem(self.nodes.pop(name))
                changed = True
        for name in wanted:
            if name not in self.nodes:
                self._add_node(name)
                changed = True
        for name, node in self.nodes.items():
            text = roles.get(name, '')
            if text != node.roles:
                node.roles = text
                node.update()
        if changed:
            self._assign_lanes()
            self.layout()
        self._placeholder.setVisible(not self.nodes)
        if changed:
            self._apply_focus()
        return changed

    def ensure_edge(self, key):
        if key in self.edges:
            return False
        for name in key[2:4]:
            if name not in self.nodes:
                self._add_node(name)
        self._add_edge(key)
        self._assign_lanes()
        self.layout()
        self._placeholder.setVisible(False)
        self._apply_focus()
        return True

    def _add_node(self, name):
        node = NodeItem(name)
        self.addItem(node)
        self.nodes[name] = node

    def _add_edge(self, key):
        for name in key[2:4]:
            if name not in self.nodes:
                self._add_node(name)
        edge = EdgeItem(key, self.nodes[key[2]], self.nodes[key[3]])
        self.addItem(edge)
        self.edges[key] = edge

    def _remove_edge(self, key):
        edge = self.edges.pop(key)
        edge.detach()
        for pk in [pk for pk in self.packets if pk.edge is edge]:
            self.packets.remove(pk)
            self.removeItem(pk)
        self.removeItem(edge)

    def _has_packets(self, edge):
        return any(pk.edge is edge for pk in self.packets)

    def _assign_lanes(self):
        groups = defaultdict(list)
        for key, edge in self.edges.items():
            groups[frozenset((key[2], key[3]))].append(edge)
        for edges in groups.values():
            edges.sort(key=lambda e: (e.kind, e.name))
            for i, e in enumerate(edges):
                e.lane, e.lanes = i, len(edges)

    def layout(self):
        """Callers on the left, the servers they call to the right."""
        names = sorted(self.nodes)
        depth = {n: 0 for n in names}
        pairs = {(k[2], k[3]) for k in self.edges if k[2] != k[3]}
        for _ in range(len(names)):
            changed = False
            for s, d in pairs:
                if depth[s] + 1 > depth[d] and depth[s] + 1 < len(names):
                    depth[d] = depth[s] + 1
                    changed = True
            if not changed:
                break
        layers = defaultdict(list)
        for n in names:
            layers[depth[n]].append(n)
        ypos = {}
        preds = defaultdict(list)
        for s, d in pairs:
            preds[d].append(s)
        for col in sorted(layers):
            column = layers[col]
            # order by the average position of callers to reduce crossings
            column.sort(key=lambda n: (sum(ypos.get(p, 0) for p in preds[n]) /
                                       max(1, len(preds[n])), n))
            for i, n in enumerate(column):
                y = (i - (len(column) - 1) / 2.0) * ROW_GAP
                ypos[n] = y
                node = self.nodes[n]
                if not node.pinned:
                    node.setPos(col * COLUMN_GAP, y)
        for e in self.edges.values():
            e.adjust()
        self._placeholder.setVisible(not self.nodes)
        self.setSceneRect(self.content_rect().adjusted(-400, -300, 400, 300))

    # ----------------------------------------------------------- animation
    def emit_packet(self, key, ptype, delay=0.0):
        edge = self.edges.get(key)
        if edge is None or ptype not in PACKETS:
            return
        if ptype == 'feedback' and sum(1 for p in self.packets if p.edge is edge) > 40:
            return   # keep the scene responsive with very fast feedback
        forward = PACKETS[ptype][1] > 0
        depart, arrive = (edge.src, edge.dst) if forward else (edge.dst, edge.src)
        feedback = ptype == 'feedback'
        wait, travel = self.scheduler.schedule(
            self.clock, delay, depart.name, arrive.name,
            FEEDBACK_TRAVEL_TIME if feedback else TRAVEL_TIME, sets_ready=not feedback)
        pk = PacketItem(edge, ptype, wait, travel)
        self.addItem(pk)
        self.packets.append(pk)
        if ptype in FLASH:
            edge.pending_flash = (FLASH[ptype], wait + travel)

    def node_clicked(self, name):
        if self.on_node_clicked is not None:
            self.on_node_clicked(name)

    def set_focus(self, name):
        """Fade everything not connected to ``name``; None shows everything."""
        self.focus = name
        self._apply_focus()

    def _apply_focus(self):
        name = self.focus if self.focus in self.nodes else None
        near = set()
        for key, edge in self.edges.items():
            touches = name is None or name in (key[2], key[3])
            edge.setOpacity(1.0 if touches else FADED)
            if name is not None and touches:
                near.update((key[2], key[3]))
        for n, node in self.nodes.items():
            node.setOpacity(1.0 if name is None or n in near or n == name else FADED)
            if node.focused != (n == name):
                node.focused = n == name
                node.update()

    def set_activity(self, counts, busy_nodes, slow_keys=()):
        # The animation can run a little behind real time (see schedule.py).
        # Keep a link busy, and its server working, until its packets have landed.
        in_flight = {pk.edge.key for pk in self.packets if pk.ptype != 'feedback'}
        busy_nodes = set(busy_nodes) | {key[3] for key in in_flight}
        slow_keys = set(slow_keys)
        for key, edge in self.edges.items():
            c = counts.get(key, 0) or (1 if key in in_flight else 0)
            if c:
                edge.ever_active = True
            slow = key in slow_keys
            if c != edge.active_count or slow != edge.slow:
                edge.active_count = c
                edge.slow = slow
                edge.refresh_label()
                edge.update()
        for name, node in self.nodes.items():
            b = name in busy_nodes
            if b != node.busy:
                node.busy = b
                node.update()

    def set_highlight(self, key):
        for k, e in self.edges.items():
            h = (k == key)
            if h != e.highlight:
                e.highlight = h
                e.update()

    def tick(self, dt):
        self.clock += dt
        alive = []
        for pk in self.packets:
            if pk.advance_by(dt):
                alive.append(pk)
            else:
                self.removeItem(pk)
        self.packets = alive
        for e in self.edges.values():
            if e.pending_flash is not None:
                color, wait = e.pending_flash
                wait -= dt
                if wait <= 0:
                    e.flash_color, e.flash_left = QColor(color), 1.4
                    e.pending_flash = None
                    e.refresh_label()
                else:
                    e.pending_flash = (color, wait)
            if e.flash_left > 0:
                e.flash_left -= dt
                if e.flash_left <= 0:
                    e.refresh_label()
                e.update()
            if e.active_count > 0:
                e.phase = (e.phase + dt * 18.0) % 70.0
                e.update()

    def content_rect(self):
        rect = QRectF()
        for item in list(self.nodes.values()) + list(self.edges.values()):
            rect = rect.united(item.sceneBoundingRect())
        if not self.nodes:
            rect = self._placeholder.sceneBoundingRect()
        return rect

    def clear_packets(self):
        for pk in self.packets:
            self.removeItem(pk)
        self.packets = []
        self.scheduler.clear()


class FlowView(QGraphicsView):

    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.BoundingRectViewportUpdate)
        self.setCacheMode(QGraphicsView.CacheBackground)
        self._keep_fitted = True

    def wheelEvent(self, event):
        self._keep_fitted = False   # the user zoomed: stop auto-fitting
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.auto_fit()

    def auto_fit(self):
        if self._keep_fitted:
            self.fit()

    def fit(self, keep=False):
        if keep:
            self._keep_fitted = True
        rect = self.scene().content_rect().adjusted(-40, -40, 40, 40)
        if rect.isValid():
            self.fitInView(rect, Qt.KeepAspectRatio)
            # don't blow small graphs up too much
            if self.transform().m11() > 1.3:
                self.resetTransform()
                self.scale(1.3, 1.3)
                self.centerOn(rect.center())
