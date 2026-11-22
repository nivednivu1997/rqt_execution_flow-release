"""Decide when each animated packet leaves, so the animation keeps cause and effect.

Packets take about a second to travel on screen, while the real messages take
milliseconds.  Without care, a "canceled" result could leave a server before
the cancel request has visibly reached it.

Rule: a packet may leave a node only after every earlier packet heading to that
node has arrived.  Things that happen within milliseconds of each other are
therefore stretched out in order.  To keep a busy system from falling further and
further behind, waits are capped and packets travel faster while there is a
backlog.  The timeline and durations always show real times; only the animation
is stretched.

No ROS or Qt imports, so it can be unit tested on its own.
"""

MAX_WAIT = 4.0          # never hold a packet back longer than this (animation seconds)
BACKLOG = 1.0           # above this wait, packets travel faster
FAST_FACTOR = 0.55      # travel time multiplier while there is a backlog
MIN_TRAVEL = 0.4


class PacketScheduler:

    def __init__(self):
        self.ready = {}     # node -> animation time when all packets heading there arrived

    def clear(self):
        self.ready.clear()

    def schedule(self, now, delay, depart, arrive, travel, sets_ready=True):
        """Return ``(wait, travel)`` for a packet that would leave after ``delay``.

        :param now: current animation time.
        :param depart: node the packet leaves from.
        :param arrive: node the packet goes to.
        :param sets_ready: False for packets that nothing waits for (feedback).
        """
        start = max(now + delay, self.ready.get(depart, 0.0))
        start = min(start, now + MAX_WAIT)
        if start - now > BACKLOG:
            travel = max(MIN_TRAVEL, travel * FAST_FACTOR)
        if sets_ready:
            self.ready[arrive] = max(self.ready.get(arrive, 0.0), start + travel)
        return start - now, travel
