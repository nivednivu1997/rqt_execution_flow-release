from rqt_execution_flow.schedule import MAX_WAIT, PacketScheduler

T = 0.9   # travel time


def test_result_waits_until_cancel_has_arrived():
    s = PacketScheduler()
    s.schedule(0.0, 0.0, '/client', '/server', T)                   # cancel
    wait, _ = s.schedule(0.1, 0.25, '/server', '/client', T)          # canceled result
    assert 0.1 + wait >= 0.9


def test_cancel_chain_keeps_cause_and_effect():
    s = PacketScheduler()
    starts = {}
    # orchestrator cancels main_task, main_task_server cancels sub_task,
    # sub_task returns CANCELED, then main_task returns CANCELED
    for name, now, dep, arr in [('cancel_main', 0.00, '/orch', '/main'),
                                ('cancel_sub', 0.17, '/main', '/sub'),
                                ('result_sub', 0.47, '/sub', '/main'),
                                ('result_main', 0.50, '/main', '/orch')]:
        wait, travel = s.schedule(now, 0.0, dep, arr, T)
        starts[name] = (now + wait, now + wait + travel)
    assert starts['cancel_sub'][0] >= starts['cancel_main'][1]
    assert starts['result_sub'][0] >= starts['cancel_sub'][1]
    assert starts['result_main'][0] >= starts['result_sub'][1]


def test_feedback_waits_but_holds_nothing_back():
    s = PacketScheduler()
    s.schedule(0.0, 0.0, '/client', '/server', T)                         # goal
    wait, _ = s.schedule(0.05, 0.0, '/server', '/client', T, sets_ready=False)
    assert 0.05 + wait >= 0.9                                              # after the goal
    assert s.ready.get('/client', 0.0) == 0.0


def test_busy_system_never_falls_far_behind():
    s = PacketScheduler()
    worst = 0.0
    for i in range(200):          # a service called 20 times a second
        now = i * 0.05
        w1, _ = s.schedule(now, 0.0, '/c', '/s', T)
        w2, _ = s.schedule(now + 0.01, 0.0, '/s', '/c', T)
        worst = max(worst, w1, w2)
    assert worst <= MAX_WAIT + 1e-9


def test_unrelated_nodes_do_not_wait():
    s = PacketScheduler()
    s.schedule(0.0, 0.0, '/a', '/b', T)
    wait, travel = s.schedule(0.0, 0.0, '/c', '/d', T)
    assert wait == 0.0 and travel == T
