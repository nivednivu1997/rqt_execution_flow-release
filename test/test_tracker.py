from rqt_execution_flow.tracker import GraphSnapshot, Tracker


def make_tracker():
    t = Tracker()
    t.set_graph(GraphSnapshot(
        action_servers={'/main_task': {'/main_task_server'}, '/sub_task': {'/sub_task_server'}},
        action_clients={'/main_task': {'/orchestrator'}, '/sub_task': {'/main_task_server'}},
        service_servers={'/get_data': {'/data_server'}},
        service_clients={'/get_data': {'/sub_task_server', '/main_task_server'}},
        introspected={'/get_data'},
    ))
    return t


def kinds(events, kind):
    return [e for e in events if e.kind == kind]


def packets(events):
    return [e.packet for e in events if e.kind == 'packet']


def test_nested_calls_build_a_tree():
    t = make_tracker()
    t.handle(('goal_status', 1.0, '/main_task', 'aa', 1))
    t.handle(('goal_status', 1.1, '/main_task', 'aa', 2))
    t.handle(('goal_status', 1.5, '/sub_task', 'bb', 2))
    t.handle(('service_event', 2.0, '/get_data', 0, '/sub_task_server', '/sub_task_server', 1, None, 0.0))

    main = t.by_key[('action', '/main_task', 'aa')]
    sub = t.by_key[('action', '/sub_task', 'bb')]
    call = t.by_key[('service', '/get_data', '/sub_task_server', 1)]
    assert main.client == '/orchestrator' and main.server == '/main_task_server'
    assert sub.parent is main
    assert call.parent is sub
    assert t.roots == [main]


def test_cancel_and_result_packets():
    t = make_tracker()
    ev = t.handle(('goal_status', 1.0, '/main_task', 'aa', 2))
    assert packets(ev) == ['goal', 'accepted']
    assert packets(t.handle(('goal_status', 2.0, '/main_task', 'aa', 3))) == ['cancel']
    ev = t.handle(('goal_status', 3.0, '/main_task', 'aa', 5))
    assert packets(ev) == ['result_canceled']
    op = t.by_key[('action', '/main_task', 'aa')]
    assert op.outcome == 'canceled' and not op.active and op.cancel_requested


def test_missed_canceling_still_shows_cancel():
    t = make_tracker()
    t.handle(('goal_status', 1.0, '/main_task', 'aa', 2))
    assert packets(t.handle(('goal_status', 2.0, '/main_task', 'aa', 5))) == [
        'cancel', 'result_canceled']


def test_abort_is_failure():
    t = make_tracker()
    t.handle(('goal_status', 1.0, '/main_task', 'aa', 1))
    ev = t.handle(('goal_status', 2.0, '/main_task', 'aa', 6))
    assert packets(ev) == ['result_failed']
    assert t.by_key[('action', '/main_task', 'aa')].outcome == 'failed'


def test_feedback_counts_and_is_throttled():
    t = make_tracker()
    t.handle(('goal_status', 1.0, '/main_task', 'aa', 2))
    sent = 0
    for i in range(10):
        sent += packets(t.handle(('feedback', 2.0 + i * 0.05, '/main_task', 'aa', 'x'))).count(
            'feedback')
    op = t.by_key[('action', '/main_task', 'aa')]
    assert op.feedback_count == 10
    assert 0 < sent < 10


def test_feedback_after_result_is_ignored():
    t = make_tracker()
    t.handle(('goal_status', 1.0, '/main_task', 'aa', 4))
    assert t.handle(('feedback', 2.0, '/main_task', 'aa', 'x')) == []


def test_service_both_sides_make_one_call():
    t = make_tracker()
    t.handle(('service_event', 1.0, '/get_data', 0, '/sub_task_server', '/sub_task_server', 7, 'req', 10.0))
    t.handle(('service_event', 1.01, '/get_data', 1, '/sub_task_server', '/sub_task_server', 7, 'req', 20.0))
    t.handle(('service_event', 1.3, '/get_data', 2, '/sub_task_server', '/sub_task_server', 7, 'res', 20.3))
    ev = t.handle(('service_event', 1.31, '/get_data', 3, '/sub_task_server', '/sub_task_server', 7, 'res', 10.31))
    assert len(t.by_uid) == 1
    op = next(iter(t.by_uid.values()))
    assert op.status == 'RESPONSE RECEIVED' and op.outcome == 'ok'
    assert op.request_text == 'req' and op.response_text == 'res'
    assert abs((op.stamps[3] - op.stamps[0]) - 0.31) < 1e-6
    assert kinds(ev, 'packet') == []   # response packet was sent only once


def test_ambiguous_service_caller_uses_busy_node():
    t = make_tracker()
    t.handle(('goal_status', 1.0, '/sub_task', 'bb', 2))   # /sub_task_server busy
    t.handle(('service_event', 2.0, '/get_data', 1, None, 'gid1', 1, None, 0.0))
    op = t.by_key[('service', '/get_data', 'gid1', 1)]
    assert op.client == '/sub_task_server' and 'guessed' in op.client_note


def test_old_roots_are_pruned():
    t = Tracker(max_roots=3)
    for i in range(6):
        t.handle(('service_event', float(i), '/s', 0, '/c', '/c', i, None, 0.0))
        t.handle(('service_event', i + 0.5, '/s', 3, '/c', '/c', i, None, 0.0))
    assert len(t.roots) == 3
    assert len(t.by_uid) == 3


def test_service_events_out_of_order_keep_real_duration():
    t = make_tracker()
    # server's response arrives before the client's request event
    t.handle(('service_event', 1.30, '/get_data', 2, '/sub_task_server', '/sub_task_server', 9, None, 0.0))
    t.handle(('service_event', 1.00, '/get_data', 0, '/sub_task_server', '/sub_task_server', 9, None, 0.0))
    op = t.by_key[('service', '/get_data', '/sub_task_server', 9)]
    assert abs(op.duration(0) - 0.30) < 1e-9
    assert [h[1] for h in op.history] == ['REQUEST SENT', 'RESPONSE SENT']


def test_unknown_caller_is_filled_in_when_graph_catches_up():
    t = Tracker()
    t.set_graph(GraphSnapshot(action_servers={'/main_task': {'/main_task_server'}}))
    t.handle(('goal_status', 1.0, '/main_task', 'aa', 2))
    op = t.by_key[('action', '/main_task', 'aa')]
    assert op.client is None
    changed = t.set_graph(GraphSnapshot(action_servers={'/main_task': {'/main_task_server'}},
                                        action_clients={'/main_task': {'/orchestrator'}}))
    assert changed == [op] and op.client == '/orchestrator'


def test_baseline_ignores_old_results_but_finishes_known_goals():
    t = make_tracker()
    # an old goal that finished before we watched: ignored
    assert t.handle(('goal_status', 1.0, '/main_task', 'old', 4, True)) == []
    # a goal we show as running, whose result arrives in the baseline window: finished
    t.handle(('goal_status', 1.0, '/main_task', 'run', 2, True))
    t.handle(('goal_status', 1.2, '/main_task', 'run', 4, True))
    op = t.by_key[('action', '/main_task', 'run')]
    assert op.status == 'SUCCEEDED' and not op.active
    assert ('action', '/main_task', 'old') not in t.by_key


def test_finished_call_with_unknown_ends_is_filled_in_later():
    t = Tracker()
    t.handle(('service_event', 1.0, '/get_data', 2, None, 'gid', 1, None, 0.0))
    op = next(iter(t.by_uid.values()))
    assert not op.active and op.server is None
    t.set_graph(GraphSnapshot(service_servers={'/get_data': {'/data_server'}},
                              service_clients={'/get_data': {'/sub_task_server'}}))
    assert op.server == '/data_server' and op.client == '/sub_task_server'


def test_call_seen_only_at_the_end_is_partial():
    t = make_tracker()
    t.handle(('service_event', 1.0, '/get_data', 3, '/sub_task_server', '/sub_task_server', 4, None, 0.0))
    assert next(iter(t.by_uid.values())).partial


def test_late_parent_adopts_children_already_shown_as_roots():
    t = make_tracker()
    # the child goal and a call are seen first...
    t.handle(('goal_status', 1.5, '/sub_task', 'bb', 2))
    t.handle(('service_event', 1.2, '/get_data', 0, '/main_task_server', '/main_task_server',
              1, None, 0.0))
    # ...then the parent's status arrives, with its real (earlier) start time
    t.handle(('goal_status', 1.0, '/main_task', 'aa', 2))
    main = t.by_key[('action', '/main_task', 'aa')]
    sub = t.by_key[('action', '/sub_task', 'bb')]
    assert t.roots == [main]
    assert [c.label for c in main.children] == ['/get_data #seq 1', sub.label]


def test_missing_type_is_filled_in_later():
    t = Tracker()
    t.handle(('service_event', 1.0, '/sensor', 0, '/c', '/c', 1, None, 0.0))
    op = next(iter(t.by_uid.values()))
    assert op.type == ''
    t.set_graph(GraphSnapshot(service_servers={'/sensor': {'/s'}},
                              service_types={'/sensor': 'pkg/srv/Thing'}))
    assert op.type == 'pkg/srv/Thing'


def test_statistics_count_outcomes_and_timings():
    t = make_tracker()
    t.handle(('goal_status', 0.0, '/main_task', 'a', 2))
    t.handle(('goal_status', 2.0, '/main_task', 'a', 4))       # ok, 2 s
    t.handle(('goal_status', 3.0, '/main_task', 'b', 2))
    t.handle(('goal_status', 3.5, '/main_task', 'b', 5))       # canceled, 0.5 s
    t.handle(('goal_status', 4.0, '/main_task', 'c', 2))
    t.handle(('goal_status', 10.0, '/main_task', 'c', 6))      # failed, 6 s
    t.handle(('goal_status', 11.0, '/main_task', 'd', 2))      # still running
    st = t.stats['/main_task']
    assert (st.calls, st.ok, st.canceled, st.failed) == (4, 1, 1, 1)
    timing = st.timing(slow_limit=5.0)
    assert abs(timing['avg'] - 8.5 / 3) < 1e-9
    assert timing['max'] == 6.0 and timing['slow'] == 1
    assert timing['p95'] == 6.0          # 3 samples: the 95th percentile is the top one


def test_statistics_survive_pruning_and_reset_on_clear():
    t = Tracker(max_roots=2)
    for i in range(5):
        t.handle(('service_event', float(i), '/s', 0, '/c', '/c', i, None, 0.0))
        t.handle(('service_event', i + 0.2, '/s', 3, '/c', '/c', i, None, 0.0))
    assert len(t.roots) == 2
    assert t.stats['/s'].calls == 5 and len(t.stats['/s'].durations) == 5
    t.clear()
    assert t.stats == {}


def test_partial_calls_are_counted_but_not_timed():
    t = make_tracker()
    t.handle(('service_event', 1.0, '/get_data', 3, '/c', '/c', 1, None, 0.0))
    st = t.stats['/get_data']
    assert st.calls == 1 and st.ok == 1 and st.timing(5.0)['avg'] is None
