from rqt_execution_flow.recorder import Player, Recording
from rqt_execution_flow.tracker import GraphSnapshot, Tracker


def sample_recording():
    rec = Recording(start=100.0)
    rec.add_graph(100.0, GraphSnapshot(action_servers={'/a': {'/srv'}},
                                       action_clients={'/a': {'/cli'}},
                                       action_types={'/a': 'pkg/action/A'}))
    rec.add_event(('goal_status', 101.0, '/a', 'g1', 1))
    rec.add_event(('feedback', 101.5, '/a', 'g1', 'x: 1'))
    rec.add_event(('goal_status', 102.0, '/a', 'g1', 4))
    rec.finish(103.0)
    return rec


def test_unchanged_graph_is_stored_once():
    rec = sample_recording()
    rec.add_graph(101.2, GraphSnapshot(action_servers={'/a': {'/srv'}},
                                       action_clients={'/a': {'/cli'}},
                                       action_types={'/a': 'pkg/action/A'}))
    assert len(rec.graphs) == 1


def test_player_returns_items_in_time_order():
    p = Player(sample_recording())
    first = p.advance_to(101.2)
    assert [k for k, _ in first] == ['graph', 'event']
    assert abs(p.offset - 1.2) < 1e-9
    rest = p.advance_to(999)
    assert [e[0] for _, e in rest] == ['feedback', 'goal_status']
    assert p.at_end


def test_step_goes_one_event_at_a_time():
    p = Player(sample_recording())
    assert [k for k, _ in p.step()] == ['graph', 'event']
    assert p.pos == 101.0
    assert p.step()[0][1][0] == 'feedback'
    assert p.step()[0][1][0] == 'goal_status'
    assert p.step() == []
    assert p.at_end


def test_reset_replays_from_start():
    p = Player(sample_recording())
    p.advance_to(999)
    p.reset()
    assert len(p.advance_to(999)) == 4


def test_save_and_load_round_trip(tmp_path):
    rec = sample_recording()
    path = tmp_path / 'rec.json'
    rec.save(path)
    loaded = Recording.load(path)
    assert loaded.events == rec.events
    assert loaded.duration == rec.duration
    assert loaded.graphs[0][1].action_servers == {'/a': {'/srv'}}

    # a replay drives the tracker exactly like live events do
    t = Tracker()
    for kind, item in Player(loaded).advance_to(999):
        if kind == 'graph':
            t.set_graph(item)
        else:
            t.handle(item)
    op = next(iter(t.by_uid.values()))
    assert op.client == '/cli' and op.status == 'SUCCEEDED' and op.feedback_count == 1


def test_skip_to_jumps_forward_without_losing_events():
    p = Player(sample_recording())
    p.advance_to(101.0)
    p.skip_to(101.4)
    assert abs(p.pos - 101.4) < 1e-9
    assert [e[0] for _, e in p.advance_to(999)] == ['feedback', 'goal_status']
    p.skip_to(50.0)            # never goes backwards
    assert p.pos == 103.0
