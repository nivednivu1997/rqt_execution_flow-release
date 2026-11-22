^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_execution_flow
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-11-22)
------------------
* Flag slow or stuck calls: a configurable "Slow after" limit turns long-running
  calls red in the call tree and the flow view (`#6 <https://github.com/nivednivu1997/rqt_execution_flow/issues/6>`_)
* Statistics tab: calls, running, succeeded, canceled, failed, slow, average,
  95th-percentile and slowest time per action and service (`#7 <https://github.com/nivednivu1997/rqt_execution_flow/issues/7>`_)
* Click a node in the flow view to show only its calls (`#8 <https://github.com/nivednivu1997/rqt_execution_flow/issues/8>`_)
* "Show" list to filter calls by result: running, succeeded, canceled, failed,
  slow or problems; replaces "Hide finished" (`#9 <https://github.com/nivednivu1997/rqt_execution_flow/issues/9>`_)
* Colour nodes by role: action server, action client, service server,
  service client (`#3 <https://github.com/nivednivu1997/rqt_execution_flow/issues/3>`_)
* Keep cause and effect in the animation: a packet only leaves a node after
  earlier packets heading there have arrived (`#3 <https://github.com/nivednivu1997/rqt_execution_flow/issues/3>`_)
* Replay controls on their own row; the recording bar shows only the file name (`#1 <https://github.com/nivednivu1997/rqt_execution_flow/issues/1>`_)
* Clear warning when message types can't be loaded; fill in call types
  discovered after startup (`#1 <https://github.com/nivednivu1997/rqt_execution_flow/issues/1>`_)
* Documentation: service introspection for Python and C++, guides for every
  feature, new screenshots
* Contributors: Nived Krishnan

0.1.0 (2026-11-21)
------------------
* Initial version: live call tree, animated flow view, event timeline, details
  panel, record and slow-motion replay, warnings for duplicate node names and
  services without introspection, demo system
* Contributors: Nived Krishnan
