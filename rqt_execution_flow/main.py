"""Run the plugin on its own:  ros2 run rqt_execution_flow rqt_execution_flow"""

import sys

from rqt_gui.main import Main


def main():
    sys.exit(Main().main(sys.argv, standalone='rqt_execution_flow.plugin.ExecutionFlowPlugin'))


if __name__ == '__main__':
    main()
