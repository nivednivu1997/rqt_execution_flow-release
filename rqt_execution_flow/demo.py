"""Demo system with nested actions and a service, to try the visualizer.

    orchestrator     --/main_task-->  main_task_server
    main_task_server --/get_data-->   data_server        (check before starting)
    main_task_server --/sub_task-->   sub_task_server
    sub_task_server  --/get_data-->   data_server

The orchestrator runs a repeating script: succeed, cancel, succeed, abort.

Run:  ros2 run rqt_execution_flow execution_flow_demo
"""

import random
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from action_tutorials_interfaces.action import Fibonacci
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from .introspection import enable_service_introspection

MAIN_TASK = '/main_task'
SUB_TASK = '/sub_task'
DATA_SERVICE = '/get_data'


def wait_for(future, timeout=30.0):
    end = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > end:
            return False
        time.sleep(0.01)
    return True


class DataServer(Node):
    """Answers /get_data after a short delay."""

    def __init__(self):
        super().__init__('data_server')
        self.create_service(Trigger, DATA_SERVICE, self.on_request,
                            callback_group=ReentrantCallbackGroup())
        enable_service_introspection(self)

    def on_request(self, request, response):
        time.sleep(random.uniform(0.2, 0.4))
        response.success = True
        response.message = f'value={random.uniform(1, 9):.2f}'
        return response


class SubTaskServer(Node):
    """Serves /sub_task. Calls /get_data every few steps and sends feedback."""

    def __init__(self):
        super().__init__('sub_task_server')
        group = ReentrantCallbackGroup()
        self.data = self.create_client(Trigger, DATA_SERVICE, callback_group=group)
        enable_service_introspection(self)
        self.server = ActionServer(
            self, Fibonacci, SUB_TASK, execute_callback=self.execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)

    def execute(self, goal_handle):
        seq = [0, 1]
        for i in range(goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result(sequence=seq)
            if i % 3 == 0:
                wait_for(self.data.call_async(Trigger.Request()), 5.0)
            seq.append(seq[-1] + seq[-2])
            goal_handle.publish_feedback(Fibonacci.Feedback(partial_sequence=seq))
            time.sleep(0.35)
        goal_handle.succeed()
        return Fibonacci.Result(sequence=seq)


class MainTaskServer(Node):
    """Serves /main_task. Checks /get_data, then runs a /sub_task goal."""

    def __init__(self):
        super().__init__('main_task_server')
        group = ReentrantCallbackGroup()
        self.data = self.create_client(Trigger, DATA_SERVICE, callback_group=group)
        enable_service_introspection(self)
        self.sub_task = ActionClient(self, Fibonacci, SUB_TASK, callback_group=group)
        self.server = ActionServer(
            self, Fibonacci, MAIN_TASK, execute_callback=self.execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)

    def execute(self, goal_handle):
        order = goal_handle.request.order
        if order < 3:
            time.sleep(0.8)
            self.get_logger().warn('task too short, aborting')
            goal_handle.abort()
            return Fibonacci.Result()

        # check before starting
        wait_for(self.data.call_async(Trigger.Request()), 5.0)
        goal_handle.publish_feedback(Fibonacci.Feedback(partial_sequence=[0]))

        self.sub_task.wait_for_server(timeout_sec=5.0)
        send = self.sub_task.send_goal_async(Fibonacci.Goal(order=order))
        if not wait_for(send, 5.0) or not send.result().accepted:
            goal_handle.abort()
            return Fibonacci.Result()
        child = send.result()
        result_future = child.get_result_async()
        step = 1
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                child.cancel_goal_async()
                wait_for(result_future, 5.0)
                goal_handle.canceled()
                return Fibonacci.Result()
            goal_handle.publish_feedback(Fibonacci.Feedback(partial_sequence=[0, step]))
            step += 1
            time.sleep(0.5)
        if result_future.result().status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return Fibonacci.Result(sequence=result_future.result().result.sequence)


class Orchestrator(Node):
    """Top-level caller. Runs a repeating script of /main_task goals."""

    SCRIPT = [('succeed', 8), ('cancel', 10), ('succeed', 6), ('abort', 1)]

    def __init__(self):
        super().__init__('orchestrator')
        self.client = ActionClient(self, Fibonacci, MAIN_TASK,
                                   callback_group=ReentrantCallbackGroup())
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        self.client.wait_for_server(timeout_sec=10.0)
        time.sleep(2.0)
        i = 0
        while not self._stop.is_set() and rclpy.ok():
            plan, order = self.SCRIPT[i % len(self.SCRIPT)]
            i += 1
            self.get_logger().info(f'task {i}: {plan} (order {order})')
            send = self.client.send_goal_async(Fibonacci.Goal(order=order))
            if not wait_for(send, 5.0) or not send.result().accepted:
                continue
            handle = send.result()
            result = handle.get_result_async()
            if plan == 'cancel':
                time.sleep(3.0)
                handle.cancel_goal_async()
            wait_for(result, 60.0)
            time.sleep(2.5)

    def stop(self):
        self._stop.set()


def main(args=None):
    rclpy.init(args=args)
    nodes = [DataServer(), SubTaskServer(), MainTaskServer(), Orchestrator()]
    executor = MultiThreadedExecutor(num_threads=8)
    for n in nodes:
        executor.add_node(n)
    nodes[-1].thread.start()
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodes[-1].stop()
        executor.shutdown()
        for n in nodes:
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
