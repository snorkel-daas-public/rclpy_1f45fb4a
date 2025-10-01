# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from typing import List
from typing import TYPE_CHECKING
import unittest
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import qos_profile_action_status_default, qos_profile_system_default
from rclpy.service_introspection import ServiceIntrospectionState

from service_msgs.msg import ServiceEventInfo

from test_msgs.action import Fibonacci
from typing_extensions import TypeAlias

from unique_identifier_msgs.msg import UUID


FibonacciActionClient: TypeAlias = ActionClient[Fibonacci.Goal,
                                                Fibonacci.Result,
                                                Fibonacci.Feedback]

# TODO(jacobperron) Reduce fudge once wait_for_service uses node graph events
TIME_FUDGE = 0.3


class MockActionServer:

    def __init__(self, node: rclpy.node.Node):
        self.goal_srv = node.create_service(
            Fibonacci.Impl.SendGoalService, '/fibonacci/_action/send_goal',
            self.goal_callback)
        self.cancel_srv = node.create_service(
            Fibonacci.Impl.CancelGoalService, '/fibonacci/_action/cancel_goal',
            self.cancel_callback)
        self.result_srv = node.create_service(
            Fibonacci.Impl.GetResultService, '/fibonacci/_action/get_result',
            self.result_callback)
        self.feedback_pub = node.create_publisher(
            Fibonacci.Impl.FeedbackMessage, '/fibonacci/_action/feedback', 1)
        self.status_pub = node.create_publisher(
            Fibonacci.Impl.GoalStatusMessage,
            '/fibonacci/_action/status',
            qos_profile_action_status_default)

    def goal_callback(self, request: Fibonacci.Impl.SendGoalService.Request,
                      response: Fibonacci.Impl.SendGoalService.Response
                      ) -> Fibonacci.Impl.SendGoalService.Response:
        response.accepted = True
        return response

    def cancel_callback(self, request: Fibonacci.Impl.CancelGoalService.Request,
                        response: Fibonacci.Impl.CancelGoalService.Response
                        ) -> Fibonacci.Impl.CancelGoalService.Response:
        response.goals_canceling.append(request.goal_info)
        return response

    def result_callback(self, request: Fibonacci.Impl.GetResultService.Request,
                        response: Fibonacci.Impl.GetResultService.Response
                        ) -> Fibonacci.Impl.GetResultService.Response:
        return response

    def publish_feedback(self, goal_id: UUID) -> None:
        feedback_message = Fibonacci.Impl.FeedbackMessage()
        feedback_message.goal_id = goal_id
        self.feedback_pub.publish(feedback_message)


class TestActionClient(unittest.TestCase):

    if TYPE_CHECKING:
        context: rclpy.context.Context
        executor: SingleThreadedExecutor
        node: rclpy.node.Node
        mock_action_server: MockActionServer

    @classmethod
    def setUpClass(cls) -> None:
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.node = rclpy.create_node('TestActionClient', context=cls.context)
        cls.mock_action_server = MockActionServer(cls.node)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def setUp(self) -> None:
        self.feedback = None

    def feedback_callback(self, feedback: Fibonacci.Feedback) -> None:
        self.feedback = feedback

    def timed_spin(self, duration: float) -> None:
        start_time = time.time()
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

    def test_constructor_defaults(self) -> None:
        # Defaults
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')
        ac.destroy()

    def test_constructor_no_defaults(self) -> None:
        ac: FibonacciActionClient = ActionClient(
            self.node,
            Fibonacci,
            'fibonacci',
            goal_service_qos_profile=rclpy.qos.QoSProfile(depth=10),
            result_service_qos_profile=rclpy.qos.QoSProfile(depth=10),
            cancel_service_qos_profile=rclpy.qos.QoSProfile(depth=10),
            feedback_sub_qos_profile=rclpy.qos.QoSProfile(depth=10),
            status_sub_qos_profile=rclpy.qos.QoSProfile(depth=10)
        )
        ac.destroy()

    def test_get_num_entities(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')
        num_entities = ac.get_num_entities()
        self.assertEqual(num_entities.num_subscriptions, 2)
        self.assertEqual(num_entities.num_guard_conditions, 0)
        self.assertEqual(num_entities.num_timers, 0)
        self.assertEqual(num_entities.num_clients, 3)
        self.assertEqual(num_entities.num_services, 0)
        ac.destroy()

    def test_wait_for_server_nowait(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'not_fibonacci')
        try:
            start = time.monotonic()
            self.assertFalse(ac.wait_for_server(timeout_sec=0.0))
            end = time.monotonic()
            self.assertGreater(0.0, end - start - TIME_FUDGE)
            self.assertLess(0.0, end - start + TIME_FUDGE)
        finally:
            ac.destroy()

    def test_wait_for_server_timeout(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'not_fibonacci')
        try:
            start = time.monotonic()
            self.assertFalse(ac.wait_for_server(timeout_sec=2.0))
            end = time.monotonic()
            self.assertGreater(2.0, end - start - TIME_FUDGE)
            self.assertLess(2.0, end - start + TIME_FUDGE)
        finally:
            ac.destroy()

    def test_wait_for_server_exists(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            start = time.monotonic()
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))
            end = time.monotonic()
            self.assertGreater(0.0, end - start - TIME_FUDGE)
            self.assertLess(0.0, end - start + TIME_FUDGE)
        finally:
            ac.destroy()

    def test_send_goal_async(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))
            future = ac.send_goal_async(Fibonacci.Goal())
            rclpy.spin_until_future_complete(self.node, future, self.executor)
            self.assertTrue(future.done())
            goal_handle = future.result()
            assert goal_handle
            self.assertTrue(goal_handle.accepted)
        finally:
            ac.destroy()

    def test_send_goal_async_with_feedback_after_goal(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Send a goal and then publish feedback
            goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
            future = ac.send_goal_async(
                Fibonacci.Goal(),
                feedback_callback=self.feedback_callback,
                goal_uuid=goal_uuid)
            rclpy.spin_until_future_complete(self.node, future, self.executor)

            # Publish feedback after goal has been accepted
            self.mock_action_server.publish_feedback(goal_uuid)
            self.timed_spin(1.0)
            self.assertNotEqual(self.feedback, None)
        finally:
            ac.destroy()

    def test_send_goal_async_with_feedback_before_goal(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Publish feedback before goal has been accepted
            goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
            self.mock_action_server.publish_feedback(goal_uuid)
            self.timed_spin(1.0)
            self.assertEqual(self.feedback, None)

            # Send a goal and then publish feedback
            future = ac.send_goal_async(
                Fibonacci.Goal(),
                feedback_callback=self.feedback_callback,
                goal_uuid=goal_uuid)
            rclpy.spin_until_future_complete(self.node, future, self.executor)

            # Check the feedback was not received
            self.assertEqual(self.feedback, None)
        finally:
            ac.destroy()

    def test_send_goal_async_with_feedback_after_goal_result_requested(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Send a goal and wait for completion
            goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
            goal_future = ac.send_goal_async(
                Fibonacci.Goal(),
                feedback_callback=self.feedback_callback,
                goal_uuid=goal_uuid)
            rclpy.spin_until_future_complete(self.node, goal_future, self.executor)
            self.assertTrue(goal_future.done())
            # Then request result
            goal_handle = goal_future.result()
            assert goal_handle
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, self.executor)
            self.assertTrue(result_future.done())

            # Publish feedback after goal result is requested
            self.mock_action_server.publish_feedback(goal_uuid)
            self.timed_spin(1.0)
            self.assertEqual(self.feedback, None)
        finally:
            ac.destroy()

    def test_send_goal_async_with_feedback_for_another_goal(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Send a goal and then publish feedback
            first_goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
            future = ac.send_goal_async(
                Fibonacci.Goal(),
                feedback_callback=self.feedback_callback,
                goal_uuid=first_goal_uuid)
            rclpy.spin_until_future_complete(self.node, future, self.executor)

            # Send another goal, but without a feedback callback
            second_goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
            future = ac.send_goal_async(
                Fibonacci.Goal(),
                goal_uuid=second_goal_uuid)
            rclpy.spin_until_future_complete(self.node, future, self.executor)

            # Publish feedback for the second goal
            self.mock_action_server.publish_feedback(second_goal_uuid)
            self.timed_spin(1.0)
            self.assertEqual(self.feedback, None)
            # Publish feedback for the first goal (with callback)
            self.mock_action_server.publish_feedback(first_goal_uuid)
            self.timed_spin(1.0)
            self.assertNotEqual(self.feedback, None)
        finally:
            ac.destroy()

    def test_send_goal_async_with_feedback_for_not_a_goal(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Send a goal and then publish feedback
            goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
            future = ac.send_goal_async(
                Fibonacci.Goal(),
                feedback_callback=self.feedback_callback,
                goal_uuid=goal_uuid)
            rclpy.spin_until_future_complete(self.node, future, self.executor)

            # Publish feedback for a non-existent goal ID
            self.mock_action_server.publish_feedback(UUID(uuid=list(uuid.uuid4().bytes)))
            self.timed_spin(1.0)
            self.assertEqual(self.feedback, None)
        finally:
            ac.destroy()

    def test_send_goal_multiple(self) -> None:
        ac: FibonacciActionClient = ActionClient(
            self.node,
            Fibonacci,
            'fibonacci',
            callback_group=ReentrantCallbackGroup())
        executor = MultiThreadedExecutor(context=self.context)
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))
            future_0 = ac.send_goal_async(Fibonacci.Goal())
            future_1 = ac.send_goal_async(Fibonacci.Goal())
            future_2 = ac.send_goal_async(Fibonacci.Goal())
            rclpy.spin_until_future_complete(self.node, future_0, executor)
            rclpy.spin_until_future_complete(self.node, future_1, executor)
            rclpy.spin_until_future_complete(self.node, future_2, executor)
            self.assertTrue(future_0.done())
            self.assertTrue(future_1.done())
            self.assertTrue(future_2.done())
            future_0_result = future_0.result()
            future_1_result = future_1.result()
            future_2_result = future_2.result()
            assert future_0_result
            assert future_1_result
            assert future_2_result
            self.assertTrue(future_0_result.accepted)
            self.assertTrue(future_1_result.accepted)
            self.assertTrue(future_2_result.accepted)
        finally:
            ac.destroy()

    def test_send_goal_async_no_server(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'not_fibonacci')
        try:
            future = ac.send_goal_async(Fibonacci.Goal())
            self.timed_spin(2.0)
            self.assertFalse(future.done())
        finally:
            ac.destroy()

    def test_send_cancel_async(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Send a goal
            goal_future = ac.send_goal_async(Fibonacci.Goal())
            rclpy.spin_until_future_complete(self.node, goal_future, self.executor)
            self.assertTrue(goal_future.done())
            goal_handle = goal_future.result()

            # Cancel the goal
            assert goal_handle
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future, self.executor)
            self.assertTrue(cancel_future.done())
            cancel_result = cancel_future.result()
            assert cancel_result
            self.assertEqual(
                cancel_result.goals_canceling[0].goal_id,
                goal_handle.goal_id)
        finally:
            ac.destroy()

    def test_get_result_async(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Send a goal
            goal_future = ac.send_goal_async(Fibonacci.Goal())
            rclpy.spin_until_future_complete(self.node, goal_future, self.executor)
            self.assertTrue(goal_future.done())
            goal_handle = goal_future.result()

            # Get the goal result
            assert goal_handle
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, self.executor)
            self.assertTrue(result_future.done())
        finally:
            ac.destroy()

    def test_different_type_raises(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')
        try:
            with self.assertRaises(TypeError):
                ac.send_goal('different goal type')  # type: ignore[call-arg]
            with self.assertRaises(TypeError):
                ac.send_goal_async('different goal type')
        finally:
            ac.destroy()

    def test_action_introspection_default_status(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')

        self.event_messages: List[Fibonacci.Impl.SendGoalService.Event] = []

        def sub_callback(msg: Fibonacci.Impl.SendGoalService.Event) -> None:
            self.event_messages.append(msg)

        # There is no need to check if introspection is enabled for all internal services,
        # as the implementation in the RCL interface operates on the three internal services
        # simultaneously. So only check send_goal service event.
        send_goal_service_event_sub = self.node.create_subscription(
            Fibonacci.Impl.SendGoalService.Event,
            '/fibonacci/_action/send_goal/_service_event',
            sub_callback, 3)

        try:
            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Send a goal
            goal_future = ac.send_goal_async(Fibonacci.Goal())
            rclpy.spin_until_future_complete(self.node, goal_future, self.executor)
            self.assertTrue(goal_future.done())

            # By default, action client introspection is disabled.
            # So no service event message can be received.
            start = time.monotonic()
            end = start + 1.0
            while len(self.event_messages) < 1:
                rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
                now = time.monotonic()
                if now >= end:
                    break

            self.assertEqual(len(self.event_messages), 0)
        finally:
            self.node.destroy_subscription(send_goal_service_event_sub)
            ac.destroy()

    def test_configure_introspection_content(self) -> None:
        ac: FibonacciActionClient = ActionClient(self.node, Fibonacci, 'fibonacci')

        self.event_messages = []

        def sub_callback(msg: Fibonacci.Impl.SendGoalService.Event) -> None:
            self.event_messages.append(msg)

        # There is no need to check if introspection is enabled for all internal services,
        # as the implementation in the RCL interface operates on the three internal services
        # simultaneously. So only check send_goal service event.
        send_goal_service_event_sub = self.node.create_subscription(
            Fibonacci.Impl.SendGoalService.Event,
            '/fibonacci/_action/send_goal/_service_event',
            sub_callback, 3)

        try:
            ac.configure_introspection(self.node.get_clock(),
                                       qos_profile_system_default,
                                       ServiceIntrospectionState.CONTENTS)

            self.assertTrue(ac.wait_for_server(timeout_sec=2.0))

            # Send a goal
            goal_future = ac.send_goal_async(Fibonacci.Goal())
            rclpy.spin_until_future_complete(self.node, goal_future, self.executor)
            self.assertTrue(goal_future.done())

            start = time.monotonic()
            end = start + 5.0
            while len(self.event_messages) < 1:
                rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
                now = time.monotonic()
                self.assertTrue(now < end)

            self.assertEqual(len(self.event_messages), 1)

            self.assertEqual(self.event_messages[0].info.event_type, ServiceEventInfo.REQUEST_SENT)

            # For ServiceIntrospectionState.CONTENTS mode, the request or response section must
            # contain data. In this case, the request section must contain data.
            self.assertEqual(len(self.event_messages[0].request), 1)
            self.assertEqual(len(self.event_messages[0].response), 0)
        finally:
            self.node.destroy_subscription(send_goal_service_event_sub)
            ac.destroy()


if __name__ == '__main__':
    unittest.main()
