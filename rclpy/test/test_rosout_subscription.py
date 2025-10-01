# Copyright 2023 Sony Group Corporation.
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

from typing import Optional
from typing import TYPE_CHECKING
import unittest

from rcl_interfaces.msg import Log
import rclpy
import rclpy.context
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.task import Future


class TestRosoutSubscription(unittest.TestCase):

    if TYPE_CHECKING:
        context: rclpy.context.Context
        node: rclpy.node.Node
        executor: SingleThreadedExecutor

    @classmethod
    def setUpClass(cls) -> None:
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('test_rosout_subscription', context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def setUp(self) -> None:
        # create subscriber of 'rosout' topic
        self.sub = self.node.create_subscription(
            Log,
            '/rosout',
            self._rosout_subscription_callback,
            1
        )
        self.fut: Future[None] = Future()
        self.rosout_msg_name: Optional[str] = None

    def _rosout_subscription_callback(self, msg: Log) -> None:
        if msg.name == self.rosout_msg_name:
            self.fut.set_result(None)

    def test_parent_log(self) -> None:
        self.rosout_msg_name = 'test_rosout_subscription'
        logger = self.node.get_logger()
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())

    def test_child_log(self) -> None:
        self.rosout_msg_name = 'test_rosout_subscription.child1'
        logger = self.node.get_logger()
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertFalse(self.fut.done())

        logger = self.node.get_logger().get_child('child1')
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())
        self.fut = Future()

        logger = self.node.get_logger().get_child('child2')
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertFalse(self.fut.done())

        self.rosout_msg_name = 'test_rosout_subscription.child2'
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())

    def test_child_hierarchy(self) -> None:
        self.rosout_msg_name = 'test_rosout_subscription.child.grandchild'
        logger = self.node.get_logger().get_child('child').get_child('grandchild')
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())

    def test_first_child_removed(self) -> None:
        self.rosout_msg_name = 'test_rosout_subscription.child'
        logger = self.node.get_logger().get_child('child')
        logger2 = self.node.get_logger().get_child('child')
        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())
        logger = None  # type: ignore[assignment]
        logger2.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())

    def test_logger_parameter(self) -> None:
        self.rosout_msg_name = 'test_rosout_subscription.child'
        logger = self.node.get_logger().get_child('child')

        def call_logger(logger: RcutilsLogger) -> None:
            logger1 = logger
            logger1.info('test')
        call_logger(logger)
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())

        logger.info('test')
        self.executor.spin_until_future_complete(self.fut, 3)
        self.assertTrue(self.fut.done())

    def test_logger_rosout_disabled_without_exception(self) -> None:
        node = rclpy.create_node('mynode', context=self.context, enable_rosout=False)
        try:
            logger = node.get_logger().get_child('child')
            logger.info('test')
        except Exception as e:
            self.fail(f'Not expected failure: {e}')


if __name__ == '__main__':
    unittest.main()
