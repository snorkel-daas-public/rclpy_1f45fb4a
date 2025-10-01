# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import threading
import time
from types import TracebackType
from typing import Optional
from typing import Type
import unittest

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.waitable import NumberOfEntities
from rclpy.waitable import Waitable
from test_msgs.msg import BasicTypes
from test_msgs.srv import BasicTypes as BasicTypesSrv


# Arbitrary sleep time
TIMEOUT = 0.5


class TestCreateWhileSpinning(unittest.TestCase):
    """
    Test that the executor wakes after an entity is created.

    This is a regression test for ros2/rclpy#188.
    """

    def setUp(self) -> None:
        rclpy.init()
        self.node = rclpy.create_node('TestCreateWhileSpinning', namespace='/rclpy')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.exec_thread = threading.Thread(target=self.executor.spin)
        self.exec_thread.start()
        # Make sure executor is blocked by rcl_wait
        time.sleep(TIMEOUT)

    def tearDown(self) -> None:
        self.executor.shutdown()
        rclpy.shutdown()
        self.exec_thread.join()
        self.node.destroy_node()

    def test_publish_subscribe(self) -> None:
        evt = threading.Event()
        self.node.create_subscription(BasicTypes, 'foo', lambda msg: evt.set(), 1)
        pub = self.node.create_publisher(BasicTypes, 'foo', 1)
        pub.publish(BasicTypes())
        assert evt.wait(TIMEOUT)

    def test_client_server(self) -> None:
        evt = threading.Event()

        def trigger_event(req: BasicTypesSrv.Request,
                          resp: BasicTypesSrv.Response) -> BasicTypesSrv.Response:
            nonlocal evt
            evt.set()
            return resp

        self.node.create_service(BasicTypesSrv, 'foo', trigger_event)
        cli: Client[BasicTypesSrv.Request, BasicTypesSrv.Response] = \
            self.node.create_client(BasicTypesSrv, 'foo')
        cli.wait_for_service()
        cli.call_async(BasicTypesSrv.Request())
        assert evt.wait(TIMEOUT)

    def test_guard_condition(self) -> None:
        evt = threading.Event()

        guard = self.node.create_guard_condition(lambda: evt.set())
        guard.trigger()
        assert evt.wait(TIMEOUT)

    def test_timer(self) -> None:
        evt = threading.Event()

        self.node.create_timer(TIMEOUT / 10, lambda: evt.set())
        assert evt.wait(TIMEOUT)

    def test_waitable(self) -> None:
        evt = threading.Event()

        class DummyWaitable(Waitable[None]):

            def __init__(self) -> None:
                super().__init__(ReentrantCallbackGroup())

            def __enter__(self) -> None:
                pass

            def __exit__(
                self,
                exc_type: Optional[Type[BaseException]],
                exc_val: Optional[BaseException],
                exc_tb: Optional[TracebackType],
            ) -> None:
                pass

            def is_ready(self, wait_set: _rclpy.WaitSet) -> bool:
                return False

            def take_data(self) -> None:
                return None

            async def execute(self, taken_data: None) -> None:
                pass

            def get_num_entities(self) -> NumberOfEntities:
                return NumberOfEntities(0, 0, 0, 0, 0)

            def add_to_wait_set(self, wait_set: _rclpy.WaitSet) -> None:
                nonlocal evt
                evt.set()
                pass

        self.node.add_waitable(DummyWaitable())
        assert evt.wait(TIMEOUT)


if __name__ == '__main__':
    unittest.main()
