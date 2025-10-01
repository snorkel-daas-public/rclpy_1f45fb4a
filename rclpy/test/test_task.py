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

import asyncio
from typing import Any
from typing import Callable
from typing import List
from typing import Tuple
import unittest

from rclpy.task import Future
from rclpy.task import Task


class DummyExecutor:

    def __init__(self) -> None:
        self.done_callbacks: List[Tuple[Callable[..., Any], Any]] = []

    def create_task(self, cb: Callable[..., Any], *args: Any) -> None:
        self.done_callbacks.append((cb, args))


class TestTask(unittest.TestCase):

    def test_task_normal_callable(self) -> None:

        def func() -> str:
            return 'Sentinel Result'

        t = Task(func)
        t()
        self.assertTrue(t.done())
        self.assertEqual('Sentinel Result', t.result())

    def test_task_lambda(self) -> None:

        def func() -> str:
            return 'Sentinel Result'

        t = Task(lambda: func())
        t()
        self.assertTrue(t.done())
        self.assertEqual('Sentinel Result', t.result())

    def test_coroutine(self) -> None:
        called1 = False
        called2 = False

        async def coro() -> str:
            nonlocal called1
            nonlocal called2
            called1 = True
            await asyncio.sleep(0)
            called2 = True
            return 'Sentinel Result'

        t = Task(coro)
        t()
        self.assertTrue(called1)
        self.assertFalse(called2)

        called1 = False
        t()
        self.assertFalse(called1)
        self.assertTrue(called2)
        self.assertTrue(t.done())
        self.assertEqual('Sentinel Result', t.result())

    def test_done_callback_scheduled(self) -> None:
        executor = DummyExecutor()

        t = Task(lambda: None, executor=executor)  # type: ignore[call-overload]
        t.add_done_callback('Sentinel Value')
        t()
        self.assertTrue(t.done())
        self.assertEqual(1, len(executor.done_callbacks))
        self.assertEqual('Sentinel Value', executor.done_callbacks[0][0])
        args = executor.done_callbacks[0][1]
        self.assertEqual(1, len(args))
        self.assertEqual(t, args[0])

    def test_done_task_done_callback_scheduled(self) -> None:
        executor = DummyExecutor()

        t = Task(lambda: None, executor=executor)  # type: ignore[call-overload]
        t()
        self.assertTrue(t.done())
        t.add_done_callback('Sentinel Value')
        self.assertEqual(1, len(executor.done_callbacks))
        self.assertEqual('Sentinel Value', executor.done_callbacks[0][0])
        args = executor.done_callbacks[0][1]
        self.assertEqual(1, len(args))
        self.assertEqual(t, args[0])

    def test_done_task_called(self) -> None:
        called = False

        def func() -> None:
            nonlocal called
            called = True

        t = Task(func)
        t()
        self.assertTrue(called)
        self.assertTrue(t.done())
        called = False
        t()
        self.assertFalse(called)
        self.assertTrue(t.done())

    def test_cancelled(self) -> None:
        t = Task(lambda: None)
        t.cancel()
        self.assertTrue(t.cancelled())

    def test_done_task_cancelled(self) -> None:
        t = Task(lambda: None)
        t()
        t.cancel()
        self.assertFalse(t.cancelled())

    def test_exception(self) -> None:

        def func() -> None:
            e = Exception()
            e.sentinel_value = 'Sentinel Exception'  # type: ignore[attr-defined]
            raise e

        t = Task(func)
        t()
        self.assertTrue(t.done())
        self.assertEqual('Sentinel Exception',
                         t.exception().sentinel_value)  # type: ignore[union-attr]
        with self.assertRaises(Exception):
            t.result()

    def test_coroutine_exception(self) -> None:

        async def coro() -> None:
            e = Exception()
            e.sentinel_value = 'Sentinel Exception'  # type: ignore[attr-defined]
            raise e

        t = Task(coro)
        t()
        self.assertTrue(t.done())
        self.assertEqual('Sentinel Exception',
                         t.exception().sentinel_value)  # type: ignore[union-attr]
        with self.assertRaises(Exception):
            t.result()

    def test_task_normal_callable_args(self) -> None:
        arg_in = 'Sentinel Arg'

        def func(arg: Any) -> Any:
            return arg

        t = Task(func, args=(arg_in,))
        t()
        self.assertEqual('Sentinel Arg', t.result())

    def test_coroutine_args(self) -> None:
        arg_in = 'Sentinel Arg'

        async def coro(arg: Any) -> Any:
            return arg

        t = Task(coro, args=(arg_in,))
        t()
        self.assertEqual('Sentinel Arg', t.result())

    def test_task_normal_callable_kwargs(self) -> None:
        arg_in = 'Sentinel Arg'

        def func(kwarg: Any = None) -> Any:
            return kwarg

        t = Task(func, kwargs={'kwarg': arg_in})
        t()
        self.assertEqual('Sentinel Arg', t.result())

    def test_coroutine_kwargs(self) -> None:
        arg_in = 'Sentinel Arg'

        async def coro(kwarg: Any = None) -> Any:
            return kwarg

        t = Task(coro, kwargs={'kwarg': arg_in})
        t()
        self.assertEqual('Sentinel Arg', t.result())

    def test_executing(self) -> None:
        t = Task(lambda: None)
        self.assertFalse(t.executing())


class TestFuture(unittest.TestCase):

    def test_cancelled(self) -> None:
        f: Future[Any] = Future()
        f.cancel()
        self.assertTrue(f.cancelled())

    def test_done(self) -> None:
        f: Future[None] = Future()
        self.assertFalse(f.done())
        f.set_result(None)
        self.assertTrue(f.done())

    def test_set_result(self) -> None:
        f: Future[str] = Future()
        f.set_result('Sentinel Result')
        self.assertEqual('Sentinel Result', f.result())
        self.assertTrue(f.done())

    def test_set_exception(self) -> None:
        f: Future[Any] = Future()
        f.set_exception('Sentinel Exception')   # type: ignore[arg-type]
        self.assertEqual('Sentinel Exception', f.exception())
        self.assertTrue(f.done())

    def test_await(self) -> None:
        f: Future[str] = Future()

        async def coro() -> Any:
            nonlocal f
            return await f

        c = coro()
        c.send(None)
        f.set_result('Sentinel Result')
        try:
            c.send(None)
        except StopIteration as e:
            self.assertEqual('Sentinel Result', e.value)

    def test_await_exception(self) -> None:
        f: Future[Any] = Future()

        async def coro() -> Any:
            nonlocal f
            return await f

        c = coro()
        c.send(None)
        f.set_exception(RuntimeError('test exception'))
        with self.assertRaises(RuntimeError):
            c.send(None)

    def test_cancel_schedules_callbacks(self) -> None:
        executor = DummyExecutor()
        f: Future[Any] = Future(executor=executor)  # type: ignore[arg-type]
        f.add_done_callback(lambda f: None)
        f.cancel()
        self.assertTrue(executor.done_callbacks)

    def test_set_result_schedules_callbacks(self) -> None:
        executor = DummyExecutor()
        f: Future[str] = Future(executor=executor)  # type: ignore[arg-type]
        f.add_done_callback(lambda f: None)
        f.set_result('Anything')
        self.assertTrue(executor.done_callbacks)

    def test_set_exception_schedules_callbacks(self) -> None:
        executor = DummyExecutor()
        f: Future[Any] = Future(executor=executor)  # type: ignore[arg-type]
        f.add_done_callback(lambda f: None)
        f.set_exception(Exception('Anything'))
        self.assertTrue(executor.done_callbacks)

    def test_cancel_invokes_callbacks(self) -> None:
        called = False

        def cb(fut: Future[Any]) -> None:
            nonlocal called
            called = True

        f: Future[Any] = Future()
        f.add_done_callback(cb)
        f.cancel()
        assert called

    def test_set_result_invokes_callbacks(self) -> None:
        called = False

        def cb(fut: Future[Any]) -> None:
            nonlocal called
            called = True

        f: Future[str] = Future()
        f.add_done_callback(cb)
        f.set_result('Anything')
        assert called

    def test_set_exception_invokes_callbacks(self) -> None:
        called = False

        def cb(fut: Future[Any]) -> None:
            nonlocal called
            called = True

        f: Future[Any] = Future()
        f.add_done_callback(cb)
        f.set_exception(Exception('Anything'))
        assert called

    def test_add_done_callback_invokes_callback(self) -> None:
        called = False

        def cb(fut: Future[Any]) -> None:
            nonlocal called
            called = True

        f: Future[str] = Future()
        f.set_result('Anything')
        f.add_done_callback(cb)
        assert called

    def test_set_result_on_done_future_without_exception(self) -> None:
        f: Future[None] = Future()
        f.set_result(None)
        self.assertTrue(f.done())
        self.assertFalse(f.cancelled())
        f.set_result(None)
        self.assertTrue(f.done())
        self.assertFalse(f.cancelled())

    def test_set_result_on_cancelled_future_without_exception(self) -> None:
        f: Future[None] = Future()
        f.cancel()
        self.assertTrue(f.cancelled())
        self.assertFalse(f.done())
        f.set_result(None)
        self.assertTrue(f.done())

    def test_set_exception_on_done_future_without_exception(self) -> None:
        f: Future[None] = Future()
        f.set_result(None)
        self.assertIsNone(f.exception())
        f.set_exception(Exception())
        f.set_result(None)
        self.assertIsNotNone(f.exception())

    def test_set_exception_on_cancelled_future_without_exception(self) -> None:
        f: Future[Any] = Future()
        f.cancel()
        self.assertTrue(f.cancelled())
        self.assertIsNone(f.exception())
        f.set_exception(Exception())
        self.assertIsNotNone(f.exception())


if __name__ == '__main__':
    unittest.main()
