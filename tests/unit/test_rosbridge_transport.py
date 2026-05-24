"""
Offline unit tests for ROSBridgeTransport.

Focus: disconnect must use roslibpy's ``close()`` (which leaves the process-wide
Twisted reactor running) and never ``terminate()`` (which stops the reactor for
good — ReactorNotRestartable — and breaks every later connect in a long-lived
process such as the web server).
"""

import pytest

from walkie_sdk.core.transports.rosbridge.transport import ROSBridgeTransport


class FakeRos:
    def __init__(self):
        self.closed = False
        self.terminated = False
        self.is_connected = True

    def close(self):
        self.closed = True
        self.is_connected = False

    def terminate(self):
        self.terminated = True


def test_disconnect_closes_without_terminating_reactor():
    t = ROSBridgeTransport(host="10.0.0.1", port=9090)
    fake = FakeRos()
    t._ros = fake

    t.disconnect()

    assert fake.closed is True, "disconnect should call close()"
    assert fake.terminated is False, "disconnect must NOT call terminate() (kills reactor)"
    assert t._ros is None


def test_disconnect_is_idempotent():
    t = ROSBridgeTransport(host="10.0.0.1", port=9090)
    # No connection established: should be a no-op, not an error.
    t.disconnect()
    t.disconnect()
    assert t._ros is None


def test_is_connected_false_when_no_ros():
    t = ROSBridgeTransport(host="10.0.0.1", port=9090)
    assert t.is_connected is False


def test_connect_unreachable_fails_fast_without_starting_reactor():
    # Port 1 refuses immediately. The TCP pre-check must raise ConnectionError
    # and never create a roslibpy.Ros — so a bad address can't start (and then
    # poison) the process-wide Twisted reactor for later connects.
    t = ROSBridgeTransport(host="127.0.0.1", port=1, timeout=2)
    with pytest.raises(ConnectionError):
        t.connect()
    assert t._ros is None
