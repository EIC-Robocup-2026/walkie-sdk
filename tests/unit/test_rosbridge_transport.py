"""
Offline unit tests for ROSBridgeTransport.

Focus: disconnect must use roslibpy's ``close()`` (which leaves the process-wide
Twisted reactor running) and never ``terminate()`` (which stops the reactor for
good — ReactorNotRestartable — and breaks every later connect in a long-lived
process such as the web server).
"""

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
