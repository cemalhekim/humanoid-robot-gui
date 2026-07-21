import unittest

import server


class TrackingGatingTests(unittest.TestCase):
    def make_store(self):
        return server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_start_requires_risk_ack(self):
        store = self.make_store()
        status, response = store.request_track_start({})
        self.assertEqual(status, 403)
        self.assertFalse(response["ok"])

    def test_start_refused_when_feature_disabled(self):
        store = self.make_store()
        old = server.TRACKING_ENABLED
        server.TRACKING_ENABLED = False
        try:
            status, response = store.request_track_start(
                {"armed": True, "i_understand_risk": True}
            )
        finally:
            server.TRACKING_ENABLED = old
        self.assertEqual(status, 409)

    def test_start_without_dds_returns_503(self):
        store = self.make_store()
        old = server.TRACKING_ENABLED
        server.TRACKING_ENABLED = True
        try:
            status, response = store.request_track_start(
                {"armed": True, "i_understand_risk": True}
            )
        finally:
            server.TRACKING_ENABLED = old
        self.assertEqual(status, 503)  # no wrist_publisher offline

    def test_stop_is_idempotent(self):
        store = self.make_store()
        status, response = store.request_track_stop()
        self.assertEqual(status, 200)
        self.assertTrue(response["ok"])

    def test_snapshot_shape(self):
        store = self.make_store()
        snap = store.track_snapshot()
        for key in ("enabled", "active", "phase", "message", "updated_at"):
            self.assertIn(key, snap)


class TrackingRouteTests(unittest.TestCase):
    def test_routes_are_dispatched(self):
        src = open("server.py").read()
        self.assertIn('"/api/track/start"', src)
        self.assertIn('"/api/track/stop"', src)
        self.assertIn('"/api/track/status"', src)


if __name__ == "__main__":
    unittest.main()
