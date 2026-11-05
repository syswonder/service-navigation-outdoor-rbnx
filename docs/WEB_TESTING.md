# Current Scene validation

Use the robot deployment repository's `outdoor/` profile and the unified Scene
page. Global plans come from the mapping service; the navigation service has
no independent web UI or campus map. Legacy web demo scripts were retired.

Run the package tests with generated bindings, then `tests/stairs_smoke.py` in
a ROS-compatible environment. It uses localhost domain 188 and virtual outputs.
Robot deployment tests `test_scene_controls.py` and `scene_ui_fixture.py` cover
the actual Scene app with a newly constructed synthetic recorded road and real
generated navigation gRPC. They read no legacy map assets.

Physical movement and actual stairs require operator field acceptance.
