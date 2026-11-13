# Original xiangruicheng vision code

Source files are retained byte-for-byte from the project snapshot
outdoor_sources_full_20261106.tar.gz. source_manifest.json records every origin
and original SHA-256. The current ranger depth_node and rtabmap_backend hashes were checked against this snapshot before integration.

- semantic_direct_navigation/depth_node.py: original registered RGB-D checks,
  calibrated geometric reference, RTAB-Map pairing and positive/drop hazard output.
- semantic_direct_navigation/rtabmap_backend.py: exact source-frame pairing and
  original >= 0.10 m positive hazard / measured drop policy.
- semantic_local_grid/projection_node.py: original registered semantic projection.
- semantic_direct_navigation/costmap.py: original known-ground-only semantic costs.
- sensor_sanity_checker/images.py: original stride/endian-safe Image decoder.

The service adapters only provide parameters/remaps and insert vision_filter
after the original detector. They do not modify any vendored detection algorithm.
No checkpoint weights, original device configuration or runtime data are bundled.
The original semantic_direct_navigation, semantic_local_grid and sensor_sanity_checker
package manifests declare Apache-2.0; see LICENSE.apache-2.0. Original source contents
and notices are retained. The service wrapper keeps its own top-level license.
