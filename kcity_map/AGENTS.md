# Map context router

Read `README.md` first. Runtime consumers select the map explicitly; source defaults for planning and
perception reference `lanelet_map/lanelet2_map_v99.osm`. Verify installed/runtime selection separately.
Keep map/projector/speed-limit assets and planning route semantics unchanged during
cleanup. Version builders, probes' old reports and historical geometry are not a
request to rebuild, upload, or replace a runtime map.
`legacy/` preserves construction history; see the repository archive policy before
trying an archived script whose relative paths assume the original source tree.
