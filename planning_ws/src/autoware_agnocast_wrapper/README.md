# autoware_agnocast_wrapper ROS1 disabled profile

This is a deliberately narrow P2 candidate for `ENABLE_AGNOCAST=0` (or an
unset variable).  It exports the ROS1 `message_filters` aliases corresponding
to the upstream non-Agnocast branch.  The aliases preserve ROS1 topic, message
type, and callback behavior because they are the native types, not a shim.

`ENABLE_AGNOCAST=1` (and every nonzero value) fails during CMake configure.
There is no launch file, component registration, `LD_PRELOAD`, `agnocastlib`,
`agnocast_components`, IPC pointer, or zero-copy claim in this package.  A
consumer needing an executable/nodelet decision must own that decision.

See `PORT_MAP.md` for source-to-candidate boundaries and the sibling P2
validation view for build, runtime, negative, and installed-consumer evidence.

