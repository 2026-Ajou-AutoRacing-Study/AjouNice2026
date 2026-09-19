# Experimental DetectionArea input correction

Both original scenes inspect predicted objects after point-cloud detection. The
normal manager declares this input, but the experimental manager omitted it. Full
BVP collects only declared optional inputs, silently disabling the experimental
object fallback when this is the sole object-consuming plugin. Declare objects in
the experimental manager as in the normal manager; keep all scene calculations,
target-filter parameters and point-cloud requirements unchanged. A missing object
message defers planning; a received empty message is valid. Existing retained host
regression already requires this input. This is an explicit upstream defect fix.
