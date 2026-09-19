# NoStoppingArea required-input correction

Original normal and experimental managers returned empty RequiredSubscriptionInfo,
but both scenes consume predicted_objects; normal check_stuck_vehicle dereferences
that pointer. Full BVP only collects module-declared optional inputs. Loading this
module alone could therefore enter its algorithm without any object message.

This revision requires predicted_objects for both manager variants. It retains all
geometry, object filtering, RTC and stop decisions. A received empty object message
remains valid; a missing message must defer host output. Retained manager regression
already expects this declaration. This is an explicit upstream-contract defect fix,
not a claim that literal original declarations were semantically correct.
