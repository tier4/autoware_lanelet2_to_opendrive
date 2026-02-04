# Stop Line Position Discrepancies

## Issue

Stop line positions may not be accurately preserved in the conversion.

## Cause

CARLA simulator (the primary target platform for this tool) uses **TriggerVolume-based collision detection** for traffic signals. This means:

- Traffic signals in CARLA use invisible 3D trigger volumes to detect vehicles
- Stop line positions are automatically determined by the trigger volume placement
- The automatic positioning can differ from explicit stop line positions defined in Lanelet2 maps

## Impact

- Stop lines in the resulting OpenDRIVE map may be **shifted** from their original Lanelet2 positions
- The shift amount depends on CARLA's trigger volume configuration
- This is a CARLA architectural limitation, not a converter bug

## Workaround

If precise stop line positioning is critical for your use case:

1. Manually adjust trigger volumes in CARLA after importing the map
2. Use post-processing scripts to modify the OpenDRIVE file
3. Consider using a different simulator that supports explicit stop line positioning

---

[← Back to Limitations Overview](index.md)
