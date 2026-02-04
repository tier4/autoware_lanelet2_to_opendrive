# Coordinate System Considerations

## MGRS Projection

The converter requires a valid **MGRS (Military Grid Reference System)** code for coordinate transformation.

## Limitations

- Maps spanning multiple MGRS grid zones are **not supported**
- Very large maps (> 100 km²) may have accumulated projection errors
- High-latitude regions (> 84°N or < 80°S) are outside MGRS coverage

## Workaround

For maps spanning multiple grid zones:

1. Split the map into separate grid zone sections
2. Convert each section independently
3. Manually merge the resulting OpenDRIVE files (advanced)

---

[← Back to Limitations Overview](index.md)
