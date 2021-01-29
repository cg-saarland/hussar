# CAD
To perform measurements with our AWR1243 radar sensor, we needed some form of construction that allows us to precisely control (or determine) the relative position and orientation of our measurement samples to the radar sensor.
This is required to match the digital scene for simulation exactly to the real world scene that was measured.

The designs contained in this directory have been modeled with Blender 2.90 and 3d-printed on an _Anycubic Mega i3_ and a _Prusa i3 MK3_.

| construction   | description |
|----------------|-------------|
| [`goniometer`](goniometer) | Our first attempt at building a measurement device. Was too wobbly and intrusive for measurements. |
| [`office`](office) | The second generation of our measurement device. This time less intrusive and with higher precision. |
| [`mobile-radar`](mobile-radar) | A device suited for measurements of large scenes. Does not position the sample itself, relies on photogrammetry to determine the position afterwards. |
