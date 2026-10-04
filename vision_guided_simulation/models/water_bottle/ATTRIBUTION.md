# Water Bottle model

Source: [Water Bottle, version 3, uploaded by iche033](https://fuel.gazebosim.org/1.0/iche033/models/Water%20Bottle).

License: [Creative Commons Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0/).

Changes: converted GLB to COLLADA, rotated to Z-up, scaled to 0.22 m height,
centred at the bottom, reduced the colour texture to 512 pixels, and made the
colour texture opaque for compatibility with the VM renderer.

The original GLB is downloaded into the workspace's `work/` directory. The
conversion is reproducible with `scripts/prepare_bottle_model.py` using trimesh
and pycollada. These converted model files retain the source model's CC BY 4.0
license and are excluded from the repository's MIT license.
