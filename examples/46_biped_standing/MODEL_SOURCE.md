# Unitree G1 model provenance

- Upstream: `google-deepmind/mujoco_menagerie/unitree_g1`
- Upstream commit: `71f066ad0be9cd271f7ed58c030243ef157af9f4`
- Imported: 2026-08-02
- License: BSD-3-Clause; see `LICENSE`
- Minimum MuJoCo version stated upstream: 2.3.4

The tutorial retains `g1.xml`, `scene.xml` (renamed to `model.xml`), the
upstream `README.md` (renamed to `MENAGERIE_README.md`), the model license,
and only the 35 STL files referenced by `g1.xml`. Unused dexterous-hand
models, meshes, and gallery images are intentionally omitted.

The tutorial's `main.cc`, CMake project, numerical acceptance criteria, and
viewer overlay are original teaching material and are not upstream Menagerie
files.
