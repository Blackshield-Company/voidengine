# templar_valley assets: sources and licences

Every third-party file this demo uses is CC0 1.0 (public domain dedication). Nothing was purchased and nothing
from Bethesda or any other non-CC0 source is used. Attribution is not required; it is recorded anyway.

The binary assets are **not committed**. `tools/fetch_assets.py` downloads them into `assets/fetched/` and checks
each against `assets.lock` (sha256, listed below too); `tools/pack_ground.py` then writes the terrain layers into
`assets/generated/`. Both folders are git-ignored.

```
python3 examples/templar_valley/tools/fetch_assets.py
python3 examples/templar_valley/tools/pack_ground.py
make templar_valley && ./templar_valley
```

## Models (Quaternius, CC0)

Author: Quaternius (https://quaternius.com/). Each pack's page states CC0 1.0. Downloaded 2026-11-23 from the
CC0 mirror https://github.com/Papyszoo/CC0-Public-Domain-Models (per-pack `pack.json` records licence CC0),
unmodified.

| file | what | source | licence | sha256 |
|---|---|---|---|---|
| `king.glb` | King (Ultimate Modular Men pack): realistic-proportion human, 62 joints, Idle/Walk/Run/sword clips; the rider | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-ultimatemodularcharacters/models/king | CC0-1.0 | c40d2a51983565a5614d7f280d1e591c73ff4a38d93a2fbf3c03f26186b7bccd |
| `horse.glb` | Horse (Ultimate Animated Animals): 50 joints, Walk/Gallop/Idle clips | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-ultimateanimatedanimals/models/horse | CC0-1.0 | c28783e739a97411fe597d0de8ef1d1260c622f39aea4f0912132d83a9d5aee5 |
| `pine_1.glb` | Pine 1 (Stylized Nature MegaKit): forest-floor trees | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-stylized-nature-megakit/models/pine_1 | CC0-1.0 | 88c5a8a1d1a3a2d5e6af38371b2339f171af59bc3a26e81366a41257e56d808b |
| `pine_2.glb` | Pine 2 (Stylized Nature MegaKit) | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-stylized-nature-megakit/models/pine_2 | CC0-1.0 | 570c34e6934d42ddddfc6e1b6a2cff263f2d468cd7351a3e0f96ade15061eae1 |
| `common_tree_1.glb` | Common Tree 1 (Stylized Nature MegaKit): meadow trees | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-stylized-nature-megakit/models/common_tree_1 | CC0-1.0 | beeef68853237a59aca0a702f0f16e6e70f9191ac20bc30f9b3c2583f79b54cb |
| `common_tree_2.glb` | Common Tree 2 (Stylized Nature MegaKit) | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-stylized-nature-megakit/models/common_tree_2 | CC0-1.0 | b53959ba126f622580c0a1bec1e3e165cdb6622e4b0d39eb5d04e0242767ca4f |
| `grass_common_short.glb` | Grass clump (Stylized Nature MegaKit), base colour tinted in code | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-stylized-nature-megakit/models/grass_common_short | CC0-1.0 | e189fe9503f139c9ce9182f46db574dc6e5582895d16bc0f915eef5c74c65f4c |
| `grass_wispy_short.glb` | Wispy grass clump (Stylized Nature MegaKit) | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-stylized-nature-megakit/models/grass_wispy_short | CC0-1.0 | e1cc83a4deef616a1b95626a0d282160a79d70bb8af48fc0354e03a04a8b4eb3 |
| `rock_medium_1.glb` | Rock (Stylized Nature MegaKit): clutter | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-stylized-nature-megakit/models/rock_medium_1 | CC0-1.0 | cc0780f873af90972916a0a0eb15546f2053ce73be1641cefab07f2d01b8a0ef |
| `house_1.glb` | House 1 (Medieval Village pack) | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-medievalvillage/models/house_1 | CC0-1.0 | 961305d83022fd76ba8c6f173743e7e92c6a84b86130b250ad35049a5d436f5b |
| `house_2.glb` | House 2 (Medieval Village pack) | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-medievalvillage/models/house_2 | CC0-1.0 | 576a28fe785fcafeb1544ecf996df2cad66b6cacebec4776f32185fb5dde3e78 |
| `house_3.glb` | House 3 (Medieval Village pack) | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-medievalvillage/models/house_3 | CC0-1.0 | f5d51c90bc3166133fc541430992d108875014d1a65ee61abf0c775ecf72fbe0 |
| `crate.glb` | Crate (Medieval Village pack): clutter | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-medievalvillage/models/crate | CC0-1.0 | 1752344e4b558ab1c0667947b44226005d8cc4e20845bf2c86aafbd768ddf69f |
| `barrel.glb` | Barrel (Medieval Village pack): clutter | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-medievalvillage/models/barrel | CC0-1.0 | c740f5f085dfb7e4a1712ec6106f7a28915c5b5d093f1085931e7ffead451223 |
| `hay.glb` | Hay bale (Medieval Village pack): clutter | https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-medievalvillage/models/hay | CC0-1.0 | 7db43137767e8ebaffb5d947266b24beff8e98e1e72aaf3e1bd7162516eb5d00 |

## Ground textures (Poly Haven, CC0)

Templar Valley's four ground slots (same Poly Haven assets as Templar's `assets/terrain/SOURCES.md`), fetched at 1k
JPG from https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/. Licence https://polyhaven.com/license (CC0 1.0).

| file | slot | author | page | sha256 |
|---|---|---|---|---|
| `forrest_ground_01_diff_1k.jpg` | slot 0 grass | Rob Tuytel | https://polyhaven.com/a/forrest_ground_01 | 3dd6875cb3908e022a3c45ebbffa5e84c670ff2691fbbb6dc9ea4bff88523800 |
| `forrest_ground_01_nor_gl_1k.jpg` | slot 0 grass | Rob Tuytel | https://polyhaven.com/a/forrest_ground_01 | 32528a7cdee962cc0b248ee4023a74d0df175737ea90e1eb425122351e0bdab4 |
| `forrest_ground_01_rough_1k.jpg` | slot 0 grass | Rob Tuytel | https://polyhaven.com/a/forrest_ground_01 | 30d8b56a03d7b12da16f58011b675662a40e58e1cdc768119a5f03968140058c |
| `forrest_ground_01_disp_1k.jpg` | slot 0 grass | Rob Tuytel | https://polyhaven.com/a/forrest_ground_01 | a1ce10a2426d20d7fec2e48efa255638055922c809ff3ea03067257c0a594953 |
| `rock_face_03_diff_1k.jpg` | slot 1 rock | Dario Barresi, Rico Cilliers | https://polyhaven.com/a/rock_face_03 | ea48e0e47ad42c8bd312476178b1248ec66f0eb19156a26a1c222f2a40caaf2d |
| `rock_face_03_nor_gl_1k.jpg` | slot 1 rock | Dario Barresi, Rico Cilliers | https://polyhaven.com/a/rock_face_03 | c6423169583f31715cf0636954bad524e806848a5f2861fcea6794275f726ff0 |
| `rock_face_03_rough_1k.jpg` | slot 1 rock | Dario Barresi, Rico Cilliers | https://polyhaven.com/a/rock_face_03 | 9e3d9a2fd75244892e7acb6bab2eaed1279299510a7970672c0ee27ff1c19fea |
| `rock_face_03_disp_1k.jpg` | slot 1 rock | Dario Barresi, Rico Cilliers | https://polyhaven.com/a/rock_face_03 | fb053bfd46cef976da9cfee67f045367469cddd69469dcacd67d3fca4bd41545 |
| `sparse_grass_diff_1k.jpg` | slot 2 forest floor | Amal Kumar | https://polyhaven.com/a/sparse_grass | ae94f2b34597b9108eefd88217f55eccaec6d6b382e858a478ee92df90e66617 |
| `sparse_grass_nor_gl_1k.jpg` | slot 2 forest floor | Amal Kumar | https://polyhaven.com/a/sparse_grass | 1c840a24807346ddaaf94a5057aee4078fcec3a4cd84bfe45da7d3b480ef460b |
| `sparse_grass_rough_1k.jpg` | slot 2 forest floor | Amal Kumar | https://polyhaven.com/a/sparse_grass | 9ebe6d03a9551c17d1802835c92c1442acb8f5d38d4874ee8c37770f11c4bed1 |
| `sparse_grass_disp_1k.jpg` | slot 2 forest floor | Amal Kumar | https://polyhaven.com/a/sparse_grass | b38a605af66737ebe078ef5fdffe79e59f17a5ee4ed9dd187caaef67e4f8ac2a |
| `rocky_trail_diff_1k.jpg` | slot 3 dirt | Amal Kumar | https://polyhaven.com/a/rocky_trail | 7f20d83cd4a67f9c38abf72f39f5a3749a962e90bdc156998be027b1227a5bd6 |
| `rocky_trail_nor_gl_1k.jpg` | slot 3 dirt | Amal Kumar | https://polyhaven.com/a/rocky_trail | 087b8912c8cf40a528965c4282a5b3e4be011da5790273ffcdb5bd2697bde7dd |
| `rocky_trail_rough_1k.jpg` | slot 3 dirt | Amal Kumar | https://polyhaven.com/a/rocky_trail | aa17fcfe3be8fa4d79316d1c7e7a3e32e8522849fb4811e7009aec7762275caa |
| `rocky_trail_disp_1k.jpg` | slot 3 dirt | Amal Kumar | https://polyhaven.com/a/rocky_trail | 0509ab95fe3b38c62711bdfa481ce9d7becb8ad8e25fdf2ff6d8ffc2a94fb744 |

## Committed data

- `assets/valley_props.csv` (sha256 2f96d0c6eee3173c0af3b1221fe97923961039d8e64e3f8dadf2b883d0416aef): Templar Valley's prop layout (buildings, trees, crates:
  kind, position, size) exactly as the Godot build generates it (`spike/tools/gen_cells.gd` on Templar branch
  `m1/terrain-pass` at `1356930`; per-cell RandomNumberGenerator seeded from `hash(Vector2i)`), baked with Godot 4.7
  by `tools/bake_valley.gd`. Generated data, no third-party content. Regenerate in an empty project that holds only
  the script: `godot --headless --path <dir> -s bake_valley.gd -- valley_props.csv`.
- The terrain height and ground-splat rules are ported in code (`valley.odin`, with the engine's FastNoiseLite port
  `src/core/noise_fnl.odin`, MIT, matching Godot's noise bit-for-bit in the tests).
