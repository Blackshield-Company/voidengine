# templar3d assets: sources and licences

Every file here is CC0 1.0 (public domain dedication). Nothing was purchased. Attribution is not
required; it is recorded anyway. sha256 values are of the files in this folder.

| file | what | author | source | licence | sha256 |
|---|---|---|---|---|---|
| `quaternius_king.glb` | King from Quaternius' Ultimate Modular Men pack: realistic proportions, skinned (62 joints, 24 clips incl. Idle/Walk/Run), flat material colours; the demo's character since 3D-2 | Quaternius (https://quaternius.com/) | mirrored at https://github.com/Papyszoo/CC0-Public-Domain-Models/tree/main/packs/quaternius-ultimatemodularcharacters/models/king (downloaded 2026-11-23, unmodified) | CC0-1.0 | c40d2a51983565a5614d7f280d1e591c73ff4a38d93a2fbf3c03f26186b7bccd |
| `kaykit_knight.glb` | Knight from KayKit: Adventurers Character Pack (skinned, 41 joints, 76 clips, palette texture, sword/shield/helmet/cape props on joints; the 3D-1 character, no longer loaded) | Kay Lousberg (https://kaylousberg.com/) | https://kaylousberg.itch.io/kaykit-adventurers, mirrored at https://github.com/Papyszoo/CC0-Public-Domain-Models/blob/main/packs/kaykit-character-pack-adventures/models/knight/knight.glb (downloaded 2026-11-23, unmodified) | CC0-1.0 | 60428e3abc09ba83e595d256e3af8c5c976b46cdae599f0802fc82b4a3445168 |
| `ground_grass_albedo.png` | Poly Haven `forrest_ground_01` albedo | Rob Tuytel | https://polyhaven.com/a/forrest_ground_01 | CC0-1.0 | d93e0683e82a1353945c69a673f3b04241f827bc87e27bba1afd0a96af66cc74 |
| `ground_grass_normal.png` | same, OpenGL normal | Rob Tuytel | same | CC0-1.0 | 7e928ff3621aa6e773d06cbdab02a7558abe9b752ff3c2fc6a0054a5977375b2 |
| `ground_grass_orm.png` | same, AO (cavity from displacement) / roughness / metallic 0 | Rob Tuytel | same | CC0-1.0 | ac9b4aed9acd9756246e2b7b64be14b4dfe44609d397db7d9e27bc061cff9b85 |
| `ground_dirt_albedo.png` | Poly Haven `rocky_trail` albedo | Amal Kumar | https://polyhaven.com/a/rocky_trail | CC0-1.0 | d568b460b82e635bd51f1fd4b5c6b722b751c40a9c7459c9d13109c95f43cbd8 |
| `ground_dirt_normal.png` | same, OpenGL normal | Amal Kumar | same | CC0-1.0 | 495dffe2f9a806c41bba6c11e4a0f058961474729ae1d9dfba52e0bb575a733a |
| `ground_dirt_orm.png` | same, AO / roughness / metallic 0 | Amal Kumar | same | CC0-1.0 | a8db1d6352cc53e0804e0c4c53588095bc1f7efb74c4026fea6a34a3d3e4419b |

The ground maps are derived, with `tools/pack_ground.py`, from Templar Valley's packed terrain textures
(`assets/terrain/0_grass_*.png` and `3_dirt_*.png` on branch `m1/terrain-pass` at `1356930`, whose own
`SOURCES.md` records the Poly Haven downloads). Inputs used, sha256:

- `0_grass_albedo_height.png` 8226b09aeefae34729a1ebabd426f3bde6c1e85b779ed002db039e5846cca3c2
- `0_grass_normal_rough.png` 7603c00863aab162246aea3e5a1c7d11890ba47b8d69b7b2b3ab4eb2a054f6cc
- `3_dirt_albedo_height.png` 00001974bb330d10f518720055cdc3bb4f080eb0ac81eed30c00104dbac2c021
- `3_dirt_normal_rough.png` 85492e49c88c9462e4f39b3b9cf5c5a6c7c65dc48f13872e8e0c78508b1ea2f0

Regenerate: `python3 examples/templar3d/tools/pack_ground.py <templar>/assets/terrain examples/templar3d/assets 0:grass 3:dirt`
