# Credits

## 3D model — `assets/c172_full.glb`

- **Title:** Cessna172 (interior + exterior)
- **Author:** KOG_THORNS (https://sketchfab.com/ioai25312)
- **Source:** https://sketchfab.com/3d-models/cessna172-d1b15841c29c43d0862667300bad55a4
- **License:** [CC BY 4.0](http://creativecommons.org/licenses/by/4.0/)
- **Modifications:** None. The model file is used as-is. It ships no
  skeleton/animation, so at runtime each separate control-surface node is rotated
  about its own measured leading-edge line (`Transform::rotate_around`) — no
  re-export or geometry edit.

## 3D model — `assets/f22.glb`

- **Title:** F22 Raptor FREE
- **Author:** NLM (https://sketchfab.com/NLM-Group)
- **Source:** https://sketchfab.com/3d-models/f22-raptor-free-2a64abf0866a405c865466c7642ca689
- **License:** [CC BY 4.0](http://creativecommons.org/licenses/by/4.0/)
- **Modifications:** None. The model file is used as-is; its control-surface
  pivot nodes are driven at runtime by the simulation (no re-export or geometry edit).

## 3D model — `assets/f22_cockpit.glb`

- **Title:** F-22 Raptor - Fighter Jet - Free
- **Author:** bohmerang (https://sketchfab.com/bohmerang)
- **Source:** https://sketchfab.com/3d-models/f-22-raptor-fighter-jet-free-508de5c48845456bb033fb267ebe1d1e
- **License:** [CC BY-NC-SA 4.0](http://creativecommons.org/licenses/by-nc-sa/4.0/)
- **Modifications:** None. Shown only in the F-22's pilot-eye cockpit view; its
  separate cockpit, canopy and HUD meshes supply the interior. The airframe is
  used as-is (single welded mesh, no animation).

## 3D model — `assets/b737.glb`

- **Title:** Boeing737
- **Author:** manilov.ap (https://sketchfab.com/manilov.ap)
- **Source:** https://sketchfab.com/3d-models/boeing737-259de41be0c7410e86b7e79be33d6b3a
- **License:** [CC BY 4.0](http://creativecommons.org/licenses/by/4.0/)
- **Modifications:** None. The model file is used as-is; its wing, stabilizer
  and fin have no separate control-surface geometry, so nothing is animated on
  the mesh.

The same attribution for each model is carried inside the file itself
(`asset.extras` in the glTF JSON).

## Shader — `assets/shaders/island_water.wesl`

- **Title:** Seascape
- **Author:** Alexander Alekseev aka TDM (https://www.shadertoy.com/user/TDM)
- **Source:** https://www.shadertoy.com/view/Ms2SD1
- **License:** [CC BY-NC-SA 3.0](https://creativecommons.org/licenses/by-nc-sa/3.0/)
- **Modifications:** Ported from a raymarched full-screen scene to a flat,
  world-scale water plane: the wave height field, choppy octaves, Fresnel
  colour model and normalized specular lobe are reused, with distance-based
  octave LOD, a world-scale-safe value-noise hash, the project's own sky
  gradient/weather and aerial haze. Attribution required; non-commercial.

## 3D model — `assets/ah64d_cockpit.glb`

- **Title:** AH-64D Apache pilot-station cockpit (PLT rear seat)
- **Author:** Procedurally generated for this project by
  `tools/ah64_cockpit.py`.
- **Geometry:** Original work. No third-party mesh or texture data is included;
  the panels, canopy frame, MPD bezels, screen symbology and HOCAS are built
  from primitives in Blender.
- **Layout reference:** Public DCS AH-64D cockpit/controls guide
  (gamepressure) and publicly available AH-64 cockpit photographs. The model
  is an approximate game interior, not a Boeing engineering drawing.
- **License:** Same as the rest of this project's original code/assets.

## 3D model — `assets/ah64d.glb`

- **Title:** Boeing AH-64D Apache Combat Helicopter
- **Author:** Muhamad Mirza Arrafi (https://sketchfab.com/nazidefenseforceofficial)
- **Source:** https://sketchfab.com/3d-models/boeing-ah-64d-apache-combat-helicopter-c3b58008c46b45048fdd7dd283a3c8c8
- **License:** [CC BY 4.0](http://creativecommons.org/licenses/by/4.0/)
- **Modifications:** Imported and re-exported by `tools/prepare_apache.py`. The
  original material batches are kept; the main and tail rotors are split out of
  the airframe batch into `AH64_MainRotor` / `AH64_TailRotor` nodes with hub
  pivots so the simulator can spin them. Node names are made readable and the
  original `asset.extras` attribution is preserved here.
- **Use:** exterior visual for the AH-64D entry; the FDM is currently the AH-1S
  model as a stand-in.
