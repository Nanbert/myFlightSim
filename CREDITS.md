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

## 3D model — `assets/mi-24_hind.glb`

- **Title:** Mi-24 Hind
- **Author:** Duane's Mind (https://sketchfab.com/duanesmind)
- **Source:** https://sketchfab.com/3d-models/mi-24-hind-004d68143e1a4df88e136dbc0a05f181
- **License:** [CC BY 4.0](http://creativecommons.org/licenses/by/4.0/)
- **Modifications:** None. Used as the visual for the AH-1S rotorcraft FDM; its
  main and tail rotor hubs are spun at runtime from the simulated rotor rpm.

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
