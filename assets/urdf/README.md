# Airframe assets (URDF / SDF / meshes)

Vehicle description files, one folder per airframe family. These are shared by
the Gazebo bringup in `ros2_ws/src/sim_gazebo` and by RViz visualization.

```
assets/urdf/
├── multirotor/   quad / hex / octo
├── fixed_wing/   twin-motor wing, single-motor wing
├── monocopter/
└── tvc/          thrust-vector-controlled airframes
```

## Conventions

- **Frame**: `base_link` at the vehicle center of mass, **x** forward, **y** left,
  **z** up (ROS REP-103).
- **Naming**: `<airframe>_<variant>.urdf`, e.g. `fixed_wing/twin_wing.urdf`.
- **Meshes**: keep alongside the description in a `meshes/` subfolder and
  reference them with `package://` or a relative path, not an absolute one.
- **Gazebo variants**: if a model needs Gazebo-specific plugin tags, keep the
  clean description and the Gazebo variant as separate files
  (`twin_wing.urdf` / `twin_wing_gazebo.urdf`), the way
  `ros2_ws/src/sim_gazebo/models/lr_drone_urdf` already does.

## Existing models

The quadcopter models currently live under
`ros2_ws/src/sim_gazebo/models/` (`lr_drone_urdf`, `lr_drone_controlled`,
`x3_velocity_control`). They stay there for now; consolidating them into this
folder is part of the airframe-abstraction work.
