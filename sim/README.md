# sim/

Sensor-simulation layer: kinematic puppet + noise models + rclpy publisher.

Scaffolding-quality Python by design (ADR 0002): the artifact is the C++
fusion node; this layer exists to generate honest sensor streams.

```bash
# from repo root, in pixi env:
pixi run python -m otolith_sim.sim_node        # with PYTHONPATH=sim
PYTHONPATH=sim pixi run python -m otolith_sim.sim_node
```

Topics: `/otolith/imu`, `/otolith/joint_states`, `/otolith/foot_contacts`,
`/otolith/ground_truth`. Open Foxglove (Windows) and add panels for them.

The puppet is *consistent by construction*: footholds fixed in the world,
base advancing in lockstep, closed-form leg IK — contact flags and leg
odometry are true by construction, so estimator error is attributable to
sensor noise, not puppet artifacts.
