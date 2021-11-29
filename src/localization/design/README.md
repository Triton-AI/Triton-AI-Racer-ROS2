# Localization Design

## Transforms

Per [REP105](https://www.ros.org/reps/rep-0105.html) we implement a full transform tree of `earth -> map -> odom -> base_link`.

### As of December 2021

`earth -> map` transformation is omitted as we currently target indoor competitions.

## Nodes

- `odometry_localizer_node` publishes an `Odometry` that corresponds to `odom -> base_link` transform.
- A map localizer, such as a `particle_fitler_py_node`, publishes a `PoseWithCovarianceStamped` that corresponds to `map -> base_link` transform.
- A `transform_manager_node` that takes the odometry and pose published by the aforementioned nodes and publishes the transform tree.