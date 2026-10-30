# cu-moveit 0.2.0

This pre-1.0 release adds fixed-capacity trajectory generation and following,
Cartesian path timing, and versioned planning-scene snapshots for joint-space
RRT* and collision checks.

- RRT* requests now carry a planning scene and collision exclusions. Results
  include the scene revision and exclusion snapshot; consumers must compare both
  against current inputs before accepting a plan.
- The collision-check task accepts a scene and exclusions alongside joint positions.
- Cartesian S-curve timing checks joint jerk on quintic reconstruction; execute
  that profile with quintic interpolation. Other Cartesian profiles use cubic
  interpolation for their velocity and acceleration checks.
- Collision checks are discrete. Execution collision monitoring, pose-goal
  planning, and Cartesian planning against the shared scene remain future work.
- Payload and task interface changes break compatibility with earlier clients
  and may require migration of recorded data.

Crates.io publishing remains disabled; this release is distributed as a Git tag.
