{
  "summary": {
    "task_name": "lift_box",
    "total_demonstrations": 6,
    "original_demonstrations": 1,
    "augmented_demonstrations": 5,
    "num_objects_per_demo": 2,
    "num_action_steps_per_demo": 8,
    "timestamp": "yy-MM-DD-hh-mm-ss",
    "has_real_actions": true
  },
  "demonstrations": [
    {
      "obs": [
        [x, y, z], [x, y, z], ...  // Original point cloud points
      ],
      "actions": [
        {
          "left_arm": [x, y, z, rx, ry, rz],
          "right_arm": [x, y, z, rx, ry, rz],
          "step": 0,
          "task_phase": "approach_box"
        },
        {
          "left_arm": [x, y, z, rx, ry, rz],
          "right_arm": [x, y, z, rx, ry, rz],
          "step": 1,
          "task_phase": "grasp_box"
        },
        // More action steps...
      ],
      "demo_idx": 0,
      "is_augmented": false,
      "task_name": "lift_box",
      "num_objects": 2,
      "num_action_steps": 8
    },
    {
      "obs": [
        [x, y, z], [x, y, z], ...  // Augmented point cloud points
      ],
      "actions": [
        {
          "left_arm": [x, y, z, rx, ry, rz],
          "right_arm": [x, y, z, rx, ry, rz],
          "step": 0,
          "task_phase": "approach_box"
        },
        // More action steps...
      ],
      "demo_idx": 1,
      "is_augmented": true,
      "task_name": "lift_box",
      "transformations": [
        {
          "translation": [0.05, 0.02, 0.0],
          "rotation_angle": 15.0,
          "rotation_axis": "z",
          "scale_factor": 1.0
        },
        {
          "translation": [-0.03, 0.04, 0.0],
          "rotation_angle": -10.0,
          "rotation_axis": "z",
          "scale_factor": 1.0
        }
      ],
      "num_objects": 2,
      "num_action_steps": 8
    }
    // More demonstrations...
  ]
}

Key Components:
1. Summary Section: Contains metadata about the dataset

  - task_name: Name of the task (e.g., "lift_box")

  - total_demonstrations: Total number of demonstrations (original + augmented)

  - original_demonstrations: Number of original demonstrations (typically 1)

  - augmented_demonstrations: Number of augmented demonstrations

  - num_objects_per_demo: Number of objects in each demonstration

  - num_action_steps_per_demo: Number of action steps in each demonstration

2. Demonstrations Array: Contains all demonstrations (original + augmented)

  - Each demonstration has:

    - obs: Array of 3D points representing the scene point cloud

    - actions: Array of action steps, each with:

    - left_arm: 6-DOF pose [x, y, z, rx, ry, rz] for left robot

    - right_arm: 6-DOF pose [x, y, z, rx, ry, rz] for right robot

    - step: Step index

    - task_phase: Phase of the task (e.g., "approach_box", "grasp_box", etc.)

    - demo_idx: Unique index for the demonstration (0 for original)

    - is_augmented: Boolean flag (false for original, true for augmented)

    - transformations: Array of transformations applied to objects (for augmented demos)
