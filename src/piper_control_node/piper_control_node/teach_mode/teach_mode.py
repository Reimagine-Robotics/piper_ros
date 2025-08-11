"""Teach mode via a custom gravity compensation mode.

The custom gravity compensation works by gathering a set of stable gravity
compensating torques using run_gather_data.py for various joint configurations.
These are then used to learn a torque residual function against a MUJOC model,
and this composite gravity model (MUJOCO + learned residual) is used to generate
on-the-fly gravity compensation torques for the Piper arm. These torques are
just enough to prevent the arm dropping from gravity, but any external force
applied (eg: a person moving the arm) will be enough to move the arm around
freely.
"""

import dataclasses
import json
from typing import Mapping, Sequence

import numpy as np
from piper_control import piper_control, piper_interface
from transformations import transformations as tr


@dataclasses.dataclass
class GravityTorqueSample:
  joint_configuration: np.ndarray
  joint_idx: int
  torque: float


def grav_comp_adjustment_fn(input_array, *params):
  """Enhanced gravity compensation using joint configuration features.

  Args:
    input_array: Flattened array containing [sim_torques(6), joint_angles(6), joint_idx(1)]
    *params: Variable number of parameters that scipy will optimize

  Returns:
    Adjusted gravity compensation torque for the specified joint
  """
  # Unpack the flattened input array
  sim_torques = input_array[:6]
  joint_angles = input_array[6:12]
  joint_idx = int(input_array[12])

  params = np.array(params)

  # Build feature vector using helper function
  features = _build_gravity_features(sim_torques, joint_angles, joint_idx)

  # Ensure we have the right number of parameters
  if len(params) != len(features):
    raise ValueError(f"Expected {len(features)} parameters, got {len(params)}")

  return np.dot(features, params)


def _build_gravity_features(sim_torques, joint_angles, joint_idx):
  """Build feature vector for enhanced gravity compensation.

  Separate function to make it easy to experiment by commenting out feature
  groups.

  Args:
    sim_torques: Array of all 6 MuJoCo predicted torques
    joint_angles: Array of 6 joint angles
    joint_idx: Index of joint we're predicting for (0-5)

  Returns:
    numpy array of features
  """
  features = []

  # Get the specific torque for this joint
  sim_torque = sim_torques[joint_idx]

  # Base features: powers of sim_torque for this joint
  features.extend(
      [
          1.0,  # bias
          sim_torque,
          sim_torque**2,
          sim_torque**3,
      ]
  )

  # # Joint angle features (trigonometric for periodic/smooth behavior)
  # # All joint angles can affect any joint's gravity compensation
  # for angle in joint_angles:
  #   features.extend(
  #       [
  #           np.sin(angle),
  #           np.cos(angle),
  #           np.sin(2 * angle),  # higher harmonics for finer detail
  #           np.cos(2 * angle),
  #       ]
  #   )

  # # Cross-terms: this joint's sim_torque modulated by all joint angles
  # for angle in joint_angles:
  #   features.extend(
  #       [
  #           sim_torque * np.sin(angle),
  #           sim_torque * np.cos(angle),
  #       ]
  #   )

  # # Inter-joint torque coupling: other joints' torques can affect this joint
  # for i, other_sim_torque in enumerate(sim_torques):
  #   if i != joint_idx:  # Don't include self, already covered in base features
  #     features.extend(
  #         [
  #             other_sim_torque,  # Linear coupling
  #             other_sim_torque
  #             * np.sin(
  #                 joint_angles[joint_idx]
  #             ),  # Modulated by this joint's angle
  #             other_sim_torque * np.cos(joint_angles[joint_idx]),
  #         ]
  #     )

  # # Joint coupling terms (some joints work together for gravity compensation)
  # features.extend(
  #     [
  #         np.sin(joint_angles[1])
  #         * np.cos(joint_angles[2]),  # shoulder-elbow coupling
  #         joint_angles[0] * joint_angles[1],  # base-shoulder coupling
  #         np.sin(joint_angles[1] + joint_angles[2]),  # combined arm pose
  #     ]
  # )

  return np.array(features)


class _SimGravityTorquePrediction:
  """Predict gravity comp torques for the arm in any joint configuration."""

  def __init__(
      self,
      model_path: str,
      grav_comp_params: Mapping[int, Sequence[float]],
      arm_orientation: str = "upright",
  ):
    # JIT import of mujoco to not force mujoco install for piper_ros users that
    # don't use teach mode functionality.
    import mujoco  # type: ignore pylint: disable=import-outside-toplevel

    self._model = mujoco.MjModel.from_xml_path(model_path)

    # Set gravity vector based on arm orientation using new system
    gravity = self._get_gravity_vector_for_orientation(arm_orientation)
    self._model.opt.gravity[:] = gravity
    print(f"Set gravity vector for {arm_orientation} orientation: {gravity}")

    self._data = mujoco.MjData(self._model)
    self._mj_forward_fn = mujoco.mj_forward

    self._grav_comp_params = grav_comp_params

    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    self._joint_ids = [self._model.joint(name).id for name in joint_names]

  @staticmethod
  def _get_gravity_vector_for_orientation(orientation: str) -> list[float]:
    """Get gravity vector for arm orientation using analytical transforms.

    Uses the ArmOrientation system and r2-transformations for proper analytical
    computation instead of hardcoded transformations.
    """

    # Get the orientation object and compute gravity analytically
    arm_orientation = piper_control.ArmOrientations.from_string(orientation)

    # Standard downward gravity in world frame
    world_gravity = np.array([0.0, 0.0, -9.81])

    # Rotate gravity into arm's coordinate frame using mounting quaternion
    gravity_vector = tr.quat_rotate(
        np.array(arm_orientation.mounting_quaternion), world_gravity
    )

    return np.asarray(gravity_vector).tolist()

  def predict(self, joint_angles: Sequence[float]) -> Sequence[float]:
    assert len(joint_angles) == len(self._joint_ids)

    for i, joint_id in enumerate(self._joint_ids):
      self._data.qpos[joint_id] = joint_angles[i]
      self._data.qvel[:] = 0.0

    # Propagate the changes through the simulation
    self._mj_forward_fn(self._model, self._data)
    result = [self._data.qfrc_bias[ji] for ji in self._joint_ids]

    # Get all sim torques as numpy array
    sim_torques = np.array(result)

    # Apply learned adjustments to each joint
    for i in range(len(result)):
      if i in self._grav_comp_params:
        params = self._grav_comp_params[i]

        # Create flattened input array: sim_torques(6) + joint_angles(6) + joint_idx(1)
        input_array = np.concatenate([sim_torques, np.array(joint_angles), [i]])
        result[i] = grav_comp_adjustment_fn(input_array, *params)

    assert len(result) == len(self._joint_ids)
    return result


def _load_sampled_gravity(
    filename: str,
) -> dict[int, Sequence[GravityTorqueSample]]:
  with open(filename, "r", encoding="utf-8") as f:
    serialised = json.load(f)

  result = {}
  for se in serialised:
    joint_idx = se["joint_idx"]

    if joint_idx not in result:
      result[joint_idx] = []

    result[joint_idx].append(
        GravityTorqueSample(
            joint_configuration=np.array(se["joints_cfg"]),
            joint_idx=joint_idx,
            torque=se["grav_comp_torque"],
        )
    )

  return result


def _compute_gravity_model(
    piper_model_path: str,
    grav_torques_file_path: str,
    arm_orientation: str = "upright",
) -> _SimGravityTorquePrediction:
  """Given sampled grav comp torques generated by run_gather_data.py, output a
  gravity model that uses both MUJOCO and a learned residual.
  """

  # JIT import of scipy to not force it on piper_ros users that don't use teach
  # mode functionality.
  from scipy import optimize  # type: ignore pylint: disable=import-outside-toplevel

  real_gravity = _load_sampled_gravity(grav_torques_file_path)
  sim_gravity = _SimGravityTorquePrediction(
      piper_model_path, {}, arm_orientation
  )

  per_joint_params = {}

  for joint_idx, samples in real_gravity.items():
    real_torques = []
    input_data_list = []

    for sample in samples:
      real_torques.append(sample.torque)

      desired_angles = list(sample.joint_configuration)
      model_torques = sim_gravity.predict(desired_angles)
      model_torque = model_torques[joint_idx]

      # Pack sim_torque and joint_angles as tuple for enhanced function
      input_data_list.append((model_torque, np.array(desired_angles)))

    # Convert to format expected by curve_fit
    real_torques = np.array(real_torques)

    # Calculate expected number of features by calling the helper function
    # Use first sample to determine feature count
    sample_sim_torque, sample_joint_angles = input_data_list[0]
    # Create dummy sim_torques array for feature building (we need all torques)
    dummy_sim_torques = np.array([sample_sim_torque] * len(sample_joint_angles))
    sample_features = _build_gravity_features(
        dummy_sim_torques, sample_joint_angles, joint_idx
    )
    n_total_features = len(sample_features)

    # Initial guess: mostly zeros except for the linear sim_torque term
    p0_enhanced = np.zeros(n_total_features)
    p0_enhanced[1] = 1.0  # sim_torque linear term (index 1 in feature vector)

    # Prepare input data for grav comp function - concatenate into flat arrays
    input_arrays = []
    for sample in samples:
      desired_angles = list(sample.joint_configuration)
      model_torques = sim_gravity.predict(desired_angles)
      sim_torques_array = np.array(model_torques)

      # Concatenate sim_torques(6) + joint_angles(6) + joint_idx(1) into single array
      input_array = np.concatenate(
          [sim_torques_array, np.array(desired_angles), [joint_idx]]
      )
      input_arrays.append(input_array)

    # Convert to 2D array for scipy
    x_data = np.array(input_arrays)
    breakpoint()
    opt_params = optimize.curve_fit(
        grav_comp_adjustment_fn,
        x_data,
        real_torques,
        p0=p0_enhanced,
        maxfev=2000,  # Increase max function evaluations for complex fit
    )[0]

    print(
        f"Joint {joint_idx}: Using gravity compensation model with "
        f"{len(opt_params)} parameters"
    )

    per_joint_params[joint_idx] = opt_params

  return _SimGravityTorquePrediction(
      piper_model_path, per_joint_params, arm_orientation
  )


class TeachController:
  """A controller that allows teach mode by commanding grav comp torques."""

  def __init__(
      self,
      robot: piper_interface.PiperInterface,
      controller: piper_control.MitJointPositionController,
      piper_model_path: str,
      grav_torques_file_path: str,
      arm_orientation: str = "upright",
  ):
    self._robot = robot
    self._controller = controller
    self._gravity_model = _compute_gravity_model(
        piper_model_path, grav_torques_file_path, arm_orientation
    )

  def step(self) -> None:
    cur_joints = self._robot.get_joint_positions()
    cur_vel = np.array(self._robot.get_joint_velocities())

    hover_torque = np.array(self._gravity_model.predict(cur_joints))

    # Stability torque acts to counteract joint movement. A dampener basically.
    stability_torque = -cur_vel * 1.0
    applied_torque = hover_torque + stability_torque

    self._controller.command_torques(list(applied_torque))
