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
from collections.abc import Mapping, Sequence

import numpy as np
from piper_control import piper_control, piper_interface
from transformations import transformations as tr


@dataclasses.dataclass
class GravityTorqueSample:
  joint_configuration: np.ndarray
  joint_idx: int
  torque: float


def grav_comp_adjustment_fn(model_inputs: np.ndarray, *params) -> np.ndarray:
  """Enhanced gravity compensation using joint configuration features.

  Args:
    model_inputs: 2D array of shape (n_samples, 13) containing flattened data
    *params: Variable number of parameters that scipy will optimize

  Returns:
    Array of adjusted gravity compensation torques for all samples
  """
  # scipy passes us the entire dataset at once for vectorized operation
  if model_inputs.ndim == 1:
    # Single sample case (when called from predict)
    model_inputs = model_inputs.reshape(1, -1)

  # Build feature matrix for all samples at once
  features_matrix = _build_model_features(model_inputs)

  # Ensure we have the right number of parameters
  params_array = np.array(params)
  if len(params_array) != features_matrix.shape[1]:
    raise ValueError(
        f"Expected {features_matrix.shape[1]} parameters, got "
        "{len(params_array)}",
    )

  # (n_samples, n_features) @ (n_features,) -> (n_samples,)
  results = features_matrix @ params_array

  return results if results.shape[0] > 1 else results[0]


def _build_model_features(
    model_inputs: np.ndarray,
) -> np.ndarray:
  """Build feature matrix for enhanced gravity compensation (vectorized).

  Args:
    model_inputs: Array of shape (n_samples, 13) containing sim_torques,
      joint_angles, and joint_idx for the joint being predicted.
      torques for each sample.

  Returns:
    numpy array of shape (n_samples, n_features)
  """
  # Unpack all samples at once
  sim_torques = model_inputs[:, :6]  # (n, 6)
  joint_angles = model_inputs[:, 6:12]  # (n, 6)
  joint_idx = model_inputs[:, 12].astype(int)  # (n,)

  features_list = []
  n_samples = sim_torques.shape[0]  # scalar

  # Get the specific torque for each sample's joint using advanced indexing.
  sim_torque = sim_torques[np.arange(n_samples), joint_idx]  # (n,)

  this_joint_angles = joint_angles[np.arange(n_samples), joint_idx]  # (n,)
  this_joint_angles = this_joint_angles[:, np.newaxis]  # (n, 1)

  # Base features: powers of sim_torque for this joint
  features_list.append(np.ones(n_samples))  # bias; (n_samples,)
  features_list.append(sim_torque)  # (n,)
  features_list.append(sim_torque**2)  # (n,)
  features_list.append(sim_torque**3)  # (n,)

  # This joint's angle features
  features_list.append(np.sin(this_joint_angles))  # (n, 1)
  features_list.append(np.cos(this_joint_angles))  # (n, 1)

  # # All joint angle features
  # features_list.append(np.sin(joint_angles))  # (n, 6)
  # features_list.append(np.cos(joint_angles))  # (n, 6)

  # # Higher freqs for finer detail
  # features_list.append(np.sin(3 * joint_angles))  # (n, 6)
  # features_list.append(np.cos(3 * joint_angles))  # (n, 6)

  # # Cross-terms: this joint's sim_torque modulated by all joint angles
  # sim_torque_expanded = sim_torque[:, np.newaxis]  # (n, 1)
  # features_list.append(sim_torque_expanded * np.sin(joint_angles))  # (n, 6)
  # features_list.append(sim_torque_expanded * np.cos(joint_angles))  # (n, 6)

  # # Inter-joint torque coupling: other joints' torques can affect this joint
  # features_list.append(sim_torques)  # Linear coupling for all joints # (n, 6)
  # features_list.append(sim_torques * np.sin(this_joint_angles))
  # features_list.append(sim_torques * np.cos(this_joint_angles))

  # # Joint coupling terms (some joints work together for gravity compensation)
  # features_list.append(
  #     np.sin(joint_angles[:, 1:2]) * np.cos(joint_angles[:, 2:3])
  # )  # shoulder-elbow coupling  # (n, 1)
  # features_list.append(
  #     joint_angles[:, 0:1] * joint_angles[:, 1:2]
  # )  # base-shoulder coupling  # (n, 1)
  # features_list.append(
  #     np.sin(joint_angles[:, 1:2] + joint_angles[:, 2:3])
  # )  # combined arm pose  # (n, 1)

  # Stack features into matrix: (n_features, n_samps) -> (n_samps, n_features)
  features_matrix = np.column_stack(features_list)  # (n, n_features)

  return features_matrix


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
        np.array(arm_orientation.mounting_quaternion),
        world_gravity,
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
        model_inputs = np.concatenate(
            [sim_torques, np.array(joint_angles), [i]],
        )
        result[i] = grav_comp_adjustment_fn(model_inputs, *params)

    assert len(result) == len(self._joint_ids)
    return result


def _load_sampled_gravity(
    filename: str,
) -> dict[int, Sequence[GravityTorqueSample]]:
  with open(filename, encoding="utf-8") as f:
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
        ),
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

  # Load mapping from joint_idx to GravityTorqueSample
  real_gravity = _load_sampled_gravity(grav_torques_file_path)

  # Create a predictor without any learned parameters to get a sim-only baseline
  # prediction, which we'll add as input features for the regression.
  sim_gravity = _SimGravityTorquePrediction(
      piper_model_path, {}, arm_orientation
  )

  # Dictionary mapping joint_idx to array of learned model parameters.
  per_joint_params = {}

  # Iterate over each joint's samples and fit a per-joint model.
  for joint_idx, samples in real_gravity.items():
    real_torques = []
    input_arrays = []

    for sample in samples:
      real_torques.append(sample.torque)  # (n, 6)
      joint_angles = list(sample.joint_configuration)  # (n, 6)
      model_torques = sim_gravity.predict(joint_angles)  # (n, 6)
      sim_torques = np.array(model_torques)  # (n, 6)

      # Concatenate sim_torques(6) + joint_angles(6) + joint_idx(1) -> (13,).
      model_inputs = np.concatenate(
          [sim_torques, np.array(joint_angles), [joint_idx]],
      )
      input_arrays.append(model_inputs)

    # Convert to format expected by curve_fit
    real_torques = np.array(real_torques)
    x_data = np.array(input_arrays)  # (n, 13)

    # Calculate expected number of features using first sample
    sample_features = _build_model_features(x_data[:1])
    n_total_features = sample_features.shape[1]

    # Initial guess: mostly zeros except for the linear sim_torque term
    p0 = np.zeros(n_total_features)
    p0[1] = 1.0  # sim_torque linear term (index 1 in feature vector)
    # p0 = np.random.uniform(-1, 1, n_total_features)

    fit = optimize.curve_fit(
        grav_comp_adjustment_fn,
        x_data,
        real_torques,
        p0=p0,
        maxfev=2000,  # Increase max function evaluations for complex fit
        full_output=True,  # Return additional info including residuals
    )
    opt_params = fit[0]  # Extract optimized parameters
    infodict = fit[2]  # Info dict with residuals and other details
    mesg = fit[3]  # Message about convergence
    ier = fit[4]  # Integer flag about convergence

    print(
        f"Joint {joint_idx}: Using gravity compensation model with "
        f"{len(opt_params)} parameters",
    )

    print(f"Joint {joint_idx} convergence: {mesg} (ier={ier})")
    # print("Residuals:", infodict["fvec"])
    print("Residuals (sum):", np.abs(infodict["fvec"]).sum())
    print("Num func evals:", infodict["nfev"], "\n")

    per_joint_params[joint_idx] = opt_params

  return _SimGravityTorquePrediction(
      piper_model_path,
      per_joint_params,
      arm_orientation,
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
        piper_model_path,
        grav_torques_file_path,
        arm_orientation,
    )

  def gravity_model(self, cur_joints: Sequence[float]) -> Sequence[float]:
    return self._gravity_model.predict(cur_joints)

  def step(self) -> None:
    cur_joints = self._robot.get_joint_positions()
    cur_vel = np.array(self._robot.get_joint_velocities())

    hover_torque = np.array(self._gravity_model.predict(cur_joints))

    # Stability torque acts to counteract joint movement. A dampener basically.
    stability_torque = -cur_vel * 1.0
    applied_torque = hover_torque + stability_torque

    self._controller.command_torques(list(applied_torque))
