import numpy as np

class Motor:
  """
  A basic solid rocket motor.

  Parameters
  ----------
  thrust_curve : ndarray
    The thrust (coordinate 1) v. time (coordinate 0) curve for the motor. 
  mass_curve : ndarray
    The mass (coordinate 1) v. time (coordinate 0) curve for the motor.

  Attributes
  ----------
  thrust_curve : ndarray
    The thrust (coordinate 1) v. time (coordinate 0) curve for the motor. 
  mass_curve : ndarray
    The mass (coordinate 1) v. time (coordinate 0) curve for the motor.
  ignition_time : scalar
    Time since motor ignition. Used to calculate current thrust and mass.
  """

  def __init__(self, thrust_curve, mass_curve):
    self.thrust_curve = np.concatenate(
      [thrust_curve,
      np.array([[thrust_curve[0, -1] + 0.1], [0]])],
      axis=1
    )
    self.mass_curve = np.copy(mass_curve)
    self.ignition_time = 0

  def thrust(self, time=None):
    """
    Get motor thrust at a specific point in time since ignition.

    Parameters
    ----------
    time : scalar, default=None
      The time at which to calculate thrust. Defaults to the motor's ignition time.

    Returns
    -------
    scalar
      The thrust at the requested time.
    """
    if time is None:
      time = self.ignition_time
    thrust = self.thrust_curve[:, self.thrust_curve[0] <= time][1, -1]
    return thrust

  def mass(self, time=None):
    """
    Get motor's mass at a specific point in time since ignition.

    Parameters
    ----------
    time : scalar, default=None
      The time at which to calculate the mass. Defaults to the motor's ignition time.

    Returns
    -------
    scalar
      The motor's mass at the requested time.
    """

    if time is None:
      time = self.ignition_time
    mass = self.mass_curve[:, self.mass_curve[0] <= time][1, -1]
    return mass 

  def update(self, timestep):
    """
    Update the motor's ignition time.

    Parameters
    ----------
    timestep : scalar
      The ellapsed time with which to update the motor's ignition time.
    """
    self.ignition_time += timestep


class UniformMotor(Motor):
  """
  A solid rocket motor with a uniform constant burn and mass loss over a certain period of time.

  Parameters
  ----------
  max_thrust : scalar, default=0
    The constant thrust of the burning motor.
  burn_time : scalar, default=0
    The total burn time of the motor.
  full_mass : scalar, default=0
    The initial weight of the motor.
  empty_mass : scalar, default=0
    The final weight of the depleted motor.
  """

  def __init__(self, max_thrust=0, burn_time=0, full_mass=0, empty_mass=0):
    thrust_curve = np.array([[0, burn_time], [max_thrust, 0]])
    mass_curve = np.array([[0, burn_time], [full_mass, empty_mass]])
    super().__init__(thrust_curve, mass_curve)
