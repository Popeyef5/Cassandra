import numpy as np
import quaternion as q
import math
import abc
from cassandra.physics.kinematics import EulerAngles, Dynamic


class PID:
  """
  A simple PID controller.

  Parameters
  ----------
  Kp : scalar, default=0
    The PID's proportional gain.
  Ki : scalar, default=0
    The PID's integral gain.
  Kd : scalar, default=0
    The PID's derivative gain.

  Attributes
  ----------
  Kp : scalar
    The PID's proportional gain.
  Ki : scalar
    The PID's integral gain.
  Kd : scalar
    The PID's derivative gain.
  integral : scalar
    The accumulated integral of the error.
  last_error : scalar
    The last error recorded. 
  """
  def __init__(self, Kp=0, Ki=0, Kd=0):
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd

    self.integral = 0
    self.last_error = 0

  def calculate(dt, error):
    """
    Calculate the output from an error and an ellapsed time.

    Parameters
    ----------
    dt : scalar
      The ellapsed time since the last calculation.
    error : scalar
      The error input.

    Returns
    -------
    scalar
      The output of the PID.
    """
    # Proportional term
    proportional = error * self.Kp

    # Integral term
    self.integral += error * dt
    integral = self.integral * self.Ki

    # Derivative term
    derivative = (error - self.last_error) / dt * self.Kd
    self.last_error = error

    return proportional + integral + derivative


class FlightSoftware(Dynamic, abc.ABC):
  """
  A flight software for controlling the rocket before, during and after flight.

  This class is meant to be subclassed to specify a specific program method, overriding the default one.

  Parameters
  ----------
  loop_time : scalar, default=0
    The time per loop of the program, used to create a more accurate simulation.

  Attributes
  ----------
  loop_time : scalar
    The time per loop of the program, used to create a more accurate simulation.
  last_execution : scalar
    Keeps track of the last execution of the program in the simulation to know when to run it again.
  """
 
  def __init__(self, loop_time=0):
    self.last_execution = 0    
    self.loop_time = loop_time

  def update(self, timestep, rocket, measurements):
    """
    Update the software according to an ellapsed time, the controlled rocket and the physical measurements.

    Parameters
    ----------
    timestep : scalar
      The time ellapsed since the last update.
    rocket : Rocket
      The rocket being controlled by the software.
    measurements : dict
      The physical information available to the software at this point in time.
    """
    self.last_execution += timestep
    if self.last_execution > self.loop_time:
      self.program(rocket, measurements)
      self.last_execution = 0

  @staticmethod
  @abc.abstractmethod
  def program(rocket, measurements):
    """
    The program being run by the software in loop.

    Parameters
    ----------
    rocket : Rocket
      The rocket being controlled by the software.
    measurements : dict
      The physical information available to the software at this point in time.
    """
    pass


class DummySoftware(FlightSoftware):
  """
  Dummy test software that does absolutely nothing.
  """
  @staticmethod
  def program(rocket, measurements):
    pass


class RandomSoftware(FlightSoftware):
  """
  Basic flight software that controls the rocket randomly.
  """
  @staticmethod
  def program(rocket, measurements):
    roll = np.random.uniform(-rocket.mount.max_roll, rocket.mount.max_roll)
    pitch = np.random.uniform(-rocket.mount.max_pitch, rocket.mount.max_pitch)
    orientation = EulerAngles(roll, pitch, 0).to_quaternion()
    rocket.mount.set_target(orientation)

