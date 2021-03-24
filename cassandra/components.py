from cassandra.physics import Component, Force, EulerAngles
from cassandra.utils import constrain
import numpy as np
import quaternion as q


class Frame(Component):
  """
  The rigid frame of a rocket.

  Parameters
  ----------
  name : str, default='frame'
    A string identifier for the frame.
  height : scalar, default=1
    The height of the frame in meters.
  diameter : scalar, default=0.1
    The diameter of the frame in meters.
 
  Attributes
  ----------
  height : scalar, default=1
    The height of the frame in meters.
  diameter : scalar, default=0.1
    The diameter of the frame in meters.
  """
  
  def __init__(
      self,
      name='frame',
      height=1,
      diameter=0.1,
  ):
    super().__init__(name=name)
    self.height = height
    self.diameter = diameter


class MotorMount(Component):
  """
  A motor mount to attach solid rocket motors to the frame.

  Parameters
  ----------
  name : str, default='mount'
    A string identifier for the mount.
  position : array_like, default=np.zeros(3)
    The initial position of the mount.
  max_speed : scalar
    The mount's maximum angular speed in degrees/second.
  max_roll : scalar
    The mount's maximum amplitude in the roll direction.
  max_pitch : scalar
    The mount's maximum amplitude in the pitch direction. 

  Attributes
  ----------
  max_speed : scalar
    The mount's maximum angular speed in degrees/second.
  max_roll : scalar
    The mount's maximum amplitude in the roll direction.
  max_pitch : scalar
    The mount's maximum amplitude in the pitch direction. 
  target : quaternion
    The mount's target orientation encoded in a quaternion.
  """ 
  def __init__(
      self,
      name='mount',
      position=np.zeros(3),
      max_speed=60,
      max_roll=5,
      max_pitch=5 
  ):
    super().__init__(name=name, position=position)
    self.motor = None
    self.max_speed = np.deg2rad(max_speed) 
    self.max_roll = np.deg2rad(max_roll)
    self.max_pitch = np.deg2rad(max_pitch)
    self.target = np.quaternion(1, 0, 0, 0)

  def attach_motor(self, motor):
    """
    Attach a motor to the mount.

    Parameters
    ----------
    motor : Motor
      The motor to attach.
    """
    self.motor = motor

  def detach_motor(self):
    """
    Detach any existing motors from the mount.
    """
    self.motor = None

  def set_target(self, target):
    """
    Set the mount's target orientation.

    Parameters
    ----------
    target : quaternion
      The desired orientation for the mount encoded in a quaternion.
    """
    target_euler = EulerAngles.from_quaternion(target)
    target_euler.yaw = 0
    target_euler.roll = constrain(target_euler.roll, self.max_roll) 
    target_euler.pitch = constrain(target_euler.pitch, self.max_pitch)
    self.target = target_euler.to_quaternion()

  def update_orientation(self, timestep):
    """
    Update the mount's orientation given a certain ellapsed time.
 
    Parameters
    ----------
    timestep : scalar
      The ellapsed time with which to update the mount.
    """
    # Nasty euler angles are nasty
    target_euler = EulerAngles.from_quaternion(self.target)  
    orientation_euler = EulerAngles.from_quaternion(self.orientation)
 
    roll_delta = target_euler.roll - orientation_euler.roll
    pitch_delta = target_euler.pitch - orientation_euler.pitch
    roll_delta = constrain(roll_delta, self.max_speed * timestep)
    pitch_delta = constrain(pitch_delta, self.max_speed * timestep) 

    orientation_euler.roll += roll_delta
    orientation_euler.pitch += pitch_delta
    self.orientation = orientation_euler.to_quaternion()

  def update(self, timestep):
    """
    Update the mount's physics (orientation and eventual motor) given a certain ellapsed time.

    Parameters
    ----------
    timestep : scalar
      The ellapsed time with which to update the mount.
    """
    if self.motor:
      self.motor.update(timestep)
    self.update_orientation(timestep)

  def thrust(self, timestep):
    """
    Get the thrust produced by the motor given a certain ellapsed time.

    Parameters
    ----------
    timestep : scalar
      The ellapsed time with which to calculate the total thrust.

    Returns
    -------
    Force or None
      The eventual thrust produced, encoded in a Force with the appropriate direction.
    """
    if self.motor is None:
      return None
    else:
      thrust = self.motor.thrust()
      if not thrust:
        return None
      else:
        vector = q.rotate_vectors(self.orientation, np.array([0, 0, 1]))
        return Force(thrust * vector, self.position)      


