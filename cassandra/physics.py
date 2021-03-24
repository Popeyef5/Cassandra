from cassandra.utils import skew_symmetric
import numpy as np
import quaternion as q
import math


class EulerAngles:
  """
  Tuple of Euler angles in the form 3-2-1 (yaw, pitch, roll).

  Although generic, this class will often be populated with tuples
  which have yaw=0.

  Parameters 
  ----------
  roll : scalar, default=0
    The initial roll component.
  pitch : scalar, default=0
    The pitch component.
  yaw : scalar, default=0
    The yaw component.

  Attributes
  ----------
  roll : scalar
    The roll amount.
  pitch : scalar
    The pitch amount.
  yaw : scalar
    The yaw amount.
  """

  def __init__(self, roll=0, pitch=0, yaw=0):
    self.roll, self.pitch, self.yaw = roll, pitch, yaw

  def to_quaternion(self):
    """
    Return the transformation in quaternion form.
    Based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles.

    Returns
    -------
    quaternion
      The transformation in quaternion representation.
    """
 
    # Abbreviations for the various angular functions
    cy = math.cos(self.yaw * 0.5)
    sy = math.sin(self.yaw * 0.5)
    cp = math.cos(self.pitch * 0.5)
    sp = math.sin(self.pitch * 0.5)
    cr = math.cos(self.roll * 0.5)
    sr = math.sin(self.roll * 0.5)

    # Quaternion components
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.quaternion(w, x, y, z)   
 
  @classmethod
  def from_quaternion(cls, q):
    """
    Constructs the Euler representation of a quaternion.
    Based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles.

    Parameters
    ----------
    q : quaternion
      The quaternion representing the transformation.

    Returns
    -------
    EulerAngles
      The transformation in the Euler representation.
    """

    # Roll (Body X)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Body Y)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if (abs(sinp) >= 1):
      pitch = math.copysign(math.pi * 0.5, sinp) # use 90 degrees if out of range
    else:
      pitch = math.asin(sinp)

    # Yaw (Body Z)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return cls(roll, pitch, yaw);   


class Component:
  """
  A massive component of a solid body.

  Parameters
  ----------
  name : str
    The name of the component.
  mass : scalar
    The initial total mass of the component.
  position : array_like
    The initial position of the component's center of mass with respect to the parent's coordinate system.
  proper_inertia : ndarray
    3x3 matrix representing the component's initial inertia tensor along its principal axis of rotation.
  orientation : quaternion
    The initial orientation of the component's principal axis of rotation with respect to the parent's coordinate system.

  Attributes
  ----------
  name : str
    The name of the component.
  mass : scalar
    The total mass of the component.
  position : array_like
    The position of the component's center of mass with respect to the parent's coordinate system.
  proper_inertia : ndarray
    3x3 matrix representing the component's inertia tensor along its principal axis of rotation.
  orientation : quaternion
    The orientation of the component's principal axis of rotation wth respect to the parent's coordinate system.
  """

  def __init__(
      self,
      name='',
      mass=1,
      position=np.zeros(3),
      proper_inertia=np.eye(3),
      orientation=np.quaternion(1, 0, 0, 0)
  ):
    # For now, inertia is computed as the full tensor. This allows for more 
    # realism (taking into account the real intertia tensor). However, for most applications,
    # the only moving part in a rocket is its motor, which not only weighs a fraction of the 
    # total mass but also does not tilt too much, making the approximation of keeping the
    # principal axis of inertia as they are a valid one. Under this approximation, the principal
    # axis of inertia do not change and the inertia tensor can be stored as a vector containing
    # the tensor's eigenvalues. This would speed up calculations quite
    # a bit, since no matrix multiplications or (more importantly) matrix inversions are 
    # performed while updating angular velocity. Maybe this could be offered as a choice:
    # realism vs efficiency.

    self.name = name
    self.mass = mass
    self.position = position
    self.proper_inertia = proper_inertia
    self.orientation = orientation

  @property
  def inertia(self):
    """ndarray: The component's inertia in the parent's reference frame."""
    ret = np.copy(self.proper_inertia)
    if self.orientation.angle():
      rot = q.as_rotation_matrix(self.orientation)
      ret = rot.T.dot(ret).dot(rot)
    if self.position.any():
      skew = skew_symmetric(self.position) 
      parallel_axis_component = self.mass * skew.dot(skew)
      ret -= parallel_axis_component
    return ret

  def data(self):
    """dict: The component's current position and orientation."""
    return {
      f'{self.name}_position': np.copy(self.position),
      f'{self.name}_orientation': np.copy(self.orientation)
    }


class Kinematics:
  """
  The kinematics of a rigid body.

  Parameters
  ----------
  position : array_like, default=np.zeros(3)
    The initial position of the rigid body with respect to the world reference frame. 
    Defaults to [0, 0, 0].
  lin_velocity : array_like, default=np.zeros(3)
    The initial linear velocity of the rigid body in the world reference frame.
    Defaults to [0, 0, 0]. 
  orientation : quaternion, default=np.quaternion(1, 0, 0, 0)
    The initial orientation of the body's reference frame encoded in a quaternion.
    Defaults to [1, 0, 0, 0].
  ang_velocity : array_like, default=np.zeros(3)
    The initial angular velocity of the rigid body in the world reference frame.
    Defaults to [0, 0, 0].
  components : dict, default={}
    The massive components that make up the rigid body in the form {'name': component}.
    Defaults to {}.
  gravity : bool, default=True
    Whether the rigid body is affected by gravity or not. Defaults to True.

  Attributes
  ----------
  position : array_like
    The position of the rigid body with respect to the world reference frame.
  lin_velocity : array_like
    The linear velocity of the rigid body in the world reference frame.
  orientation : quaternion
    The orientation of the rigid body's reference frame encoded in a quaternion.
  ang_velocity : array_like
    The angular velocity of the rigid body in the world reference frame.
  components : dict
    The massive components that make up the rigid body.
  gravity : Force or None
    The gravity acceleration if the body is affected by it, else None. 
  """

  def __init__(
      self,
      position=np.zeros(3), 
      lin_velocity=np.zeros(3), 
      orientation=np.quaternion(1, 0, 0, 0), 
      ang_velocity=np.zeros(3),
      components={},
      gravity=True
  ):
    self.position = position
    self.lin_velocity = lin_velocity

    self.orientation = orientation
    self.ang_velocity = ang_velocity

    self.components = components
    self.readjust_component_positions()

    self.gravity = Force(vector=np.array([0, 0, -9.8]), type=Force.EXTERNAL) if gravity else None

  @property
  def mass(self):
    """scalar: The total mass of the rigid body calculated from its components."""
    return sum([component.mass for component in self.components.values()])

  @property
  def inertia(self):
    """ndarray: The total inertia of the rigid body calculated from its components."""
    return sum([component.inertia for component in self.components.values()])

  @property
  def cm(self):
    """
    array_like: The position of the rigid body's center of mass. This is used to constantly
    update the reference frame and ensure that the center of mass is centered.
    """
    if self.mass:
      ret = sum([component.position * component.mass for component in self.components.values()])
      return ret / self.mass
    else:
      return np.zeros(3)

  def readjust_component_positions(self):
    """
    Readjust the components' positions so that the center of mass of the rigid body lies in 
    the origin of the reference frame.
    """
    cm = self.cm
    if cm.any():
      for component in self.components.values():
        component.position -= cm

      self.position += cm
   
  def quaternion_omega_matrix(self):
    """
    Generates the Omega matrix from the rigid body's angular velocity needed to compute
    the quaternion's derivative.\n 
    For more information, see https://onlinelibrary.wiley.com/doi/pdf/10.1002/nme.2586.
    
    Returns
    -------
    ndarray
      The Omega matrix.
    """
    w = self.ang_velocity
    return np.array([
      [ 0  , -w[0], -w[1], -w[2]],
      [w[0],   0  ,  w[2], -w[1]],
      [w[1], -w[2],   0  ,  w[0]],
      [w[2],  w[1], -w[0],   0  ] 
    ])

  def update_position(self, timestep, ext_forces):
    """
    Update the positon of the center of mass according to the ellapsed time and external forces.
    
    Parameters 
    ----------
    timestep : scalar
      The ellapsed time.
    ext_forces : dict
      The external forces applied to the rigid body.

    Returns
    -------
    array_like
      A copy of the rigid body's position.
    """
    cm_force = sum([force.linear(self.orientation) for force in ext_forces.values()]) # F
    if self.gravity:
      cm_force += self.gravity.linear() * self.mass
    self.lin_velocity += cm_force / self.mass * timestep # dv = a*dt = F/m*dt 
    self.position += self.lin_velocity * timestep # dx = v*dt 
    return np.copy(self.position)

  def update_orientation(self, timestep, ext_forces):
    """
    Update the orientation of the rigid body according to the ellapsed time and external forces.
   
    Parameters
    ----------
    timestep : scalar
      The ellapsed time.
    ext_forces : dict
      The external forces applied to the rigid body.

    Returns
    -------
    quaternion
      A copy of the rigid body's orientation.
    """
    cm_torque = sum([force.torque(self.orientation) for force in ext_forces.values()]) # M 
    non_inertial_term = np.cross(self.ang_velocity, self.inertia.dot(self.ang_velocity)) # w x (I.w)
    ang_acceleration = np.linalg.inv(self.inertia).dot(cm_torque - non_inertial_term) #+ from Euler's equations I.w_dot + w x (I.w) = M  
    self.ang_velocity += ang_acceleration * timestep # dw = w_dot*dt 
    omega_matrix = self.quaternion_omega_matrix()
    d_orientation = omega_matrix.dot(q.as_float_array(self.orientation)) * 0.5 * timestep
    self.orientation += q.from_float_array(d_orientation) # q_dot=0.5*Omega*q
    return np.copy(self.orientation)

  def update(self, timestep, ext_forces):
    """
    Update the rigid body's physics according to the ellapsed tume and external forces.

    Parameters
    ----------
    timestep : scalar
      The ellapsed time.
    ext_forces : dict
      The external forces applied to the rigid body.

    Returns
    -------
    dict
      The physical status of the body and its components.
    """
    position = self.update_position(timestep, ext_forces)
    orientation = self.update_orientation(timestep, ext_forces)
    ret = {
      'cm_position': position,
      'cm_orientation': orientation
    }
    for component in self.components.values():
      ret.update(component.data())
    return ret 

  def measure(self):
    """
    Probe the current state of the rigid body.

    Returns
    -------
    dict
      Relevant physical measurements performed on the rigid body.
    """
    ret = {
      'ang_velocity': self.ang_velocity,
      'altitude': self.position[2],
    }


class Force:
  """
  A force applied on a body at a specific point. 

  Parameters
  ----------
  vector : array_like
    The force vector itself, consisting of magnitude and direction.
  anchor : array_like
    The point where the force is applied, always in the reference frame of the body.
  type : {0, 1}
    0 if the `vector` is in the body's frame of reference or 1 if it is in the world frame.

  Attributes
  ----------
  vector : array_like
    The force vector itself, consisting of magnitude and direction.
  anchor : array_like
    The point where the force is applied, always in the reference frame of the body.
  type : {0, 1}
    0 if the `vector` is in the body's frame of reference or 1 if it is in the world frame.
  """

  INTERNAL = 0
  EXTERNAL = 1
 
  def __init__(
      self,
      vector=np.zeros(3),
      anchor=np.zeros(3),
      type=0
  ):
    self.vector = vector 
    self.anchor = anchor
    self.type = type

  def linear(self, orientation=None):
    """
    Calculates the force applied to the body, adjusted for the correct frame of reference.

    Parameters
    ----------
    orientation : quaternion, default=None
      The orientation to update the force direction.

    Returns
    -------
    array_like
      The resulting applied force.
    """
    if orientation is None or self.type == self.EXTERNAL:
      return self.vector
    else:
      return q.rotate_vectors(orientation.inverse(), self.vector)

  def torque(self, orientation=None):
    """
    Calculates the torque applied to the center of mass of the body in its reference frame.

    Parameters
    ----------
    orientation : quaternion
      The orientation with which to correct the force direction.

    Returns
    -------
    array_like
      The torque applied to the body in its reference frame.
    """
    if orientation is None or self.type == self.INTERNAL:
      return np.cross(self.anchor, self.vector)
    else:
      vector = np.cross(self.anchor, q.rotate_vectors(orientation, self.vector))

