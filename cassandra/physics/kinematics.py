from cassandra.utils import skew_symmetric
import numpy as np
import quaternion as q
import math
import abc


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


class Measurable(abc.ABC):
  """
  Abstract class for objects whose properties can be measured.
  """
  @abc.abstractmethod
  def measure(self):
    pass


class Dynamic(abc.ABC):
  """
  Abstract class for objects that evolve over time.
  """
  @abc.abstractclassmethod
  def update(self, timestep, *args, **kwargs):
    pass


class Differentiable(Dynamic):
  """
  Abstract class for objects that evolve over time governed by a diferential equation
  that can be represented in the form Y' = F(Y). Currently, this does not support explicit
  time dependency in F. 
  """
  @property
  @abc.abstractmethod
  def Y(self):
    """
    Abstract method to be overwritten to return the current status of the object ready for differential operations,
    """
    pass

  @Y.setter
  @abc.abstractmethod
  def Y(self, y):
    """
    Abstract method to be overwritten to set the object status based on the result of the differential operation.
    """
    pass

  @abc.abstractmethod
  def F(self, y):
    """
    Abstract method to be overwritten to return the value of the derivative of the status as a funtion of itself.
    """
    pass

  def update(self, timestep, integrator, *args, **kwargs):
    """
    Updates the state of the object according to the differential equation that governs it
    and the numerical integration method used to perform the step.
    """
    current = self.Y
    new = integrator.step(current, self.F, timestep)
    self.Y = new
    return super().update(timestep, *args, **kwargs)


class Loggable(Dynamic):
  """
  Abstract class for object whose properties need to be logged.
  """
  @abc.abstractmethod
  def status(self):
    """
    Abstract method to be overwritten to return the current status of the object
    """
    pass

  def update(self, timestep, *args, **kwargs):
    """
    Append the status of the object to the returned value of the update method
    """
    ret = super().update(timestep, *args, **kwargs)
    status = self.status()
    if ret is not None:
      return ret.update(status)
    return status


class RigidBody(abc.ABC):
  """
  A generic rigid body with basic properties.
  
  Parameters
  ----------
  position : array_like, default=np.zeros(3)
    The initial position of the body's center of mass with respect to the parent's coordinate system.
  orientation : quaternion, default=np.quaternion(1, 0, 0, 0)
    The initial orientation of the body's principal axis of rotation with respect to the parent's coordinate system.

  Attributes
  ----------
  mass : scalar
    The total mass of the body.
  position : array_like
    The position of the body's center of mass with respect to the parent's coordinate system.
  inertia : ndarray
    3x3 matrix representing the body's total inertia tensor along its principal axis of rotation.
  orientation : quaternion
    The orientation of the body's principal axis of rotation wth respect to the parent's coordinate system.
  """
  def __init__(
    self,
    position=np.zeros(3),
    orientation=np.quaternion(1, 0, 0, 0),
    *args,
    **kwargs
  ):
    super().__init__(*args, **kwargs)
    self.position = position
    self.orientation = orientation

  @property
  @abc.abstractmethod
  def mass(self):
    """
    Abstract method to be overwritten with a case specific way to return the object's total mass.
    """
    return 0

  @property
  @abc.abstractmethod
  def inertia(self):
    """
    Abstract method to be overwritten with a case specific way to return the object's total inertia.
    """
    return np.zeros((3, 3))


class FreeBody(RigidBody, Differentiable):
  """
  A rigid body moving freely through space.

  Parameters
  ----------
  lin_velocity : array_like, default=np.zeros(3)
    The initial linear velocity in the parent's frame of reference.
  ang_velocity : array_like, default=np.zeros(3)
    The initial angular velocity in the body's frame of reference.
  gravity : Gravity, default=None
    The gravity which the body is subject to.

  Attributes
  ----------
  lin_velocity : array_like
    The current linear velocity in the parent's frame of reference.
  ang_velocity : array_like
    The current angular velocity in the body's frame of reference.
  gravity : Gravity
    The gravity which the body is subject to.
  """
  def __init__(
    self,
    lin_velocity=np.zeros(3),
    ang_velocity=np.zeros(3),
    gravity=None,
    *args,
    **kwargs
  ):
    super().__init__(*args, **kwargs)
    if gravity is None:
      gravity = Gravity()
    self.gravity = gravity
    self.lin_velocity = lin_velocity
    self.ang_velocity = ang_velocity

  @property
  def mass(self):
    """Returns the full mass of the body."""
    return super().mass

  @property
  def inertia(self):
    """Returns the body's inertia along it's principal axis of rotation."""
    return super().inertia

  @abc.abstractmethod
  def external_forces(self):
    """
    Abstract method to be overwritten to return the external forces acting on the body.
    """
    pass

  def quaternion_omega_matrix(self, w=None):
    """
    Generates the Omega matrix from the rigid body's angular velocity needed to compute
    the quaternion's derivative.\n 
    For more information, see https://onlinelibrary.wiley.com/doi/pdf/10.1002/nme.2586.

    Parameters
    ----------
    w : array_like, default=None
      The angular velocity with which to construct the Omega matrix.
    
    Returns
    -------
    ndarray
      The Omega matrix.
    """
    if w is None:
      w = self.ang_velocity
    return np.array([
      [ 0  , -w[0], -w[1], -w[2]],
      [w[0],   0  ,  w[2], -w[1]],
      [w[1], -w[2],   0  ,  w[0]],
      [w[2],  w[1], -w[0],   0  ] 
    ])

  def parse_Y(self, y):
    """
    Parses a status vector Y into its different components: position, orientation, linear and angular velocity.

    Parameters
    ----------
    y : array_like
      The object's status vector to parse.

    Returns
    -------
    List
      The corresponding list of position, orientation, linear velocity and angular velocity.
    """
    position = y[0:3]
    orientation = q.from_float_array(y[3:7])
    lin_velocity = y[7:10]
    ang_velocity = y[10:13]
    return position, orientation, lin_velocity, ang_velocity

  @property
  def Y(self):
    """array_like: the objects status vector."""
    return np.concatenate((
      self.position,
      q.as_float_array(self.orientation),
      self.lin_velocity,
      self.ang_velocity
    ))
  
  @Y.setter
  def Y(self, y):
    """
    Sets the objects attributes according to a status vector

    Parameters
    ----------
    y : array_like
      The status vector with which to update the object
    """
    position, orientation, lin_velocity, ang_velocity = self.parse_Y(y)
    self.position = position
    self.orientation = orientation
    self.lin_velocity = lin_velocity
    self.ang_velocity = ang_velocity

  def F(self, y):
    """
    Calculates the current derivative of the status vector as a function of itself.

    Parameters
    ----------
    y : array_like
      The status vector with which to compute the derivative.

    Returns
    -------
    array_like
      The derivative of the status vector
    """
    position, orientation, lin_velocity, ang_velocity = self.parse_Y(y)
    #Quaternion velocity
    omega_matrix = self.quaternion_omega_matrix(ang_velocity)
    quaternion_velocity = omega_matrix.dot(q.as_float_array(orientation)) * 0.5
    #Force
    ext_forces = self.external_forces()
    lin_acceleration = sum([force.linear(orientation) for force in ext_forces.values()]) / self.mass # F / m
    if self.gravity:
      lin_acceleration += self.gravity.linear()
    #Torque
    cm_torque = sum([force.torque(self.orientation) for force in ext_forces.values()]) # M
    non_inertial_term = np.cross(self.ang_velocity, self.inertia.dot(self.ang_velocity)) # w x (I.w)
    ang_acceleration = np.linalg.inv(self.inertia).dot(cm_torque - non_inertial_term) # from Euler's equations I.w_dot + w x (I.w) = M  

    return np.concatenate((
      lin_velocity,
      quaternion_velocity,
      lin_acceleration,
      ang_acceleration
    ))

  
class CompositeBody(RigidBody, Dynamic):
  """
  A rigid body made up of different components.

  Parameters
  ----------
  components : dict, default=None
    The dictionary with the initial components in the form {'component_name', component}.

  Attributes
  ----------
  components : dict
    The dictionary with the current subcomponents attached to the body.
  """
  def __init__(self, components=None, *args, **kwargs):
    super().__init__(*args, **kwargs)
    if components is None:
      components = {}
    self.components = components

  @property
  def mass(self):
    """scalar: The total mass of the rigid body calculated from its components."""
    ret = sum([component.mass for component in self.components.values()])
    ret += super().mass
    return ret

  @property
  def inertia(self):
    """ndarray: The total inertia of the rigid body calculated from its components."""
    ret = sum([component.inertia for component in self.components.values()])
    ret += super().inertia
    return ret
  
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
    Readjusts the components' positions so that the center of mass of the rigid body lies in 
    the origin of the reference frame.
    """
    cm = self.cm
    if cm.any():
      for component in self.components.values():
        component.position -= cm

      self.position += cm

  def attach(self, component):
    """
    Attaches a new component to the body.

    Parameters
    ----------
    component : RigidBody
      The component being attached
    """
    self.components.update(component)

  def detach(self, key):
    """ 
    Detaches an existing component from the body.

    Parameters
    ----------
    key : str
      The components key identifier
    """
    self.component.pop(key, None)

  def update(self, timestep, *args, **kwargs):
    """
    Updates all of the body's components before updating the body itself.

    Parameters
    ----------
    timestep : scalar
      The timestep with which to advance the state of the object.
    """
    ret = {}
    for component in self.components.values():
      res = component.update(timestep, *args, **kwargs)
      if res:
        ret.update(res)
    res = super().update(timestep, *args, **kwargs)
    if res:
      ret.update(res)
    return ret


class MassiveBody(RigidBody):
  """
  A massive rigid body.

  Parameters
  ----------
  proper_mass : scalar
    The initial total mass of the component.
  proper_inertia : ndarray
    3x3 matrix representing the component's initial inertia tensor along its principal axis of rotation.

  Attributes
  ----------
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
      proper_mass=1,
      proper_inertia=np.eye(3),
      *args,
      **kwargs
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
    
    super().__init__(*args, **kwargs)
    self.proper_mass = proper_mass
    self.proper_inertia = proper_inertia

  @property
  def mass(self):
    """number: The component's total mass."""
    ret = self.proper_mass
    ret += super().mass
    return ret

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
    ret += super().inertia
    return ret


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
      force_type=0
  ):
    self.vector = vector 
    self.anchor = anchor
    self.type = force_type

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
      return np.cross(self.anchor, q.rotate_vectors(orientation, self.vector))


class Gravity(Force):
  """
  Basic gravitational force in the Z direction.

  Parameters
  ----------
  magnitude : scalar, default=9.807
    The strength of the gravitational field.
  """
  def __init__(self, magnitude=9.807):
    vector = np.array([0, 0, -magnitude])
    super().__init__(vector=vector, force_type=Force.EXTERNAL)