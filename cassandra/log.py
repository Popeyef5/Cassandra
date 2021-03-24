import math
import numpy as np
import quaternion as q


class Log:
  """
  A log that stores the information of a single variable through the simulation time.

  Parameters
  ----------
  name : str
    The Log identifier.
  type : {Log.VECTOR, Log.QUATERNION, Log.SCALAR}
    The type of variable being stored.
 
  Attributes
  ----------
  name : str
    The Log identifier.
  type : {Log.VECTOR, Log.QUATERNION, Log.SCALAR}
    The type of variable being stored.
  data : list
    The data being stored.
  """

  VECTOR = 'vector'
  QUATERNION = 'quaternion'
  SCALAR = 'scalar'

  def __init__(self, name, type):
    self.name = name
    self.data = []
    self.type = type

  def add_data(self, data_point):
    """
    Add new data to the log.

    Parameters
    ----------
    data_point : scalar or quaternion or array_like
      The data to append.
    """
    self.data.append(data_point)

  def get_data(self, ops=None):
    """
    Get preprocessed data.
 
    Parameters
    ----------
    ops : list, default=None
      The list of operations to perform on the data before returning it.

    Returns
    -------
    list
      The processed data.
    """
    ret = self.data.copy()
    if ops is not None:
      for op in ops:
        ret = [op(dp) for dp in ret]
    return ret


class Logbook:
  """
  A collection of Logs.

  Attributes
  ----------
  logs : dict
    A dictionary with all the logs identified by a human readable string.
  """

  def __init__(self):
    self.logs = {
      'time': Log('Time', Log.SCALAR),
      'thrust': Log('Thrust', Log.SCALAR),
      'cm_position': Log('Center of mass position', Log.VECTOR),
      'cm_orientation': Log('Center of mass orientation', Log.QUATERNION),
      'cm_linear_velocity': Log('Center of mass linear velocity', Log.VECTOR),
      'cm_angular_velocity': Log('Center of mass angular velocity', Log.VECTOR),
      'mount_position': Log('Mount position', Log.VECTOR),
      'mount_orientation': Log('Mount orientation', Log.QUATERNION),
      'frame_position': Log('Frame position', Log.VECTOR),
      'frame_orientation': Log('Frame orientation', Log.QUATERNION)
    }

  def add_log(self, name, log):
    """
    Add new log to the logbook.

    Parameters
    ----------
    name : str
      The log's identifier.
    log : Log
      The log to add. 
    """
    self.logs['name'] = log

  def add_data(self, data_dict):
    """
    Add batch data to multiple logs.

    Parameters
    ----------
    data_dict : dict
      The dictionary containing the data points for the appropriate logs tagged by the log identifier.
    """
    for log_name in data_dict:
      if log_name not in self.logs:
        print(log_name)
        self.logs[log_name] = Log(log_name)

      self.logs[log_name].add_data(data_dict[log_name])

  def plot(self, include, in_terminal=False):
    """
    Plot relevant variables in a flexible manner.

    Parameters
    ----------
    include : list
      The list of plots to include.
    in_terminal : bool, default=False
      If True, shows plots in terminal, giving an oldschool NASA vibe. If False, uses boring matplotlib. 
    """
    if in_terminal:
      import plotext as plt 
    else:
      import matplotlib.pyplot as plt

    for plot in include:
      x_log = self.logs[plot['x']['logname']]
      x_data = x_log.get_data(plot['x']['ops'])

      y_log = self.logs[plot['y']['logname']]
      y_data = y_log.get_data(plot['y']['ops'])

      if in_terminal:
        plt.clear_plot()
        plt.nocolor()
        plt.figsize(75, 25) 
      else:
        plt.figure()
      plt.plot(x_data, y_data)
      plt.show()
 
  def animate(self):
    """
    Animates the rocket's movement according to the data in the logs.
    """
    # Ideally we'd use something other than matplotlib. It really wasn't designed for
    # this sort of application.
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib.animation import FuncAnimation
    import numpy as np
    import quaternion as q

    cm_positions = self.logs['cm_position'].get_data()
    cm_orientations = self.logs['cm_orientation'].get_data() 

    frame_rel_positions = self.logs['frame_position'].get_data()
    frame_rel_orientations = self.logs['frame_orientation'].get_data()

    mount_rel_positions = self.logs['mount_position'].get_data()
    mount_rel_orientations = self.logs['mount_orientation'].get_data()
    
    thrust = self.logs['thrust'].get_data()

    fig, ax = plt.subplots(2, 2)
    ax[0, 0].set_xlim(-21, 21)
    ax[0, 0].set_ylim(-2, 20)
    ax[0, 1].set_xlim(-21, 21)
    ax[0, 1].set_ylim(-2, 20)
    ax[1, 0].set_xlim(-21, 21)
    ax[1, 0].set_ylim(-21, 21)
   
    # Rockets definition 
    rocket_xz = patches.Rectangle((0, 0.5), 0.2, 1)
    rocket_yz = patches.Rectangle((0, 0.5), 0.2, 1)
    rocket_xy = patches.Rectangle((0, 0), 0.2, 0.2)

    # Thrust definition
    thrust_xz = patches.Rectangle((0, 0), 0.1, -1, facecolor='r')
    thrust_yz = patches.Rectangle((0, 0), 0.1, -1, facecolor='r')

    # Floor
    floor_xz = patches.Rectangle((-50, 0), 100, -2, facecolor='g')
    floor_yz = patches.Rectangle((-50, 0), 100, -2, facecolor='g')

    def init():
      ax[0, 0].add_patch(rocket_xz)
      ax[0, 0].add_patch(thrust_xz)
      ax[0, 0].add_patch(floor_xz)
      ax[0, 1].add_patch(rocket_yz)
      ax[0, 1].add_patch(thrust_yz)
      ax[0, 1].add_patch(floor_yz)
      ax[1, 0].add_patch(rocket_xy)
      return rocket_xz, thrust_xz, floor_xz, rocket_yz, thrust_yz, floor_yz, rocket_xy
  
    def update(frame_count):
      cm_position = cm_positions[frame_count]
      cm_orientation = cm_orientations[frame_count]

      frame_position = cm_position + q.rotate_vectors(
        cm_orientation,
        frame_rel_positions[frame_count] + np.array([-0.1, -0.1, -0.5])
      )
      frame_direction = q.rotate_vectors(
        frame_rel_orientations[frame_count] * cm_orientation,
        np.array([0, 0, 1])
      )
      
      mount_position = cm_position + q.rotate_vectors(
        cm_orientation,
        mount_rel_positions[frame_count] 
      )
      mount_direction = q.rotate_vectors(
        mount_rel_orientations[frame_count] * cm_orientation,
        np.array([0, 0, 1])
      )
 

      frame_angle_xz = np.rad2deg(math.atan2(
        frame_direction[0],
        frame_direction[2] 
      )) 
      frame_angle_yz = np.rad2deg(math.atan2(
        frame_direction[1],
        frame_direction[2]
      )) 
    
      # Rockets update 
      rocket_xz.set_xy(frame_position[[0, 2]])
      rocket_xz.angle = frame_angle_xz
      rocket_yz.set_xy(frame_position[[1, 2]])
      rocket_yz.angle = frame_angle_yz
      rocket_xy.set_xy(frame_position[[0, 1]])
     
      thrust_angle_xz = np.rad2deg(math.atan2(
        mount_direction[0],
        mount_direction[2]
      )) 
      thrust_angle_yz = np.rad2deg(math.atan2(
        mount_direction[1],
        mount_direction[2]
      ))

      thrust_xz.set_xy(mount_position[[0, 2]])
      thrust_xz.angle = thrust_angle_xz 
      thrust_yz.set_xy(mount_position[[1, 2]])
      thrust_yz.angle = thrust_angle_yz
      return rocket_xz, thrust_xz, floor_xz, rocket_yz, thrust_yz, floor_yz, rocket_xy
    
    anim = FuncAnimation(fig, update,
                         init_func=init,
                         frames=len(cm_positions),
                         interval=20,
                         blit=True)
  
    plt.show()

