import math
import numpy as np
import quaternion as q
import pandas as pd


class Logbook:
  """
  A collection of Logs.

  Attributes
  ----------
  logs : dict
    A pandas DataFrame with all the logs identified by a human readable string.
  """

  def __init__(self):
    self.logs = pd.DataFrame()

  def add_data(self, data):
    """
    Add batch data to logs.

    Parameters
    ----------
    data : dict
      The dictionary containing the data points for the appropriate logs tagged by the log identifier.
    """
    self.logs = self.logs.append(data, ignore_index=True)

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
    # TODO: this only works with the default rocket and is currently for testing only
    # Ideally we'd use something other than matplotlib. It really wasn't designed for
    # this sort of application.
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib.animation import FuncAnimation
    import numpy as np
    import quaternion as q

    cm_positions = self.logs['cm_position']
    cm_orientations = self.logs['cm_orientation']

    frame_rel_positions = self.logs['frame_position']
    frame_rel_orientations = self.logs['frame_orientation']

    mount_rel_positions = self.logs['mount_position']
    mount_rel_orientations = self.logs['mount_orientation']
    
    thrust = self.logs['thrust']

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

