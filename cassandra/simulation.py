from cassandra.log import Logbook

class Simulation:
  """
  A simulation of a TVC controlled rocket.

  Parameters
  ----------
  rocket : Rocket
    The rocket.

  Attributes 
  ----------
  rocket : Rocket
    The rocket.
  logbook : Logbook
    The logbook where the simulation data is stored.
  """
  def __init__(self, rocket):
    self.rocket = rocket
    self.logbook = Logbook()
  
  def run(self, sim_time, sim_timestep):
    """
    Run the simulation for a given time and with a given precision.

    Parameters
    ----------
    sim_time : scalar
      The time to run the simulation for.
    sim_timestep : scalar
      The timestep with which to update the rocket periodically.

    Returns
    -------
    Logbook
      The logbook containing the data of the simulation. 
    """
    curr_time = 0
    
    while(curr_time < sim_time):
      data = self.rocket.update(sim_timestep)
      data.update({'time': curr_time})
      self.logbook.add_data(data)
      curr_time += sim_timestep 
    return self.logbook


  def plot(self, include=None, in_terminal=False):
    """
    Plot relevant data.
 
    Parameters
    ----------
    include : list, default=None
      The list of plots.
    in_terminal : bool, default=False
      If True, shows the plots in terminal with an oldschool NASA vibe. If False, uses boring matplotlib.
    """
    self.logbook.plot(include, in_terminal) 
  
  def animate(self):
    """
    Animate the rocket's journey through time and space.
    """
    self.logbook.animate()


