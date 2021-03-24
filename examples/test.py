

if __name__ == '__main__':
  from cassandra.rocket import Rocket
  from cassandra.components import MotorMount, Frame
  from cassandra.motors import Motor
  from cassandra.physics import Kinematics 
  from cassandra.control import FlightSoftware
  from cassandra.simulation import Simulation
  import numpy as np


  MAX_MOUNT_SPEED = 1
  PID_KP = 0
  PID_KI = 0
  PID_KD = 0
  SIM_CLOCK_SPEED = 10e2 
  FLIGHT_CONTROLLER_LOOP_TIME = 0.5
  SIM_TIME = 8 

 
  thrust_curve = np.array([
    [  0, 0.5,   1, 1.5,   2, 2.5,   3, 3.5,   4, 4.5,   5, 5.5,   6 ],
    [ 35,  15,  25,  12,  30,  15,  30,  12,  30,  12,  30,  12,   0 ]
  ])
  mass_curve = np.array([[0], [0]])
  motor = Motor(thrust_curve, mass_curve) 
 
  frame = Frame() 
  mount = MotorMount()
  mount.attach_motor(motor)

  components = {
    'frame': frame,
    'mount': mount
  }

  kinematics = Kinematics(components=components)
 
  software = FlightSoftware(FLIGHT_CONTROLLER_LOOP_TIME)
  
  rocket = Rocket(kinematics, software, mount)

  simulation = Simulation(rocket)
  simulation.run(SIM_TIME, 0.01)

  plot_1 = {
    'y': {
      'logname': 'cm_position',
      'ops': [lambda x : x[2]]
      },
    'x': {
      'logname': 'time',
      'ops': []
      }
    }
  plot_2 = {
    'y': {
      'logname': 'cm_position',
      'ops': [lambda x : x[0]]
    },
    'x': {
      'logname': 'time',
      'ops': []
    }
  }


  #simulation.plot(include=[plot_1, plot_2], in_terminal=False)
  simulation.animate()
