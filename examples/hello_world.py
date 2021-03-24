

if __name__ == '__main__':
  from cassandra.rocket import Rocket
  from cassandra.components import MotorMount, Frame
  from cassandra.motors import Motor
  from cassandra.physics import Kinematics 
  from cassandra.control import FlightSoftware
  from cassandra.simulation import Simulation
  import numpy as np

  FLIGHT_CONTROLLER_LOOP_TIME = 0.5
  SIM_TIME = 8 
  SIM_TIMESTEP = 0.1

 
  thrust_curve = np.array([
    [  0, 0.5,   1, 1.5,   2, 2.5,   3, 3.5,   4, 4.5,   5, 5.5,   6 ],
    [ 35,  15,  25,  12,  30,  15,  30,  12,  30,  12,  30,  12,   0 ]
  ])
  mass_curve = np.array([[0], [0]])
  motor = Motor(thrust_curve, mass_curve) 
 
  frame = Frame() 
  mount = MotorMount()
  mount.attach_motor(motor)

  rocket_components = {
    'frame': frame,
    'mount': mount
  }

  kinematics = Kinematics(components=rocket_components)
 
  software = FlightSoftware(FLIGHT_CONTROLLER_LOOP_TIME)
  
  rocket = Rocket(kinematics, software, mount)

  simulation = Simulation(rocket)
  simulation.run(SIM_TIME, SIM_TIMESTEP)

  simulation.animate()
