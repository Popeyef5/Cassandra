#!/usr/bin/env python3

if __name__ == '__main__':
  from cassandra.rockets import RandomRocket
  from cassandra.simulation import Simulation
  from cassandra.physics.integrators import RK4

  SIM_TIME = 8 
  SIM_TIMESTEP = 0.01

  rocket = RandomRocket()
  integrator = RK4()

  simulation = Simulation(rocket, integrator)
  simulation.run(SIM_TIME, SIM_TIMESTEP)
  simulation.animate()
