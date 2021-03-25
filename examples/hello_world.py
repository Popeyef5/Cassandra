#!/usr/bin/env python3

if __name__ == '__main__':
  from cassandra.rockets import SimpleRocket
  from cassandra.simulation import Simulation

  SIM_TIME = 8 
  SIM_TIMESTEP = 0.01

  rocket = SimpleRocket()

  simulation = Simulation(rocket)
  simulation.run(SIM_TIME, SIM_TIMESTEP)

  simulation.animate()
