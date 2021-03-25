# Cassandra

### About

According to Greek mythology, Cassandra was a Trojan priestess simultaneously blessed with the gift of providence and cursed with the fact that no one was to believe her. It is then fitting that a simple simulation that attempts to model such a complex setting carries her name. This Cassandra, however, wants to break free from her curse. 

Currently written for enhanced readability, Cassandra allows from plug-and-play options to full customization to the deepest level.

Eventually, Cassandra will foresee all TVC powered flights, even those which are not mentioned to her. Until then, there's much to do.

### The bigger picture

Cassandra is part of the Hermes Project, an initiative whose goal is to create open-source reliable model rockets. There is so much to do besides coding: CAD design, Data analytics, Graphic design. If you want to know more and/or feel like you could help in any way, come say hi!

[![Widget for the Discord API guild](https://discord.com/api/guilds/824454398070882334/widget.png?style=banner2)](https://discord.gg/jEPMA4SuCB)

### Instalation

For now, until we get Cassandra on PyPi, do:

```bash
pip3 install git+https://github.com/Popeyef5/Cassandra.git --upgrade
```

### Example

A simple "hello_rocket" example:

```python
from cassandra.rockets import SimpleRocket
from cassandra.simulation import Simulation

SIM_TIME = 5
SIM_TIMESTEP = 0.01

rocket = SimpleRocket()

simulation = Simulation(rocket)
simulation.run(SIM_TIME, SIM_TIMESTEP)
simulation.animate()
```

To get a sample animation right away:

```bash
python examples/hello_world.py
```

### Simulation

UI in development. In the meantime, here's a low FPS gif for you:

<p align="center">
  <img src="https://raw.githubusercontent.com/Popeyef5/Cassandra/master/docs/images/early_sim.gif">
</p> 

### TODO

Although there is plenty to do and everything is welcome, if you want to contribute but don't know how to, here are a couple of ideas that would come in handy:

* A logo is always nice :)
* Tests (pretty please?)
* GitHub actions
* Improve UI
* Improve animation

