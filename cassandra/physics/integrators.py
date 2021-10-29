class Integrator:
  """
  Base class for numerical integrators.
  """
  def step(self, y, f, h, **kwargs):
    raise NotImplementedError


class Euler(Integrator):
  """
  Euler integration method.
  For more information check out https://en.wikipedia.org/wiki/Euler_method.
  """
  def step(self, y, f, h, **kwargs):
    return y + f(y, **kwargs) * h


class RK4(Integrator):
  """
  Runge-Kutta Order 4 integration method.
  For more information check out https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods.
  """
  def step(self, y, f, h, **kwargs):
    k1 = f(y, **kwargs)
    k2 = f(y + 0.5 * k1 * h, **kwargs)
    k3 = f(y + 0.5 * k2 * h, **kwargs)
    k4 = f(y + k3 * h, **kwargs)
    return y + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6


