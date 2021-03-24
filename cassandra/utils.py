import numpy as np

def skew_symmetric(v):
  """
  Returns the skew symmetric matrix [v] from vector `v`.\n
  [v] acts on vectors w such that [v].w = v x w
 
  Parameters
  ----------
  v : array_like
    The 3x1 vector from which to construct the skew-symmetric matrix.

  Returns
  -------
  [v] : ndarray
    The 3x3 skew-symmetric matrix associated to `v`.
  """

  return np.array([
    [  0  , -v[2],  v[1]],
    [ v[2],   0  , -v[0]],
    [-v[1],  v[0],   0  ]
  ]) 

def constrain(x, x_max, x_min=None):
  """
  Constrans a given number between a min and max value.

  Parameters
  ----------
  x : scalar
    The number to constrain.
  x_max : scalar
    The upper bound for the constrain.
  x_min : scalar, default=None
    The lower bound for the constrain. If None, defaults to -`x_max`.

  Returns
  -------
  scalar
    The constrained number. 
  """
 
  if x_min is None:
    x_min = -x_max 
  return min(max(x, x_min), x_max) 


