# pendulum-simulations

These jupyter notebooks were used for the research shown in the paper

> [1] Benjamin Jahn, Lars Watermann, Johann Reger: *On the design of stable periodic orbits of a triple pendulum on a cart with experimental validation* (<https://doi.org/10.1016/j.automatica.2020.109403>)

The paper presents the design and implementation of a periodic orbit maneuver of a triple pendulum on a cart by means of inversion-based feedforward control design for finite-time transitions. Originally, all calculations have been done using a well known commercial numerical computing environment in the past. The main goal of these notebooks is to translate all involved steps to python and its amazing libraries (numpy, sympy, scipy, plotly, matplotlib).

## Notebooks

- **[multi_link_pendulum.ipynb:](https://nbviewer.org/github/mullen25312/pendulum-simulations/blob/master/multi_link_pendulum.ipynb)** Symbolically derives the dynamics of a multi-link pendulum with adjustable number of links. The multi-link pendulum is then simulated for some example parameters and the solution is shown in an animation.

- **[triple_pendulum_swing_up.ipynb:](https://nbviewer.org/github/mullen25312/pendulum-simulations/blob/master/triple_pendulum_swing_up.ipynb)** After symbolic derivation of the dynamics of a triple pendulum, the feedfoward for a swing-up maneuver is numerically computed using scipy's boundary value problem solver. Additionally, a state feedback is designed based on the dynamics linearized around the feedforward trajectory. Finally, an authentic simulation including parameter deviations and process noise is done and the solution is shown in an animation.

- **[triple_pendulum_periodic_orbit.ipynb:](https://nbviewer.org/github/mullen25312/pendulum-simulations/blob/master/triple_pendulum_periodic_orbit.ipynb)** Similar to the previous notebook, except for maneuver containing a periodic orbit. This periodic orbit is a circular motion of the third link while the first two links stay close to their upright configuration.

## Rendering

Unfortunately, Github's document view is not capable of showing rendered outputs. It is recommended to view these notebooks in nbviewer using the following link:

> <https://nbviewer.org/github/mullen25312/pendulum-simulations/tree/master/>
