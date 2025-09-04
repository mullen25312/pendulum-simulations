#!/bin/bash

jupyter nbconvert ./rust-assisted/multi_link_pendulum.ipynb --to html --output ../docs/multi_link_pendulum.html --theme jupyterlab-night
jupyter nbconvert triple_pendulum_swing_up.ipynb --to html --output ./docs/triple_pendulum_swing_up.html --theme jupyterlab-night
jupyter nbconvert triple_pendulum_periodic_orbit.ipynb --to html --output ./docs/triple_pendulum_periodic_orbit.html --theme jupyterlab-night
