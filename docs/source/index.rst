Gymnasium-JSBSim Documentation
================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   modules

Welcome to Gymnasium-JSBSim
----------------------------

Gymnasium-JSBSim provides reinforcement learning environments for flight control
using JSBSim flight dynamics model and the Gymnasium API.

Features
--------

* Multiple aircraft models (Cessna 172P, A320, F-15)
* Configurable flight tasks (heading control, turn heading control)
* Reward shaping options
* FlightGear visualization support
* Compatible with Gymnasium API

Quick Start
-----------

.. code-block:: python

   import gymnasium as gym
   from gymnasium_jsbsim import Envs

   # Create an environment
   env = gym.make(Envs.heading_control_cessna172P_standard_nofg)
   
   # Run a simple episode
   observation, info = env.reset()
   for _ in range(100):
       action = env.action_space.sample()
       observation, reward, terminated, truncated, info = env.step(action)
       if terminated or truncated:
           break
   env.close()

API Reference
-------------

See the :doc:`modules` section for detailed API documentation.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
