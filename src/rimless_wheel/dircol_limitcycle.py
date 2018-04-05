import math
import numpy as np
import matplotlib.pyplot as plt

from pydrake.examples.pendulum import (PendulumPlant, PendulumParams, PendulumState)
from pydrake.examples.rimless_wheel import RimlessWheelParams
from pydrake.all import (DirectCollocation, PiecewisePolynomial,
                         SolutionResult)

plant = PendulumPlant()
context = plant.CreateDefaultContext()

rw_params = RimlessWheelParams()
alpha = math.PI / params.number_of_spokes()
pendulum_params = context.get_mutable_parameters(0)
pendulum_params.set_mass(rw_params.mass())
pendulum_params.set_length(rw_params.length())
pendulum_params.set_damping(0.)

dircol = DirectCollocation(plant, context, num_time_samples=11,
                           minimum_timestep=0.1, maximum_timestep=0.5)

dircol.AddEqualTimeIntervalsConstraints()

dircol.AddBoundingBoxConstraint(dircol.initial_state()[0] == math.PI+rw_params.slope()-alpha)

dircol.AddBoundingBoxConstraint(dircol.final_state()[0] == math.PI+rw_params.slope()+alpha)

result = dircol.Solve()
assert(result == SolutionResult.kSolutionFound)

x_trajectory = dircol.ReconstructStateTrajectory()

x_knots = np.hstack([x_trajectory.value(t) for t in
                     np.linspace(x_trajectory.start_time(),
                                 x_trajectory.end_time(), 100)])
plt.figure()
plt.plot(x_knots[0, :], x_knots[1, :])

plt.show()
