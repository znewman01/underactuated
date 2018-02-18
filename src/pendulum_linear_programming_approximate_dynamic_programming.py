
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

from pydrake.all import (DiagramBuilder, SignalLogger, Simulator, VectorSystem)
from pydrake.examples.pendulum import PendulumPlant
from pydrake.systems.controllers import (
    DynamicProgrammingOptions, LinearProgrammingApproximateDynamicProgramming)
from underactuated import PendulumVisualizer


plant = PendulumPlant()
simulator = Simulator(plant)
options = DynamicProgrammingOptions()


def min_time_cost(context):
    x = context.get_continuous_state_vector().CopyToVector()
    x[0] = x[0] - math.pi
    if x.dot(x) < .05:
        return 0.
    return 1.


def quadratic_regulator_cost(context):
    x = context.get_continuous_state_vector().CopyToVector()
    x[0] = x[0] - math.pi
    u = plant.EvalVectorInput(context, 0).CopyToVector()
    return 2*x.dot(x) + u.dot(u)


if (False):
    cost_function = min_time_cost
    input_limit = 1.
    options.convergence_tol = 0.001
else:
    cost_function = quadratic_regulator_cost
    input_limit = 3.
    options.convergence_tol = 0.1

qbins = np.linspace(0., 2. * math.pi, 21)
qdotbins = np.linspace(-10., 10., 21)
state_grid = [set(qbins), set(qdotbins)]
options.state_indices_with_periodic_boundary_conditions = {0}

input_grid = [set(np.linspace(-input_limit, input_limit, 9))]
timestep = 0.01

options.discount_factor = .99
cost_to_go = LinearProgrammingApproximateDynamicProgramming(simulator,
                                                            cost_function,
                                                            state_grid,
                                                            input_grid,
                                                            timestep, options)

[Q, Qdot] = np.meshgrid(qbins, qdotbins)
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_xlabel("q")
ax.set_ylabel("qdot")
J = np.reshape(cost_to_go, Q.shape)
surf = ax.plot_surface(Q, Qdot, J, rstride=1, cstride=1,
                       cmap=cm.jet)
plt.show()
