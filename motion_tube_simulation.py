import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# Simulation parameters
dt = 0.1  # time step for simulation (s)
horizon = 1  # time horizon to evaluate each motion primitive (s)
dsample = 0.1  # spatial sampling interval along tube boundaries (m)
robot_radius = 0.2  # robot modeled as circle of this radius (m)
max_speed = 1.0  # maximum forward speed (m/s)
max_omega = np.pi/4  # maximum angular velocity (rad/s)

# Environment: list of circular obstacles (x, y, radius)
obstacles = [
    (3.0, 2.0, 0.5),
    (5.5, 3.5, 0.7),
    (7.0, 1.5, 0.6),
    (6.0, 5.0, 0.4),
    (2.0, 4.0, 0.5),
    (4.5, 5.5, 0.4)
]

# Start and goal positions
start_pos = np.array([1.0, 1.0])
start_theta = 0.0
goal = np.array([8.0, 6.0])
goal_tolerance = 0.2


def wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle+np.pi) % (2*np.pi) - np.pi


def generate_trajectory(x0, y0, theta0, v, w, horizon, dsample):
    """
    Generate boundary samples for a given motion primitive (v, w) over the horizon.
    Returns a list of boundary points [(x, y), ...].
    """
    points_center = []
    points_left = []
    points_right = []

    if abs(w) > 1e-6:
        radius = v/w
        theta_horizon = w*horizon
        arc_length = abs(radius*theta_horizon)
    else:
        arc_length = abs(v*horizon)

    # Number of samples along arc, at least 2
    n_samples = max(int(np.ceil(arc_length/dsample)), 2)

    for i in range(n_samples+1):
        t = (i/n_samples) * horizon
        if abs(w) > 1e-6: # circular motion
            x = x0 + (v/w)*(np.sin(theta0 + w*t) - np.sin(theta0))
            y = y0 - (v/w)*(np.cos(theta0 + w*t) - np.cos(theta0))
            theta = wrap_angle(theta0 + w*t)
        else: # straight motion
            x = x0 + v*t*np.cos(theta0)
            y = y0 + v*t*np.sin(theta0)
            theta = theta0
        points_center.append((x, y, theta))

        x_left = x - robot_radius*np.sin(theta)
        x_right = x + robot_radius*np.sin(theta)
        y_left = y + robot_radius*np.cos(theta)
        y_right = y - robot_radius*np.cos(theta)

        points_left.append((x_left, y_left))
        points_right.append((x_right, y_right))

    boundary_points = points_left + points_right[::-1]
    return boundary_points, points_center


def is_tube_collision_free(boundary_pts, obstacles):
    """
    Check if any boundary sample collides with obstacles.
    boundary_pts: list of (x, y) points.
    obstacles: list of (x_center, y_center, radius).
    Returns True if no collisions.
    """
    for (x, y) in boundary_pts:
        for (ox, oy, orad) in obstacles:
            if np.hypot(x-ox, y-oy) <= orad:
                return False
    return True


def plan_motion(current_pose):
    """
    Plan next motion by sampling candidate (v, w) pairs and evaluating free-space tubes.
    Returns selected (v, w).
    """
    x0, y0, theta0 = current_pose
    w_candidates = np.linspace(-max_omega, max_omega, 9)  # 9 samples from -max to max
    v = max_speed  # constant forward speed

    feasible = []
    for w in w_candidates:
        # Generate tube boundary samples for this (v, w)
        boundary_pts, _ = generate_trajectory(x0, y0, theta0, v, w, horizon, dsample)
        if is_tube_collision_free(boundary_pts, obstacles):
            # Evaluate cost: directness to goal after horizon
            if abs(w) > 1e-6:
                # compute endpoint of motion to estimate cost
                theta_end = wrap_angle(theta0 + w*horizon)
                x_end = x0 + (v/w) * (np.sin(theta_end) - np.sin(theta0))
                y_end = y0 - (v/w) * (np.cos(theta_end) - np.cos(theta0))
            else:
                x_end = x0 + v*horizon*np.cos(theta0)
                y_end = y0 + v*horizon*np.sin(theta0)
            # Cost: Euclidean distance from endpoint to goal
            cost = np.hypot(x_end-goal[0], y_end-goal[1])
            feasible.append((w, cost))

    if not feasible:
        return 0.0, 0.0

    # Select w with smallest cost
    w_selected, _ = min(feasible, key=lambda item: item[1])
    return max_speed, w_selected


# Simulation loop
pose = np.array([start_pos[0], start_pos[1], start_theta])  # [x, y, theta]
path = [pose.copy()]

max_steps = 200
for step in range(max_steps):
    # Check goal reached
    if np.hypot(pose[0]-goal[0], pose[1]-goal[1]) <= goal_tolerance:
        print(f"Goal reached at step {step}")
        break

    # Plan next motion
    v, w = plan_motion(pose)
    if v == 0 and w == 0:
        print("No collision-free motion found, stopping.")
        break

    # Apply motion for dt
    if abs(w) > 1e-6:
        radius = v/w
        dtheta = w*dt
        pose[0] += radius * (np.sin(pose[2]+dtheta) - np.sin(pose[2]))
        pose[1] -= radius * (np.cos(pose[2]+dtheta) - np.cos(pose[2]))
        pose[2] = wrap_angle(pose[2]+dtheta)
    else:
        pose[0] += v*dt*np.cos(pose[2])
        pose[1] += v*dt*np.sin(pose[2])

    path.append(pose.copy())

# Visualization
fig, ax = plt.subplots(figsize=(8, 6))

for (ox, oy, orad) in obstacles:
    circle = Circle((ox, oy), orad, color='gray', alpha=0.7)
    ax.add_patch(circle)

ax.plot(start_pos[0], start_pos[1], 'go', markersize=8, label='Start')
ax.plot(goal[0], goal[1], 'ro', markersize=8, label='Goal')

path = np.array(path)
ax.plot(path[:, 0], path[:, 1], 'b-', linewidth=2, label='Robot Path')

if len(path) > 0:
    last = path[-1]
    robot_circle = Circle((last[0], last[1]), robot_radius, color='blue', fill=False, linestyle='--')
    ax.add_patch(robot_circle)

ax.set_xlim(0, 10)
ax.set_ylim(0, 7)
ax.set_aspect('equal')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Free-Space Motion Tube Navigation Simulation')
ax.legend()
plt.grid(True)
plt.savefig('images/simulation.png')
plt.close()