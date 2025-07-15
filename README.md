# IPDA Tracker Implementation

A Python implementation of the Integrated Probabilistic Data Association (IPDA) algorithm for multi-target tracking in cluttered environments.

## Overview

The IPDA tracker is an advanced probabilistic data association technique that handles:
- Multiple target tracking
- Clutter rejection
- Track initialization and termination
- Uncertainty quantification through existence probabilities

This implementation provides a complete simulation environment with trajectory generation, clutter modeling, and visualization capabilities.

## Features

- **Multi-target tracking**: Track multiple objects simultaneously
- **Clutter handling**: Robust performance in environments with false alarms
- **Probabilistic data association**: Uses likelihood-based measurement-to-track assignment
- **Track lifecycle management**: Automatic track initialization, confirmation, and termination
- **Visualization**: Animated GIF generation showing tracking performance over time
- **Configurable parameters**: Adjustable detection probabilities, clutter rates, and gating thresholds

## Installation

### Requirements

```bash
pip install numpy scipy matplotlib imageio pillow
```

### Dependencies

- `numpy`: Numerical computations
- `scipy`: Statistical distributions and spatial operations
- `matplotlib`: Plotting and visualization
- `imageio`: GIF creation
- `PIL (Pillow)`: Image handling

## Usage

### Basic Example

```python
import numpy as np
from ipda_tracker import TrackerManager, generate_trajectories

# Generate synthetic trajectories
x_trajs, z_trajs = generate_trajectories(num_trajectories=5, num_points=100, seed=9)

# Initialize tracker manager
lambda_density = 0.01  # Clutter density
P_s = 0.6             # Survival probability
r_0 = 0.3             # Initial existence probability

manager = TrackerManager(lambda_density, P_s, r_0)

# Process measurements
for i in range(100):
    # Get measurements at time step i
    measurements = [z[i] for z in z_trajs]
    
    # Add clutter
    clutter = generate_clutter(lambda_clutter=50, measurement_space=space)
    all_measurements = np.vstack([measurements, clutter])
    
    # Update tracker
    manager.associate_and_update(all_measurements)
    manager.remove_tracks()
```

### Visualization

```python
# Create animated visualization
plot_tracks_gif(
    manager.track_history, 
    z_trajs=x_trajs, 
    measurement_space=measurement_space,
    clutter=clutter_hist,
    lambda_density=lambda_density,
    filename="tracking_results.gif"
)
```

## Algorithm Description

### IPDA Filter

The IPDA algorithm extends the Probabilistic Data Association (PDA) filter by incorporating:

1. **Existence Probability**: Each tracker maintains a probability `r` that the target exists
2. **Integrated Update**: Combines measurement association and existence estimation
3. **Gating**: Uses ellipsoidal gating based on Mahalanobis distance

### Key Components

#### State Model
- **State vector**: `[x, y, vx, vy]` (position and velocity)
- **Transition matrix**: Constant velocity model
- **Process noise**: Gaussian with configurable covariance

#### Measurement Model
- **Observation**: `[x, y]` (position only)
- **Measurement noise**: Gaussian with covariance `R`

#### Track Management
- **Initialization**: New tracks created from unassociated measurements
- **Confirmation**: Tracks confirmed when existence probability exceeds threshold
- **Termination**: Tracks deleted when existence probability drops below threshold

## Configuration Parameters

### System Parameters

```python
dt = 1.0              # Time step
q = np.sqrt(0.5)      # Process noise magnitude
r = 5.0               # Measurement noise standard deviation
```

### Tracker Parameters

```python
PD = 0.95             # Probability of detection
PG = 0.98             # Probability of gating
P_s = 0.6             # Survival probability
r_0 = 0.3             # Initial existence probability
```

### Thresholds

```python
threshold_termination = 0.2   # Existence prob. for track deletion
threshold_confirmation = 0.9  # Existence prob. for track confirmation
distance_threshold = 2.0      # Distance for tracker merging
```

## Classes and Functions

### Core Classes

#### `Tracker`
Individual tracker managing a single target hypothesis.

**Key Methods:**
- `predict()`: Kalman filter prediction step
- `update()`: IPDA update with multiple measurements
- `should_delete()`: Check termination conditions

#### `TrackerManager`
Manages multiple trackers and handles data association.

**Key Methods:**
- `associate_and_update()`: Measurement-to-track association
- `remove_tracks()`: Track lifecycle management
- `remove_redundant_trackers()`: Merge nearby trackers

### Utility Functions

#### `generate_trajectories()`
Generate synthetic target trajectories with process noise.

#### `plot_tracks_gif()`
Create animated visualization of tracking performance.

#### `ellipsoidal_gating()`
Implement ellipsoidal gating for measurement validation.

## Performance Tuning

### Clutter Density
- **Low clutter**: Reduce `lambda_density` for better performance
- **High clutter**: Increase gating threshold or adjust PD/PG values

### Track Sensitivity
- **Sensitive tracking**: Lower `threshold_termination`, increase `r_0`
- **Conservative tracking**: Higher `threshold_confirmation`

### Computational Efficiency
- **Gating**: Adjust `PG` to control computational load
- **Merging**: Tune `distance_threshold` for tracker consolidation

## Mathematical Foundation

### IPDA Update Equations

The IPDA filter updates both the state estimate and existence probability:

```
r_k = (r_{k-1} * L_k) / ((1 - r_{k-1}) + r_{k-1} * L_k)
```

Where `L_k` is the integrated likelihood considering all measurements in the validation gate.

### Gating
Measurements are validated using the chi-squared test:

```
d²_j = (z_j - ẑ)^T S^{-1} (z_j - ẑ) < γ
```

Where `γ` is the chi-squared threshold for the desired gating probability.





