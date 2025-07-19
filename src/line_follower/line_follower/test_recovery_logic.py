from collections import deque
import numpy as np

# Simulated X positions of the line's centroid in the image
# Values move left over time, then the line disappears (None)
contour_centroids = [320, 300, 270, 230, 190, 150, None, None, None]

# Image width (simulate 640x480 image)
width = 640

# Recent angular velocities and direction decision
recent_directions = deque(maxlen=5)
recovery_direction = 0.0

# Process each centroid step
for i, cx in enumerate(contour_centroids):
    print(f"\nFrame {i+1}")
    if cx is not None:
        error = cx - width // 2
        angular_z = -0.002 * error
        recent_directions.append(angular_z)

        print(f"  Centroid X: {cx}, Error: {error}, Angular Z: {angular_z:.3f}")

        # Decide turn trend
        if len(recent_directions) >= 3:
            signs = [np.sign(a) for a in recent_directions]
            if all(s == signs[0] for s in signs):  # Consistent direction
                recovery_direction = sum(recent_directions) / len(recent_directions)
                print(f"  📈 Trend detected: {'Left' if recovery_direction > 0 else 'Right'}")
    else:
        print("  ❌ Line lost.")
        print(f"  🔄 Recovering with angular Z: {recovery_direction:.3f}")
