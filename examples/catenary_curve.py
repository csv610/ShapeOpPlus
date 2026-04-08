#!/usr/bin/env python3
"""
ShapeOp Example: Catenary Curve
This example demonstrates:
1. Creating a line of points (a rope).
2. Adding EdgeStrain constraints to maintain length.
3. Fixing the two endpoints using Closeness constraints.
4. Using gravity as a force to "hang" the rope.
5. Solving for the static equilibrium shape.
"""

import os
import sys

# Ensure we can find the shapeopPython module
# Assumes we are running from the build/examples directory or have it in PYTHONPATH
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../libShapeOp/bindings/python')))

try:
    import shapeopPython
except ImportError:
    print("Error: could not import shapeopPython. Make sure it's built and in your PYTHONPATH.")
    sys.exit(1)

def main():
    num_points = 10
    # 1. Initialize points in a straight line
    # Points are stored in a flat array: [x0, y0, z0, x1, y1, z1, ...]
    p = shapeopPython.doubleArray(num_points * 3)
    for i in range(num_points):
        p[i*3 + 0] = float(i) # x
        p[i*3 + 1] = 0.0      # y
        p[i*3 + 2] = 0.0      # z

    s = shapeopPython.shapeop_create()
    shapeopPython.shapeop_setPoints(s, p, num_points)

    # 2. Add Edge constraints between consecutive points
    weight = 1.0
    for i in range(num_points - 1):
        ids = shapeopPython.intArray(2)
        ids[0] = i
        ids[1] = i + 1
        shapeopPython.shapeop_addConstraint(s, "EdgeStrain", ids, 2, weight)

    # 3. Fix the two endpoints
    # Left end at (0,0,0)
    id_left = shapeopPython.intArray(1)
    id_left[0] = 0
    shapeopPython.shapeop_addConstraint(s, "Closeness", id_left, 1, 10.0)

    # Right end at (5,0,0)
    id_right = shapeopPython.intArray(1)
    id_right[0] = num_points - 1
    id_right_constraint = shapeopPython.shapeop_addConstraint(s, "Closeness", id_right, 1, 10.0)
    
    # Update the target position of the right endpoint to (5,0,0)
    right_pos = shapeopPython.doubleArray(3)
    right_pos[0] = 5.0; right_pos[1] = 0.0; right_pos[2] = 0.0
    shapeopPython.shapeop_editConstraint(s, "Closeness", id_right_constraint, right_pos, 3)

    # 4. Add Gravity force
    grav = shapeopPython.doubleArray(3)
    grav[0] = 0.0; grav[1] = 0.0; grav[2] = -9.81
    shapeopPython.shapeop_addGravityForce(s, grav)

    # 5. Solve using dynamic initialization to enable forces
    # Parameters: masses, damping, timestep
    shapeopPython.shapeop_initDynamic(s, 1.0, 0.95, 0.1)
    
    print("Solving for Catenary shape...")
    # Running more steps to reach equilibrium
    for _ in range(20):
        shapeopPython.shapeop_solve(s, 10)

    # Get the results back
    shapeopPython.shapeop_getPoints(s, p, num_points)
    
    print("\nFinal Z-coordinates of the rope:")
    for i in range(num_points):
        print(f"Point {i}: Z = {p[i*3 + 2]:.3f}")

    shapeopPython.shapeop_delete(s)

if __name__ == "__main__":
    main()
