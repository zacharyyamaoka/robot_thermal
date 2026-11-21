"""robot_thermal: thermally limited servoing — burst budgets, lumped thermal models and observers for servo drives.

WHY nothing is imported here: `robot_thermal.burst` and `robot_thermal.thermal` are standard-library only, so a
rig's control loop can import them without numpy; importing a heavy module from this file would break that.
"""

__version__ = "0.1.0"
