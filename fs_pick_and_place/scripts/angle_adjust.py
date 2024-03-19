import math

def convert_rotation(angle):
  """
  Converts a cube's rotation angle in 2D to a value between 0 and 45 degrees.

  Args:
      angle: The rotation angle in degrees (or radians).

  Returns:
      The converted angle between 0 and 45 degrees (or pi/4 radians).
  """
  # Normalize the angle to be between 0 and 360 degrees (or 2*pi radians)
  angle = angle % 360

  # Because the cube is symmetrical, we only care about the first quadrant (0 to 90 degrees)
  if angle > 90:
    angle = 180 - angle  # Reflect the angle to the first quadrant

  # Adjust the angle to be between 0 and 45 degrees (or pi/4 radians)
  converted_angle = min(angle, 90 - angle)

  return converted_angle * math.pi / 180  # Convert to radians if needed

# Example usage:
original_angle = 81  # Degrees
converted_angle = convert_rotation(original_angle)
print(f"Original angle: {original_angle} degrees")
print(f"Converted angle: {converted_angle:.4f} radians (or {converted_angle * 180 / math.pi:.4f} degrees)")
