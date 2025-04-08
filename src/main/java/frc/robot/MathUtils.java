package frc.robot;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class MathUtils {
  public static double unsignedModulus(double num, double dividend) {
    return (num % dividend + dividend) % dividend;
  }

  public static Angle subtractAngles(Angle angle1, Angle angle2) {
    Angle res = angle1.copy().minus(angle2);
    res = Degrees.of(unsignedModulus(res.in(Degrees) + 180, 360) - 180);

    return res;
  }
}
