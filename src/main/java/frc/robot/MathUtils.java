package frc.robot;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class MathUtils {
  /**
   * In Java the default modulus operator will return a value that has the same sign as the
   * dividend, this function does not do that.
   *
   * @param dividend The dividend
   * @param divisor  The divisor
   * @return Modulus of the given parameters that does not inherit the sign of the dividend
   */
  public static double unsignedModulus(double dividend, double divisor) {
    return (dividend % divisor + divisor) % divisor;
  }

  /**
   * With angles there is two differences, this function finds the smallest one.
   * <p>
   * THIS FUNCTION IS SIGNED.
   *
   * @param angle1 The first angle
   * @param angle2 The second angle
   * @return The signed smallest difference between two angles
   */
  public static Angle subtractAngles(Angle angle1, Angle angle2) {
    Angle res = angle1.copy().minus(angle2);
    res = Degrees.of(unsignedModulus(res.in(Degrees) + 180, 360) - 180);

    return res;
  }
}
