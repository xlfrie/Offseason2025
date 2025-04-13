package frc.robot.subsystems.hardware.gyroscope;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {
  /**
   * @return The estimated rotation of the current robot from the gyro
   */
  Rotation2d getRotation();

  /**
   * @return The sensed angular velocity by the gyro
   */
  AngularVelocity getAngularVelocity();
}
