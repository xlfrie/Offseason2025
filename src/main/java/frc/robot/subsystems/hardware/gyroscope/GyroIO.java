package frc.robot.subsystems.hardware.gyroscope;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {
  Rotation2d getRotation();
  AngularVelocity getAngularVelocity();
}
