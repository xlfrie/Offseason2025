package frc.robot.subsystems.hardware.gyroscope;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOPigeon2 implements GyroIO {
  private Pigeon2 pigeon;

  public GyroIOPigeon2(int id) {
    pigeon = new Pigeon2(id);
  }

  @Override
  public Rotation2d getRotation() {
    return pigeon.getRotation2d();
  }

  @Override
  public AngularVelocity getAngularVelocity() {
    // TODO this value must be checked to be correct
    return pigeon.getAngularVelocityZWorld().getValue();
  }
}
