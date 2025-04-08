package frc.robot.subsystems.hardware.gyroscope;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public Rotation2d getRotation() {
    return gyroSimulation.getGyroReading();
  }

  @Override
  public AngularVelocity getAngularVelocity() {
    return gyroSimulation.getMeasuredAngularVelocity();
  }
}
