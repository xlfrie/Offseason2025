package frc.robot.subsystems.hardware.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.dyn4j.geometry.Vector2;

public interface ModuleIO {
  void setDriveVoltage(Voltage voltage);

  void setSteerVoltage(Voltage voltage);

  Voltage getDriveVoltage();

  Voltage getSteerVoltage();

  Rotation2d getSteerAngle();

  Angle getDriveWheelPosition();

  SwerveModuleState getState();

  SwerveModulePosition getPosition();

  void setSteerPID(double angle);

  void setDrivePID(double speed);

  void setDesiredState(double speed, Rotation2d angle);

  SwerveModuleState getDesiredState();

  void tickPID();

  String getModuleName();

  Vector2 getNormalRotationVec();

  default void telemetry() {
    SmartDashboard.putData(getModuleName(), sendableBuilder -> {
      sendableBuilder.setSmartDashboardType(getModuleName());

      sendableBuilder.addDoubleProperty("Drive Voltage", () -> getDriveVoltage().magnitude(), null);
      sendableBuilder.addDoubleProperty("Steer Voltage", () -> getSteerVoltage().magnitude(), null);

      sendableBuilder.addDoubleProperty("Desired Steer Angle", () -> {
        if (getDesiredState() != null) {
          return getDesiredState().angle.getDegrees();
        } else {
          return 0;
        }
      }, null);
      sendableBuilder.addDoubleProperty("Current Steer Angle",
          () -> MathUtil.angleModulus(getSteerAngle().getRadians()) / Math.PI * 180, null);
    });
  }
}
