package frc.robot.subsystems.hardware.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.dyn4j.geometry.Vector2;

public interface ModuleIO {
  /**
   * Set voltage supplied to drive motor
   *
   * @param voltage Voltage to be supplied to drive motor
   */
  void setDriveVoltage(Voltage voltage);


  /**
   * Set voltage supplied to steer motor
   *
   * @param voltage Voltage to be supplied to steer motor
   */
  void setSteerVoltage(Voltage voltage);

  /**
   * @return Voltage that is applied to the drive motor
   */
  Voltage getDriveVoltage();

  /**
   * @return Voltage that is applied to the steer motor
   */
  Voltage getSteerVoltage();

  /**
   * @return The direction the wheel is facing
   */
  Rotation2d getSteerAngle();

  /**
   * @return The rotation of the wheel (this can be greater than one rotation)
   */
  Angle getDriveWheelPosition();

  AngularVelocity getDriveWheelVelocity();

  /**
   * @return This module's {@link SwerveModuleState}
   */
  SwerveModuleState getState();

  /**
   * @return This module's {@link SwerveModulePosition}
   */
  SwerveModulePosition getPosition();


  /**
   * @param angle The new setpoint for the steer PID
   */
  void setSteerPID(double angle);

  /**
   * This PID is not currently used
   *
   * @param speed The new setpoint for the drive PID
   */
  void setDrivePID(double speed);


  /**
   * @param speed The desired state for the wheel's speed
   * @param angle The desired angle for the wheel to face
   */
  void setDesiredState(LinearVelocity speed, Rotation2d angle);

  /**
   * @return The module's desired states
   */
  SwerveModuleState getDesiredState();

  /**
   * This function is needed if the PID is implemented in software. This should be used to set motor
   * outputs from the PID.
   */
  void tickPID();

  /**
   * @return Module's name
   */
  String getModuleName();

  /**
   * @return Vector that when set as the desired state for this module will rotate the
   * robot without translation at 1 rad / sec
   */
  Vector2 getUnitRotationVec();

  default void telemetryHook(SendableBuilder sendableBuilder) {

  }

  /**
   * Publishes telemetry for this module into the SmartDashboard
   */
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

      telemetryHook(sendableBuilder);
    });
  }
}
