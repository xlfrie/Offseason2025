package frc.robot.subsystems.hardware.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.dyn4j.geometry.Vector2;

public class ModuleIODummy implements ModuleIO {
  private final SwerveModulePosition zeroPosition = new SwerveModulePosition(0, Rotation2d.kZero);
  private final SwerveModuleState zeroState = new SwerveModuleState(0, Rotation2d.kZero);
  private final Vector2 unitVector = new Vector2(0,1);

  @Override
  public void setDriveVoltage(Voltage voltage) {

  }

  @Override
  public void setSteerVoltage(Voltage voltage) {

  }

  @Override
  public Voltage getDriveVoltage() {
    return Units.Volts.zero();
  }

  @Override
  public Voltage getSteerVoltage() {
    return Units.Volts.zero();
  }

  @Override
  public Rotation2d getSteerAngle() {
    return Rotation2d.kZero;
  }

  @Override
  public Angle getDriveWheelPosition() {
    return Units.Radians.zero();
  }

  @Override
  public AngularVelocity getDriveWheelVelocity() {
    return Units.RadiansPerSecond.zero();
  }

  @Override
  public SwerveModuleState getState() {
    return zeroState;
  }

  @Override
  public SwerveModulePosition getPosition() {
    return zeroPosition;
  }

  @Override
  public void setSteerPID(double angle) {

  }

  @Override
  public void setDrivePID(double speed) {

  }

  @Override
  public void setDesiredState(LinearVelocity speed, Rotation2d angle) {

  }

  @Override
  public SwerveModuleState getDesiredState() {
    return zeroState;
  }

  @Override
  public void tickPID() {

  }

  @Override
  public String getModuleName() {
    // only used in telemetry which is overridden
    return null;
  }

  @Override
  public void telemetry() {
  }

  @Override
  public Vector2 getUnitRotationVec() {
    return unitVector;
  }
}
