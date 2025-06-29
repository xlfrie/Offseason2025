package frc.robot.subsystems.hardware.module;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import org.dyn4j.geometry.Vector2;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.PhysicalRobotConstants.k_wheelCircumference;

public class ModuleIOReal implements ModuleIO {
  private final TalonFX driveMotorController;
  private final SparkMax steerMotorController;
  private final CANcoder CANCoder;

  // TODO this might be better of as a profiled PID controller
  private final PIDController steerPIDController;

  private final int driveMotorID;
  private final int steerMotorID;
  private final int CANCoderID;

  private final SwerveModulePosition swerveModulePosition;
  private final SwerveModuleState desiredState;
  private final SwerveModuleState currentState;

  private final String moduleName;
  private final Vector2 normalRotationVector;

  private static final double driveGearRatio = 1;


  public ModuleIOReal(int driveMotorID, int steerMotorID, int CANCoderID,
      Vector2 physicalModulePosition, String moduleName) {
    this.driveMotorID = driveMotorID;
    this.steerMotorID = steerMotorID;
    this.CANCoderID = CANCoderID;

    this.moduleName = moduleName;

    this.normalRotationVector = physicalModulePosition.rotate(Math.PI / 2).getNormalized();

    swerveModulePosition = new SwerveModulePosition();
    desiredState = new SwerveModuleState(0, new Rotation2d());
    currentState = new SwerveModuleState(0, new Rotation2d());

    // TODO figure out why this said this was on a CANivore in the old code
    driveMotorController = new TalonFX(this.driveMotorID, "rio");
    steerMotorController = new SparkMax(this.steerMotorID, SparkMax.MotorType.kBrushless);

    steerPIDController = new PIDController(0.0075, 0, 0);

    steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
    steerPIDController.setTolerance(0.05);

    CANCoder = new CANcoder(this.CANCoderID, "rio");

    // TODO more config, pid, can coder, controllers, etc

    telemetry();
  }

  @Override
  public Voltage getDriveVoltage() {
    return driveMotorController.getMotorVoltage().getValue();
  }

  @Override
  public void setDriveVoltage(Voltage voltage) {
    driveMotorController.setVoltage(voltage.in(Volts));
  }

  @Override
  public Voltage getSteerVoltage() {
    // TODO look for better way of doing this
    return Volts.of(steerMotorController.get() * RobotController.getBatteryVoltage());
  }

  @Override
  public void setSteerVoltage(Voltage voltage) {
    steerMotorController.setVoltage(voltage);
  }

  @Override
  public Rotation2d getSteerAngle() {
    return new Rotation2d(CANCoder.getAbsolutePosition().getValue());
  }

  @Override
  public Angle getDriveWheelPosition() {
    // TODO Ensure this math is correct
    return driveMotorController.getPosition().getValue().times(driveGearRatio);
  }

  @Override
  public AngularVelocity getDriveWheelVelocity() {
    return null;
  }

  @Override
  public SwerveModuleState getState() {
    currentState.angle = this.getSteerAngle();

    currentState.speedMetersPerSecond = driveMotorController.getVelocity().getValue()
        .in(RadiansPerSecond) * k_wheelCircumference.in(Meters);

    return currentState;
  }

  @Override
  public SwerveModulePosition getPosition() {
    swerveModulePosition.angle = this.getSteerAngle();
    swerveModulePosition.distanceMeters =
        Rotations.ofBaseUnits(this.getDriveWheelPosition().baseUnitMagnitude())
            .magnitude() * k_wheelCircumference.magnitude() * driveGearRatio;
    return this.swerveModulePosition;
  }

  @Override
  public void setSteerPID(double angle) {
    steerPIDController.setSetpoint(angle);
  }

  @Override
  public void setDrivePID(double speed) {
    // driveMotorController.setControl(new VelocityVoltage());
  }

  @Override
  public void setDesiredState(LinearVelocity speed, Rotation2d angle) {
    if (angle != null)
      this.desiredState.angle = angle;
    this.desiredState.speedMetersPerSecond = speed.in(MetersPerSecond);

    setSteerPID(desiredState.angle.getRadians());
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  @Override
  public void tickPID() {
    // TODO this should eventually be using speed instead of voltage
    setSteerVoltage(Volts.of(steerPIDController.calculate(getSteerAngle().getRadians())));
  }

  @Override
  public String getModuleName() {
    return moduleName;
  }

  @Override
  public Vector2 getNormalRotationVec() {
    return normalRotationVector;
  }
}
