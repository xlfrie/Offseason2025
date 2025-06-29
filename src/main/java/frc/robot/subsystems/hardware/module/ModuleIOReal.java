package frc.robot.subsystems.hardware.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import org.dyn4j.geometry.Vector2;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.PhysicalRobotConstants.k_wheelCircumference;

public class ModuleIOReal implements ModuleIO {
  private final TalonFX driveMotorController;
  private final SparkMax steerMotorController;
  private final CANcoder CANCoder;

  private final Angle CANCoderOffset;

  // TODO this might be better of as a profiled PID controller
  private final PIDController steerPIDController;

  private final VelocityVoltage driveVelocityVoltage;

  private final int driveMotorID;
  private final int steerMotorID;
  private final int CANCoderID;

  private final SwerveModulePosition swerveModulePosition;
  private final SwerveModuleState desiredState;
  private final SwerveModuleState currentState;

  private final String moduleName;
  private final Vector2 unitRotationVec;

  // TODO move to constants
  private static final double driveGearRatio = 1 / 6.12;

  // TODO BEFORE TEST DRIVE - CHECK MOTOR ENCODER DIRECTIONS - CHECK CAN CODER DIRECTIONS
  // TODO Document this
  public ModuleIOReal(int driveMotorID, int steerMotorID, int CANCoderID, Angle CANCoderOffset,
      Vector2 physicalModulePosition, String moduleName) {

    this.driveMotorID = driveMotorID;
    this.steerMotorID = steerMotorID;
    this.CANCoderID = CANCoderID;
    this.CANCoderOffset = CANCoderOffset;
    this.moduleName = moduleName;


    // Drive motor config
    driveMotorController = new TalonFX(this.driveMotorID, "rio");

    TalonFXConfigurator driveMotorConfigurator = driveMotorController.getConfigurator();

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.RealRobotConstants.kPDrive;
    slot0Configs.kI = Constants.RealRobotConstants.kIDrive;
    slot0Configs.kD = Constants.RealRobotConstants.kDDrive;
    slot0Configs.kV = Constants.RealRobotConstants.kVDrive;
    slot0Configs.kS = Constants.RealRobotConstants.kSDrive;
    driveMotorConfigurator.apply(slot0Configs);

    driveVelocityVoltage = new VelocityVoltage(0);
    driveVelocityVoltage.Slot = 0;

    // Azimuth motor config
    steerMotorController = new SparkMax(this.steerMotorID, SparkMax.MotorType.kBrushless);
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    // TODO figure out this open loop ramp rate
    sparkMaxConfig.smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake)
        .openLoopRampRate(0.2);

    steerPIDController = new PIDController(Constants.RealRobotConstants.kPAzimuth,
        Constants.RealRobotConstants.kIAzimuth, Constants.RealRobotConstants.kDAzimuth);
    steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
    steerPIDController.setTolerance(0.05);

    // CANCoder config
    CANCoder = new CANcoder(this.CANCoderID, "rio");

    CANcoderConfiguration configuration = new CANcoderConfiguration();
    configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // TODO determine if this is correct
    configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    configuration.MagnetSensor.MagnetOffset = -this.CANCoderOffset.in(Rotations);

    CANCoder.getConfigurator().apply(configuration);

    // Swerve math setup
    swerveModulePosition = new SwerveModulePosition();
    desiredState = new SwerveModuleState(0, new Rotation2d());
    currentState = new SwerveModuleState(0, new Rotation2d());

    // Explanation for this is in sim implementation
    this.unitRotationVec = physicalModulePosition.copy().rotate(Math.PI / 2).getNormalized()
        .multiply(physicalModulePosition.getMagnitude());

    telemetry();
  }

  @Override
  public Voltage getDriveVoltage() {
    return driveMotorController.getMotorVoltage().getValue();
  }

  @Override
  public AngularVelocity getDriveWheelVelocity() {
    return driveMotorController.getVelocity().getValue().times(driveGearRatio);
  }

  @Override
  public Angle getDriveWheelPosition() {
    // TODO Ensure this math is correct
    return driveMotorController.getPosition().getValue().times(driveGearRatio);
  }

  @Override
  public void setDriveVoltage(Voltage voltage) {
    driveMotorController.setVoltage(voltage.in(Volts));
  }

  @Override
  public void setDrivePID(double speed) {
    driveVelocityVoltage.Velocity = speed;
    driveMotorController.setControl(driveVelocityVoltage);
  }

  @Override
  public Voltage getSteerVoltage() {
    // TODO look for better way of doing this
    // This may not be correct
    return Volts.of(steerMotorController.get() * RobotController.getBatteryVoltage());
  }

  @Override
  public Rotation2d getSteerAngle() {
    return new Rotation2d(CANCoder.getAbsolutePosition().getValue());
  }

  @Override
  public void setSteerVoltage(Voltage voltage) {
    steerMotorController.setVoltage(voltage);
  }

  @Override
  public void setSteerPID(double angle) {
    steerPIDController.setSetpoint(angle);
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
  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  @Override
  public void setDesiredState(LinearVelocity speed, Rotation2d angle) {
    if (angle != null)
      this.desiredState.angle = angle;

    setDrivePID(speed.in(MetersPerSecond) / k_wheelCircumference.in(Meters) * driveGearRatio);

    setSteerPID(desiredState.angle.getRadians());
  }

  @Override
  public void tickPID() {
    setSteerVoltage(Volts.of(steerPIDController.calculate(getSteerAngle().getRadians())));
  }

  @Override
  public String getModuleName() {
    return moduleName;
  }

  @Override
  public Vector2 getUnitRotationVec() {
    return unitRotationVec;
  }
}
