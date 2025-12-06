package frc.robot.subsystems.hardware.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.SwerveDrive.SwerveDriveConfigurator;

import static edu.wpi.first.units.Units.*;

public class ModuleIOReal implements ModuleIO {
  private final SwerveDriveConfigurator.SwerveDriveRobotConstants robotConstants;
  private final SwerveDriveConfigurator.SwerveDriveModuleConstants moduleConstants;

  private final String moduleName;

  private final TalonFX driveMotorController;
  private final SparkMax steerMotorController;
  private final CANcoder CANCoder;

  // TODO this might be better of as a profiled PID controller
  private final PIDController steerPIDController;

  private final int steerVoltageCoefficient;

  private final VelocityVoltage driveVelocityVoltage;

  private final SwerveModulePosition swerveModulePosition;
  private final SwerveModuleState desiredState;
  private final SwerveModuleState currentState;

  private final Translation2d unitRotationVec;

  // TODO Document this
  public ModuleIOReal(SwerveDriveConfigurator.SwerveModuleCornerPosition cornerPosition,
      SwerveDriveConfigurator swerveDriveConfigurator) {
    this.robotConstants = swerveDriveConfigurator.swerveDriveRobotConstants;
    this.moduleConstants = swerveDriveConfigurator.getModuleConstants(cornerPosition);

    moduleName = moduleConstants.getModuleName();

    // Drive motor config
    driveMotorController = new TalonFX(moduleConstants.driveMotorID, "rio");

    TalonFXConfigurator driveMotorConfigurator = driveMotorController.getConfigurator();

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = moduleConstants.kPDrive;
    slot0Configs.kI = moduleConstants.kIDrive;
    slot0Configs.kD = moduleConstants.kDDrive;
    slot0Configs.kV = moduleConstants.kVDrive;
    slot0Configs.kS = moduleConstants.kSDrive;
    driveMotorConfigurator.apply(slot0Configs);
	  driveMotorConfigurator.apply(
			  new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(80)
					  .withSupplyCurrentLimit(60).withSupplyCurrentLimitEnable(true).withSupplyCurrentLowerLimit(40));
            driveMotorConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    // TODO verify falcon is ccw+
    // driveMotorConfigurator.apply(new MotorOutputConfigs().withInverted(
    //     Constants.RealRobotConstants.kDriveReversed ?
    //         InvertedValue.Clockwise_Positive :
    //         InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake));

    driveVelocityVoltage = new VelocityVoltage(0);
    driveVelocityVoltage.Slot = 0;

    // Azimuth motor config
    steerVoltageCoefficient = moduleConstants.azimuthReversed ? -1 : 1;

    steerMotorController =
        new SparkMax(moduleConstants.azimuthMotorID, SparkMax.MotorType.kBrushless);
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    // TODO figure out this open loop ramp rate
    sparkMaxConfig.smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake)
        .openLoopRampRate(0.2);

    steerMotorController.configure(sparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);


    steerPIDController = new PIDController(moduleConstants.kPAzimuth, moduleConstants.kIAzimuth,
        moduleConstants.kDAzimuth);
    steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
    steerPIDController.setTolerance(0.05);

    // CANCoder config
    CANCoder = new CANcoder(moduleConstants.CANCoderID, "rio");

    CANcoderConfiguration configuration = new CANcoderConfiguration();
    configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // TODO determine if this is correct
    configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    configuration.MagnetSensor.MagnetOffset = -moduleConstants.CANCoderOffset;

    CANCoder.getConfigurator().apply(configuration);

    // Swerve math setup
    swerveModulePosition = new SwerveModulePosition();
    desiredState = new SwerveModuleState(0, new Rotation2d());
    currentState = new SwerveModuleState(0, new Rotation2d());

    // Explanation for this is in sim implementation
    this.unitRotationVec = moduleConstants.physicalModulePosition.rotateBy(Rotation2d.kCCW_90deg);

    telemetry();
  }

  @Override
  public Voltage getDriveVoltage() {
    return driveMotorController.getMotorVoltage().getValue();
  }

  @Override
  public AngularVelocity getDriveWheelVelocity() {
    return driveMotorController.getVelocity().getValue().times(moduleConstants.driveGearRatio);
  }

  @Override
  public Angle getDriveWheelPosition() {
    // TODO Ensure this math is correct
    return driveMotorController.getPosition().getValue().times(moduleConstants.driveGearRatio);
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
    return Volts.of(steerMotorController.getAppliedOutput() * RobotController.getBatteryVoltage());
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
        .in(RadiansPerSecond) * robotConstants.wheelCircumference.in(Meters);

    return currentState;
  }

  @Override
  public SwerveModulePosition getPosition() {
    swerveModulePosition.angle = this.getSteerAngle();
    swerveModulePosition.distanceMeters =
        Rotations.ofBaseUnits(this.getDriveWheelPosition().baseUnitMagnitude())
            .magnitude() * robotConstants.wheelCircumference.magnitude() * moduleConstants.driveGearRatio;
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

    setDrivePID(speed.in(MetersPerSecond) / robotConstants.wheelCircumference.in(
        Meters) / moduleConstants.driveGearRatio);

    setSteerPID(desiredState.angle.getRadians());
  }

  @Override
  public void tickPID() {
    setSteerVoltage(Volts.of(
        steerVoltageCoefficient * steerPIDController.calculate(getSteerAngle().getRadians())));
  }

  @Override
  public String getModuleName() {
    return moduleName;
  }

  @Override
  public Translation2d getUnitRotationVec() {
    return unitRotationVec;
  }

  @Override
  public void telemetryHook(SendableBuilder sendableBuilder) {
    sendableBuilder.addDoubleProperty(moduleName + "-azimuthError",
        () -> steerPIDController.getError(), null);
    sendableBuilder.addDoubleProperty(moduleName + "-driveError",
        () -> driveMotorController.getVelocity().getValueAsDouble() - driveVelocityVoltage.Velocity,
        null);
  }

  @Override
  public AngularVelocity getSteerVelocity() {
    return CANCoder.getVelocity().getValue();
  }
}
