package frc.robot.subsystems.hardware.module;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.*;

public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation swerveModuleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController steerMotor;
  private final SwerveModulePosition swerveModulePosition;

  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController drivePID;
  private final PIDController steerPID;

  private final SwerveModuleState desiredState;

  private final String moduleName;
  private final Vector2 unitRotationVec;

  private final Distance simulatedCircumference;

  public ModuleIOSim(SwerveModuleSimulation swerveModuleSimulation, String moduleName,
      Vector2 physicalPosition) {
    this.swerveModuleSimulation = swerveModuleSimulation;

    this.swerveModulePosition = new SwerveModulePosition();

    this.driveMotor = swerveModuleSimulation.useGenericMotorControllerForDrive()
        .withCurrentLimit(Units.Amps.of(80));
    this.steerMotor =
        swerveModuleSimulation.useGenericControllerForSteer().withCurrentLimit(Units.Amps.of(20));

    this.driveFeedforward =
        new SimpleMotorFeedforward(Constants.SimulatedControlSystemConstants.kSDrive,
            Constants.SimulatedControlSystemConstants.kVDrive,
            Constants.SimulatedControlSystemConstants.kADrive);
    this.drivePID = new PIDController(Constants.SimulatedControlSystemConstants.kPDrive,
        Constants.SimulatedControlSystemConstants.kIDrive,
        Constants.SimulatedControlSystemConstants.kDDrive);
    this.steerPID = new PIDController(Constants.SimulatedControlSystemConstants.kPSteer,
        Constants.SimulatedControlSystemConstants.kISteer,
        Constants.SimulatedControlSystemConstants.kDSteer);
    steerPID.enableContinuousInput(-Math.PI, Math.PI);

    this.moduleName = moduleName;

    // This unit is 1 rad / sec
    // Proof: assume the wheel moves along a circle about the robot center. Length of an arc is
    // radius times angle in radians so the length of the arc is the distance of the module to
    // the center times one radian 
    this.unitRotationVec = physicalPosition.copy().rotate(Math.PI / 2).getNormalized()
        .multiply(physicalPosition.getMagnitude());

    this.desiredState = new SwerveModuleState(0, new Rotation2d());

    this.simulatedCircumference = swerveModuleSimulation.config.WHEEL_RADIUS.times(2 * Math.PI);

    telemetry();
  }


  @Override
  public void setDriveVoltage(Voltage voltage) {
    this.driveMotor.requestVoltage(voltage);
  }

  @Override
  public void setSteerVoltage(Voltage voltage) {
    this.steerMotor.requestVoltage(voltage);
  }

  // TODO clamp any voltages sent to motors
  @Override
  public Voltage getDriveVoltage() {
    return this.driveMotor.getAppliedVoltage();
  }

  @Override
  public Voltage getSteerVoltage() {
    return this.steerMotor.getAppliedVoltage();
  }

  @Override
  public Rotation2d getSteerAngle() {
    return swerveModuleSimulation.getSteerAbsoluteFacing();
  }

  @Override
  public Angle getDriveWheelPosition() {
    return swerveModuleSimulation.getDriveWheelFinalPosition();
  }

  @Override
  public AngularVelocity getDriveWheelVelocity() {
    return swerveModuleSimulation.getDriveWheelFinalSpeed();
  }

  @Override
  public SwerveModuleState getState() {
    return swerveModuleSimulation.getCurrentState();
  }

  @Override
  public SwerveModulePosition getPosition() {
    swerveModulePosition.angle = this.getSteerAngle();
    swerveModulePosition.distanceMeters =
        this.getDriveWheelPosition().in(Rotations) * simulatedCircumference.in(Meters);
    return this.swerveModulePosition;
  }

  @Override
  public void setSteerPID(double angle) {
    steerPID.setSetpoint(angle);
  }

  @Override
  public void setDrivePID(double speed) {
    drivePID.setSetpoint(speed);
  }


  @Override
  public void setDesiredState(LinearVelocity speed, Rotation2d angle) {
    if (angle != null)
      this.desiredState.angle = angle;
    this.desiredState.speedMetersPerSecond = speed.in(MetersPerSecond);
    setDrivePID(desiredState.speedMetersPerSecond);

    setSteerPID(desiredState.angle.getRadians());
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  @Override
  public void tickPID() {
    setSteerVoltage(Volts.of(steerPID.calculate(getSteerAngle().getRadians())));
    if (desiredState != null) {

      setDriveVoltage(Volts.of(drivePID.calculate(
          getDriveWheelVelocity().in(RotationsPerSecond) * simulatedCircumference.in(
              Meters)) + driveFeedforward.calculate(desiredState.speedMetersPerSecond)));
    }
  }

  @Override
  public String getModuleName() {
    return moduleName;
  }

  @Override
  public Vector2 getUnitRotationVec() {
    return unitRotationVec;
  }


  @Override
  public void telemetryHook(SendableBuilder sendableBuilder) {
    sendableBuilder.addDoubleProperty(getModuleName() + "-dError", () -> drivePID.getError(), null);
    sendableBuilder.addDoubleProperty(getModuleName() + "-dDesired_speed",
        () -> desiredState.speedMetersPerSecond, null);
    sendableBuilder.addDoubleProperty(getModuleName() + "-dReal_speed",
        () -> getDriveWheelVelocity().in(RotationsPerSecond) * simulatedCircumference.in(Meters),
        null);
    sendableBuilder.addDoubleProperty(getModuleName()+"dacc", () -> RobotContainer.swerveDriveSimulation.getForce().getMagnitude()/Constants.PhysicalRobotConstants.kMass.in(Kilogram), null);
  }
}
