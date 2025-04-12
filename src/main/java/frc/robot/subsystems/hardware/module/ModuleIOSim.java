package frc.robot.subsystems.hardware.module;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.PhysicalRobotConstants.k_wheelCircumference;

public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation swerveModuleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController steerMotor;
  private final SwerveModulePosition swerveModulePosition;

  private double k_gearRatio = 1;


  //  TODO remove drive pid? idk if it's worth
  private final PIDController drivePID;
  private final PIDController steerPID;

  private final SwerveModuleState desiredState;

  private final String moduleName;
  private final Vector2 normalRotationVector;

  public ModuleIOSim(SwerveModuleSimulation swerveModuleSimulation, String moduleName,
      Vector2 physicalPosition) {
    this.swerveModuleSimulation = swerveModuleSimulation;

    this.swerveModulePosition = new SwerveModulePosition();

    this.driveMotor = swerveModuleSimulation.useGenericMotorControllerForDrive()
        .withCurrentLimit(Units.Amps.of(40));
    this.steerMotor =
        swerveModuleSimulation.useGenericControllerForSteer().withCurrentLimit(Units.Amps.of(20));

    this.drivePID = new PIDController(Constants.SimulatedControlSystemConstants.kPDrive,
        Constants.SimulatedControlSystemConstants.kIDrive,
        Constants.SimulatedControlSystemConstants.kDDrive);
    this.steerPID = new PIDController(Constants.SimulatedControlSystemConstants.kPSteer,
        Constants.SimulatedControlSystemConstants.kISteer,
        Constants.SimulatedControlSystemConstants.kDSteer);
    steerPID.enableContinuousInput(-Math.PI, Math.PI);

    this.moduleName = moduleName;
    this.normalRotationVector = physicalPosition.rotate(0).getNormalized();

    this.desiredState = new SwerveModuleState(0, new Rotation2d());

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

  //  TODO clamp any voltages sent to motors
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

  //  TODO ensure this value is not limited
  @Override
  public Angle getDriveWheelPosition() {
    return swerveModuleSimulation.getDriveWheelFinalPosition();
  }

  @Override
  public SwerveModuleState getState() {
    return swerveModuleSimulation.getCurrentState();
  }

  //  TODO if swerveposeestimator is off it may be because of this
  @Override
  public SwerveModulePosition getPosition() {
    swerveModulePosition.angle = this.getSteerAngle();
    swerveModulePosition.distanceMeters =
        Rotations.ofBaseUnits(this.getDriveWheelPosition().baseUnitMagnitude())
            .magnitude() * k_wheelCircumference.magnitude() * k_gearRatio;
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
  public void setDesiredState(double speed, Rotation2d angle) {
    if (angle != null)
      this.desiredState.angle = angle;
    this.desiredState.speedMetersPerSecond = speed;

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
      setDriveVoltage(Volt.of(desiredState.speedMetersPerSecond * 6));
    }
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
