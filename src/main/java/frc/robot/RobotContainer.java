// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveDrive.DefaultJoystickCommand;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.subsystems.hardware.Intake;
import frc.robot.subsystems.hardware.gyroscope.GyroIOPigeon2;
import frc.robot.subsystems.hardware.gyroscope.GyroIOSim;
import frc.robot.subsystems.hardware.module.ModuleIOReal;
import frc.robot.subsystems.hardware.module.ModuleIOSim;
import frc.robot.subsystems.hardware.vision.VisionIO;
import frc.robot.subsystems.hardware.vision.VisionIOSim;
import frc.robot.utilities.controller.Controller;
import frc.robot.utilities.controller.DualShock4Controller;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.PhysicalRobotConstants.kDriveBaseLength;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private SwerveDrive m_swerveDrive;
  private Controller controller;
  private Intake intake;

  public static SwerveDriveSimulation swerveDriveSimulation;

  public static VisionIO visionIO;

  private static final double k_driveBaseLengthMeters = kDriveBaseLength.in(Meters);

  public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(-k_driveBaseLengthMeters / 2, -k_driveBaseLengthMeters / 2),
      new Translation2d(-k_driveBaseLengthMeters / 2, k_driveBaseLengthMeters / 2),
      new Translation2d(k_driveBaseLengthMeters / 2, -k_driveBaseLengthMeters / 2),
      new Translation2d(k_driveBaseLengthMeters / 2, k_driveBaseLengthMeters / 2));

  /**
   * Registers all important robot code, e.g. swerve, path planner, controls
   */
  public RobotContainer() {
    if (Robot.isReal()) {
      // Real drive train
      m_swerveDrive = new SwerveDrive(new GyroIOPigeon2(Constants.RealRobotConstants.kPigeon2ID),
          new ModuleIOReal(Constants.RealRobotConstants.kFLDriveMotorID,
              Constants.RealRobotConstants.kFLAzimuthMotorID,
              Constants.RealRobotConstants.kFLCANCoderID,
              Radians.of(Constants.RealRobotConstants.kFLCANCoderOffset),
              new Vector2(-k_driveBaseLengthMeters / 2, k_driveBaseLengthMeters / 2), "Front Left"),
          new ModuleIOReal(Constants.RealRobotConstants.kFRDriveMotorID,
              Constants.RealRobotConstants.kFRAzimuthMotorID,
              Constants.RealRobotConstants.kFRCANCoderID,
              Radians.of(Constants.RealRobotConstants.kFRCANCoderOffset),
              new Vector2(k_driveBaseLengthMeters / 2, k_driveBaseLengthMeters / 2), "Front Right"),
          new ModuleIOReal(Constants.RealRobotConstants.kBLDriveMotorID,
              Constants.RealRobotConstants.kBLAzimuthMotorID,
              Constants.RealRobotConstants.kBLCANCoderID,
              Radians.of(Constants.RealRobotConstants.kBLCANCoderOffset),
              new Vector2(-k_driveBaseLengthMeters / 2, -k_driveBaseLengthMeters / 2), "Back Left"),
          new ModuleIOReal(Constants.RealRobotConstants.kBRDriveMotorID,
              Constants.RealRobotConstants.kBRAzimuthMotorID,
              Constants.RealRobotConstants.kBRCANCoderID,
              Radians.of(Constants.RealRobotConstants.kBRCANCoderOffset),
              new Vector2(k_driveBaseLengthMeters / 2, -k_driveBaseLengthMeters / 2),
              "Back Right"));
      intake = new Intake();
      controller = new DualShock4Controller(Constants.OperatorConstants.kDriverControllerPort);
    } else {
      visionIO = new VisionIOSim();

      // Simulation drive train

      // TODO add constant for drive base length
      swerveDriveSimulation = new SwerveDriveSimulation(
          DriveTrainSimulationConfig.Default().withGyro(COTS.ofPigeon2())
              .withRobotMass(Units.Pound.of(78)).withSwerveModule(
                  COTS.ofMark4i(DCMotor.getFalcon500(1), DCMotor.getNEO(1),
                      COTS.WHEELS.COLSONS.cof, 3))
              .withTrackLengthTrackWidth(kDriveBaseLength, kDriveBaseLength),
          new Pose2d(2, 7, Rotation2d.kZero));

      // TODO change this to not assume square drivebase
      m_swerveDrive = new SwerveDrive(new GyroIOSim(swerveDriveSimulation.getGyroSimulation()),
          new ModuleIOSim(swerveDriveSimulation.getModules()[0], "Front Left",
              new Vector2(-k_driveBaseLengthMeters / 2, k_driveBaseLengthMeters / 2)),
          new ModuleIOSim(swerveDriveSimulation.getModules()[1], "Front Right",
              new Vector2(k_driveBaseLengthMeters / 2, k_driveBaseLengthMeters / 2)),
          new ModuleIOSim(swerveDriveSimulation.getModules()[2], "Back Left",
              new Vector2(-k_driveBaseLengthMeters / 2, -k_driveBaseLengthMeters / 2)),
          new ModuleIOSim(swerveDriveSimulation.getModules()[3], "Back Right",
              new Vector2(k_driveBaseLengthMeters / 2, -k_driveBaseLengthMeters / 2)));

      SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
      controller = new DualShock4Controller(Constants.OperatorConstants.kDriverControllerPort);
    }
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link
   * edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings() {
    if (Robot.isReal())
    {
      controller.right1().onTrue(intake.reverse());
      controller.right1().onFalse(intake.idle());
  
      controller.cross().onTrue(intake.normal());
      controller.cross().onFalse(intake.idle());
  
      controller.circle().onTrue(intake.fast());
      controller.circle().onFalse(intake.idle());
    }
  }

  /**
   * Sets the controller as the default movement command for swerve.
   */
  public void bindJoystickCommand() {
    m_swerveDrive.setDefaultCommand(
        new DefaultJoystickCommand(controller::getLeftX, controller::getLeftY,
            controller::getRightX, m_swerveDrive));
  }

  public void zeroHeading() {
    m_swerveDrive.zeroHeading();
  }

  /**
   * Removes the controller from being used for movement.
   */
  public void unbindJoystick() {
    m_swerveDrive.removeDefaultCommand();
  }

  /**
   * Gets path planner auto to be run during autonomous.
   */
  public PathPlannerAuto getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
  }

  /**
   * Stops the robot's movement.
   */
  public void clearModuleStates() {
    m_swerveDrive.drive(new ChassisSpeeds(), true);
  }
}
