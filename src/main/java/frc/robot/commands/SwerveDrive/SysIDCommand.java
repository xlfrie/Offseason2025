package frc.robot.commands.SwerveDrive;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.subsystems.hardware.module.ModuleIO;
import frc.robot.utilities.controller.Controller;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VelocityUnit;

public class SysIDCommand extends Command {
  private final SwerveDrive swerve;
  private Routine routineType;
  private final ModuleIO[] modules;
  private SysIdRoutine routine;
  private final Controller controller;

  private final SendableChooser<Routine> sendableChooser;

  private final DefaultJoystickCommand defaultJoystickCommand;

  public SysIDCommand(SwerveDrive swerveDrive, Routine routineType, Controller controller) {
    this.swerve = swerveDrive;
    this.routineType = routineType;

    this.modules = swerve.getModules();
    this.controller = controller;

    this.createRoutine(routineType);

    this.sendableChooser = new SendableChooser<>();

    this.defaultJoystickCommand = new DefaultJoystickCommand(controller::getLeftX, controller::getLeftY,
        controller::getRightX, swerveDrive);

    sendableChooser.setDefaultOption("Teleop", Routine.TELEOP);

    sendableChooser.addOption("Dynamic Drive", Routine.DRIVE_VELOCITY_DYNAMIC);
    sendableChooser.addOption("QS Drive", Routine.DRIVE_VELOCITY_QUASISTATIC);
    sendableChooser.addOption("Dynamic Drive Reverse", Routine.DRIVE_VELOCITY_DYNAMIC_REVERSE);
    sendableChooser.addOption("QS Drive Reverse", Routine.DRIVE_VELOCITY_QUASISTATIC_REVERSE);

    sendableChooser.addOption("Dynamic Steer", Routine.STEER_DYNAMIC);
    sendableChooser.addOption("QS Steer", Routine.STEER_QUASISTATIC);
    sendableChooser.addOption("Dynamic Steer Reverse", Routine.STEER_DYNAMIC_REVERSE);
    sendableChooser.addOption("QS Steer Reverse", Routine.STEER_QUASISTATIC_REVERSE);

    sendableChooser.onChange(this::createRoutine);

    SmartDashboard.putData("SysID", sendableChooser);

    addRequirements(swerve);
  }

  private void setDriveVoltage(Voltage voltage) {
    for (ModuleIO module : modules) {
      module.setDriveVoltage(voltage);
      module.tickPID();
    }
  }

  private void setAzimuthVoltage(Voltage voltage) {
    for (ModuleIO module : modules) {
      module.setSteerVoltage(voltage);
    }
  }

  private void setAzimuth(Angle angle) {
    for (ModuleIO module : modules) {
      module.setSteerPID(angle.in(Radians));
    }
  }

  private void logDrive(SysIdRoutineLog log) {
    log.motor("fldrive").angularVelocity(modules[0].getDriveWheelVelocity())
        .angularPosition(modules[0].getDriveWheelPosition()).voltage(modules[0].getDriveVoltage());
    log.motor("frdrive").angularVelocity(modules[1].getDriveWheelVelocity())
        .angularPosition(modules[1].getDriveWheelPosition()).voltage(modules[1].getDriveVoltage());
    log.motor("bldrive").angularVelocity(modules[2].getDriveWheelVelocity())
        .angularPosition(modules[2].getDriveWheelPosition()).voltage(modules[2].getDriveVoltage());
    log.motor("brdrive").angularVelocity(modules[3].getDriveWheelVelocity())
        .angularPosition(modules[3].getDriveWheelPosition()).voltage(modules[3].getDriveVoltage());
  }

  private void logSteer(SysIdRoutineLog log) {
    log.motor("fldrive").angularVelocity(modules[0].getSteerVelocity())
        .angularPosition(modules[0].getSteerAngle().getMeasure()).voltage(modules[0].getSteerVoltage());
    log.motor("frdrive").angularVelocity(modules[1].getSteerVelocity())
        .angularPosition(modules[1].getSteerAngle().getMeasure()).voltage(modules[1].getSteerVoltage());
    log.motor("bldrive").angularVelocity(modules[2].getSteerVelocity())
        .angularPosition(modules[2].getSteerAngle().getMeasure()).voltage(modules[2].getSteerVoltage());
    log.motor("brdrive").angularVelocity(modules[3].getSteerVelocity())
        .angularPosition(modules[3].getSteerAngle().getMeasure()).voltage(modules[3].getSteerVoltage());
  }

  // TODO sendable chooser? not sure if we can rebind buttons. must check
  public void createRoutine(Routine routineType) {
    this.routineType = routineType;

    switch (routineType) {
      case DRIVE_VELOCITY_DYNAMIC:
      case DRIVE_VELOCITY_QUASISTATIC:
      case DRIVE_VELOCITY_DYNAMIC_REVERSE:
      case DRIVE_VELOCITY_QUASISTATIC_REVERSE:
        routine = new SysIdRoutine(new SysIdRoutine.Config((Volts.of(1)).per(Second), Volts.of(3), Seconds.of(5)),
            new SysIdRoutine.Mechanism(this::setDriveVoltage, this::logDrive, swerve));
        break;

      case STEER_DYNAMIC:
      case STEER_QUASISTATIC:
      case STEER_DYNAMIC_REVERSE:
      case STEER_QUASISTATIC_REVERSE:
        routine = new SysIdRoutine(new SysIdRoutine.Config((Volts.of(1)).per(Second), Volts.of(7), Seconds.of(10)),
            new SysIdRoutine.Mechanism(this::setAzimuthVoltage, this::logSteer, swerve));
        break;
    }

    setAzimuth(Radians.of(0));

    switch (routineType) {
      case DRIVE_VELOCITY_QUASISTATIC:
      case DRIVE_VELOCITY_QUASISTATIC_REVERSE:
      case STEER_QUASISTATIC:
      case STEER_QUASISTATIC_REVERSE:
        controller.zero().whileTrue(
            routine.quasistatic(routineType.equals(Routine.DRIVE_VELOCITY_QUASISTATIC) || routineType.equals(Routine.STEER_QUASISTATIC) ? SysIdRoutine.Direction.kForward
                : SysIdRoutine.Direction.kReverse));
        break;
      case DRIVE_VELOCITY_DYNAMIC:
      case DRIVE_VELOCITY_DYNAMIC_REVERSE:
      case STEER_DYNAMIC:
      case STEER_DYNAMIC_REVERSE:
        controller.zero().whileTrue(
            routine.dynamic(routineType.equals(Routine.DRIVE_VELOCITY_DYNAMIC) || routineType.equals(Routine.STEER_DYNAMIC)  ? SysIdRoutine.Direction.kForward
                : SysIdRoutine.Direction.kReverse));
        break;
        case TELEOP:
        break;
    }

  }

  @Override
  public void execute() {
    switch (routineType) {
      case DRIVE_VELOCITY_DYNAMIC:
      case DRIVE_VELOCITY_QUASISTATIC:
      case DRIVE_VELOCITY_DYNAMIC_REVERSE:
      case DRIVE_VELOCITY_QUASISTATIC_REVERSE:
        swerve.tickPid();
        break;

      case TELEOP:
        defaultJoystickCommand.execute();
        break;
    }
  }

  public enum Routine {
    TELEOP,

    DRIVE_VELOCITY_QUASISTATIC,
    DRIVE_VELOCITY_DYNAMIC,
    DRIVE_VELOCITY_QUASISTATIC_REVERSE,
    DRIVE_VELOCITY_DYNAMIC_REVERSE,

    STEER_QUASISTATIC,
    STEER_DYNAMIC,
    STEER_QUASISTATIC_REVERSE,
    STEER_DYNAMIC_REVERSE,
  }
}
