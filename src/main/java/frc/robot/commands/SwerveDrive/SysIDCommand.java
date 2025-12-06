package frc.robot.commands.SwerveDrive;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.subsystems.hardware.module.ModuleIO;
import frc.robot.utilities.controller.Controller;

import static edu.wpi.first.units.Units.Radians;

public class SysIDCommand extends Command {
  private final SwerveDrive swerve;
  private final Routine routineType;
  private final ModuleIO[] modules;
  private SysIdRoutine routine;
  private final Controller controller;

  public SysIDCommand(SwerveDrive swerveDrive, Routine routineType, Controller controller) {
    this.swerve = swerveDrive;
    this.routineType = routineType;

    this.modules = swerve.getModules();
    this.controller = controller;


    this.createRoutine();

    addRequirements(swerve);
  }

  private void setDriveVoltage(Voltage voltage) {
    for (ModuleIO module : modules) {
      module.setDriveVoltage(voltage);
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
    log.motor("fldrive").angularVelocity(modules[0].getDriveWheelVelocity()).angularPosition(modules[0].getDriveWheelPosition()).voltage(modules[0].getDriveVoltage());
    log.motor("frdrive").angularVelocity(modules[1].getDriveWheelVelocity()).angularPosition(modules[1].getDriveWheelPosition()).voltage(modules[1].getDriveVoltage());
    log.motor("bldrive").angularVelocity(modules[2].getDriveWheelVelocity()).angularPosition(modules[2].getDriveWheelPosition()).voltage(modules[2].getDriveVoltage());
    log.motor("brdrive").angularVelocity(modules[3].getDriveWheelVelocity()).angularPosition(modules[3].getDriveWheelPosition()).voltage(modules[3].getDriveVoltage());
  }

  // TODO sendable chooser? not sure if we can rebind buttons. must check
  public void createRoutine() {
    switch (routineType) {
      case DRIVE_VELOCITY_DYNAMIC:
      case DRIVE_VELOCITY_QUASISTATIC:
        SysIdRoutine.Config conf = new SysIdRoutine.Config();
        routine = new SysIdRoutine(conf, new SysIdRoutine.Mechanism(this::setDriveVoltage, this::logDrive,swerve));
        break;
    }

    switch (routineType) {
      case DRIVE_VELOCITY_QUASISTATIC:
        controller.zero().whileTrue(routine.dynamic(SysIdRoutine.Direction.kForward).andThen(routine.quasistatic(SysIdRoutine.Direction.kForward)));
        break;
      case DRIVE_VELOCITY_DYNAMIC:
        controller.zero().whileTrue(routine.dynamic(SysIdRoutine.Direction.kForward).andThen(routine.quasistatic(SysIdRoutine.Direction.kForward)));

        break;
    }

  }

  @Override
  public void execute() {
    switch (routineType) {
      case DRIVE_VELOCITY_DYNAMIC:
      case DRIVE_VELOCITY_QUASISTATIC:
        swerve.tickPid();
        break;
    }
  }

  public enum Routine {
    DRIVE_VELOCITY_QUASISTATIC,
    DRIVE_VELOCITY_DYNAMIC,
  }
}
