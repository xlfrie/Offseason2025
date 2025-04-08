package frc.robot.commands.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

import java.util.function.DoubleSupplier;

public class DefaultJoystickCommand extends Command {
  private DoubleSupplier ly;
  private DoubleSupplier lx;
  private DoubleSupplier rx;
  private SwerveDrive drive;

  private final double deadbandThreshold = 0.04;

  public DefaultJoystickCommand(DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier rx,
      SwerveDrive swerveDrive) {
    this.lx = lx;
    this.ly = ly;
    this.rx = rx;

    this.drive = swerveDrive;

    addRequirements(swerveDrive);
  }

  private double applyDeadband(double rawInput) {
    return Math.abs(rawInput) < deadbandThreshold ? 0 : rawInput;
  }

  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds(applyDeadband(ly.getAsDouble()), applyDeadband(lx.getAsDouble()),
            applyDeadband(rx.getAsDouble()));

    drive.drive(chassisSpeeds, true);
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(new ChassisSpeeds(), true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
