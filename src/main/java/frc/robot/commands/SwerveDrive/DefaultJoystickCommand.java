package frc.robot.commands.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.OperatorConstants.kDeadbandThreshold;

public class DefaultJoystickCommand extends Command {
  private final DoubleSupplier ly;
  private final DoubleSupplier lx;
  private final DoubleSupplier rx;
  private final SwerveDrive drive;


  /**
   * @param lx          Translation on the x-axis supplier
   * @param ly          Translation on the y-axis supplier
   * @param rx          Desired rotation velocity supplier
   * @param swerveDrive Swerve drive train instance
   */
  public DefaultJoystickCommand(DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier rx,
      SwerveDrive swerveDrive) {
    this.lx = lx;
    this.ly = ly;
    this.rx = rx;

    this.drive = swerveDrive;

    addRequirements(swerveDrive);
  }

  /**
   * @param rawInput The input that the deadband should be applied to
   * @return The given input where if the magnitude is below the deadband threshold it is equal to
   * zero
   */
  private double applyDeadband(double rawInput) {
    return Math.abs(rawInput) < kDeadbandThreshold ? 0 : rawInput;
  }

  /**
   * Send controller data to the drive train.
   */
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds =
        new ChassisSpeeds(applyDeadband(-ly.getAsDouble()), applyDeadband(-lx.getAsDouble()),
            applyDeadband(-rx.getAsDouble()));

    drive.drive(chassisSpeeds, true);
  }

  /**
   * Called when the command is stopped.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    drive.drive(new ChassisSpeeds(), true);
  }
}
