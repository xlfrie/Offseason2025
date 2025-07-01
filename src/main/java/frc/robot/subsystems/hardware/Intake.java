package frc.robot.subsystems.hardware;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final SparkMax intakeMotor;

  public Intake() {
    this.intakeMotor = new SparkMax(Constants.RealRobotConstants.kIntakeMotorID,
        SparkLowLevel.MotorType.kBrushless);

    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.inverted(true);
    intakeMotorConfig.smartCurrentLimit(50);

    intakeMotor.configure(intakeMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {
    CommandScheduler.getInstance().run();
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public Command idle() {
    return Commands.runOnce(() -> {
      this.setIntakeSpeed(INTAKE_STATE.IDLE.speed);
    }, this);
  }

  public Command reverse() {
    return Commands.runOnce(() -> {
      this.setIntakeSpeed(INTAKE_STATE.REVERSE.speed);
    }, this);
  }

  public Command fast() {
    return Commands.runOnce(() -> {
      this.setIntakeSpeed(INTAKE_STATE.FAST_OUT.speed);
    }, this);
  }

  public Command normal() {
    return Commands.runOnce(() -> {
      this.setIntakeSpeed(INTAKE_STATE.NORMAL_OUT.speed);
    }, this);
  }
}


enum INTAKE_STATE {
  REVERSE(-0.2),
  IDLE(0.02),
  NORMAL_OUT(0.16),
  FAST_OUT(0.185);

  public final double speed;

  INTAKE_STATE(double speed) {
    this.speed = speed;
  }
}
