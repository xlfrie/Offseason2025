package frc.robot.utilities.controller;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DualShock4Controller implements Controller {
  private final CommandPS4Controller ps4Controller;

  public DualShock4Controller(int port) {
    ps4Controller = new CommandPS4Controller(port);
  }

  @Override
  public double getLeftX() {
    return ps4Controller.getLeftX();
  }

  @Override
  public double getLeftY() {
    return ps4Controller.getLeftY();
  }

  @Override
  public double getRightX() {
    return ps4Controller.getRightX();
  }

  @Override
  public double getRightY() {
    return ps4Controller.getRightY();
  }

  @Override
  public Trigger zero() {
    return ps4Controller.R1();
  }
}
