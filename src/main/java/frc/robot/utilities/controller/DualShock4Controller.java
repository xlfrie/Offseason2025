package frc.robot.utilities.controller;

import edu.wpi.first.wpilibj.PS4Controller;

public class DualShock4Controller implements Controller{
  private PS4Controller ps4Controller;

  public DualShock4Controller(int port) {
    ps4Controller = new PS4Controller(port);
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
}
