package frc.robot.utilities.controller;

import edu.wpi.first.wpilibj.Joystick;

public class SimulatedController implements Controller {
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;


  /**
   * Instantiate a keyboard based controller for simulation.
   */
  public SimulatedController() {
    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
  }

  @Override
  public double getLeftX() {
    return leftJoystick.getX();
  }

  @Override
  public double getLeftY() {
    return leftJoystick.getY();
  }

  @Override
  public double getRightX() {
    return rightJoystick.getX();
  }

  @Override
  public double getRightY() {
    return rightJoystick.getY();
  }
}
