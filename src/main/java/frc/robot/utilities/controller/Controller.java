package frc.robot.utilities.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controller {
  /**
   * @return The amount of x on the left joystick.
   */
  double getLeftX();

  /**
   * @return The amount of y on the left joystick.
   */
  double getLeftY();

  /**
   * @return The amount of x on the right joystick.
   */
  double getRightX();

  /**
   * @return The amount of y on the right joystick.
   */
  double getRightY();

  Trigger zero();
}
