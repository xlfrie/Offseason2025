// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.hardware.vision.VisionIOSim;
import org.ironmaple.simulation.SimulatedArena;

public class Robot extends TimedRobot {
  private final RobotContainer m_robotContainer;
  private PathPlannerAuto autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate all of useful robot code
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Do not remove, ticks commands
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.clearModuleStates();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * Runs at beginning of autonomous.
   * <p>
   * Schedules autonomous command
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Ensures that operator cannot move robot during autonomous
    // WARN do not remove, will break path planner
    m_robotContainer.unbindJoystick();

    if (autonomousCommand != null)
      autonomousCommand.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** Called at beginning of teleop */
  @Override
  public void teleopInit() {
    // Make sure that our movement command is registered
    m_robotContainer.bindJoystickCommand();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
      ((VisionIOSim) RobotContainer.visionIO).tick();


    SimulatedArena.getInstance().simulationPeriodic();
  }
}
