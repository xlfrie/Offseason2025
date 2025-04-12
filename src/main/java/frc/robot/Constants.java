// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //  TODO populate this class
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double kDeadbandThreshold = 0.05;
  }


  public static class PhysicalRobotConstants {
    public static final Mass kMass = Kilograms.of(35);
    public static final Distance kDriveBaseLength = Inches.of(24);

    public static final Distance k_wheelRadius = Meters.ofBaseUnits(Inches.of(2).baseUnitMagnitude());
    public static final Distance k_wheelCircumference = k_wheelRadius.times(2 * Math.PI);
  }


  public static class SimulatedControlSystemConstants {
    public static final double kPDrive = 0.1;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;

    public static final double kPSteer = 20;
    public static final double kISteer = 30;
    public static final double kDSteer = 1;
  }
}
