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
  // TODO populate this class
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double kDeadbandThreshold = 0.08;
  }


  public static class PhysicalRobotConstants {
    public static final Mass kMass = Kilograms.of(35);
    // TODO verify this and add constant for module positions
    public static final Distance kDriveBaseLength = Inches.of(20);

    public static final Distance k_wheelRadius = Inches.of(2);
    public static final Distance k_wheelCircumference = k_wheelRadius.times(2 * Math.PI);
  }


  public static class RealRobotConstants {
    public static final int kPigeon2ID = 9;

    public static final int kFRDriveMotorID = 23;
    public static final int kFLDriveMotorID = 24;
    public static final int kBLDriveMotorID = 22;
    public static final int kBRDriveMotorID = 21;

    public static final int kFRAzimuthMotorID = 13;
    public static final int kFLAzimuthMotorID = 14;
    public static final int kBLAzimuthMotorID = 12;
    public static final int kBRAzimuthMotorID = 11;

    public static final int kFRCANCoderID = 33;
    public static final int kFLCANCoderID = 34;
    public static final int kBLCANCoderID = 32;
    public static final int kBRCANCoderID = 31;

    // TODO figure out if this is correct
    public static final boolean kDriveReversed = false;
    public static final boolean kAzimuthReversed = true;

    // TODO zero these encoders
    public static final double kFRCANCoderOffset = 0.096 * Math.PI * 2;
    public static final double kFLCANCoderOffset = 0.655 * Math.PI * 2;
    public static final double kBLCANCoderOffset = 0.919 * Math.PI * 2;
    public static final double kBRCANCoderOffset = 0.117 * Math.PI * 2;

    public static final double kPDrive = 0.125;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0.01;
    public static final double kSDrive = 0;
    public static final double kVDrive = 0.11;

    public static final double kPAzimuth = 5;
    public static final double kIAzimuth = 0;
    public static final double kDAzimuth = 0;
  }


  public static class SimulatedControlSystemConstants {
    public static final double kSDrive = 0;
    public static final double kVDrive = 2.435;
    public static final double kADrive = 0;

    public static final double kPDrive = 8;
    public static final double kIDrive = 0;
    public static final double kDDrive = 0;

    public static final double kPSteer = 28;
    public static final double kISteer = 0;
    public static final double kDSteer = 0.5;
  }
}
