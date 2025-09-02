package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashSet;
import java.util.Set;

public class SwerveDriveConfigurator {
  public final SwerveDriveRobotConstants swerveDriveRobotConstants;

  private final SwerveDriveModuleConstants[] moduleConstants;

  public SwerveDriveConfigurator(SwerveDriveRobotConstants swerveDriveRobotConstants,
      SwerveDriveModuleConstants[] moduleConstants) {
    this.swerveDriveRobotConstants = swerveDriveRobotConstants;

    Set<SwerveModuleCornerPosition> swerveModuleCornerPositionSet = new HashSet<>();
    this.moduleConstants = moduleConstants;

    for (SwerveDriveModuleConstants moduleConstant : moduleConstants) {


      swerveModuleCornerPositionSet.add(moduleConstant.corner);

      moduleConstant.calculateModulePosition(swerveDriveRobotConstants.moduleDistance);
    }

    if (moduleConstants.length < 4) {
      DriverStation.reportWarning("Less than four modules were defined in the configurator", false);
    }

    if (moduleConstants.length > 4) {
      DriverStation.reportError("More than five modules were defined in the configurator", false);
    }

    if (swerveModuleCornerPositionSet.size() != moduleConstants.length) {
      DriverStation.reportError("Duplicate swerve module positions were configured", false);
    }
  }

  public SwerveDriveModuleConstants getModuleConstants(SwerveModuleCornerPosition corner) {
    for (SwerveDriveModuleConstants moduleConstant : moduleConstants) {
      if (moduleConstant.corner == corner) {
        return moduleConstant;
      }
    }

    throw new RuntimeException("Tried to get configuration of an unconfigured module");
  }

  public static class SwerveDriveRobotConstants {
    public final Mass robotMass;
    public final Distance driveBaseLength;
    public final Distance moduleDistance;
    public final Distance wheelRadius;
    public final Distance wheelCircumference;

    public final int pigeonID;

    public SwerveDriveRobotConstants(Mass robotMass, Distance driveBaseLength,
        Distance moduleDistance, Distance wheelRadius, int pigeonID) {
      this.robotMass = robotMass;
      this.driveBaseLength = driveBaseLength;
      this.moduleDistance = moduleDistance;
      this.wheelRadius = wheelRadius;
      this.wheelCircumference = wheelRadius.times(2 * Math.PI);

      this.pigeonID = pigeonID;
    }
  }


  public static class SwerveDriveModuleConstants {
    private final SwerveModuleCornerPosition corner;

    public final int CANCoderID;
    public final int driveMotorID;
    public final int azimuthMotorID;

    public final double CANCoderOffset;

    public final double kPDrive;
    public final double kIDrive;
    public final double kDDrive;
    public final double kSDrive;
    public final double kVDrive;

    public final double kPAzimuth;
    public final double kIAzimuth;
    public final double kDAzimuth;
    public final double kSAzimuth;

    public final double driveGearRatio;

    public final boolean azimuthReversed;

    public Translation2d physicalModulePosition;

    public SwerveDriveModuleConstants(SwerveDriveModuleConstants clonedConstants,
        SwerveModuleCornerPosition corner, int CANCoderID, int driveMotorID, int azimuthMotorID,
        double CANCoderOffset) {
      this.corner = corner;

      this.CANCoderID = CANCoderID;
      this.driveMotorID = driveMotorID;
      this.azimuthMotorID = azimuthMotorID;

      this.CANCoderOffset = CANCoderOffset;

      this.kPDrive = clonedConstants.kPDrive;
      this.kIDrive = clonedConstants.kIDrive;
      this.kDDrive = clonedConstants.kDDrive;
      this.kSDrive = clonedConstants.kSDrive;
      this.kVDrive = clonedConstants.kVDrive;

      this.kPAzimuth = clonedConstants.kPAzimuth;
      this.kIAzimuth = clonedConstants.kIAzimuth;
      this.kDAzimuth = clonedConstants.kDAzimuth;
      this.kSAzimuth = clonedConstants.kSAzimuth;

      this.driveGearRatio = clonedConstants.driveGearRatio;

      this.azimuthReversed = clonedConstants.azimuthReversed;
    }

    // TODO fix wheel spinning when wheel is rotating (like planetary gears)
    public SwerveDriveModuleConstants(SwerveModuleCornerPosition corner, int CANCoderID,
        int driveMotorID, int azimuthMotorID, double CANCoderOffset, double kPDrive, double kIDrive,
        double kDDrive, double kSDrive, double kVDrive, double kPAzimuth, double kIAzimuth,
        double kDAzimuth, double kSAzimuth, int azimuthReverseCount, boolean shaftFacingUp,
        double driveGearRatio) {
      this.corner = corner;

      this.CANCoderID = CANCoderID;
      this.driveMotorID = driveMotorID;
      this.azimuthMotorID = azimuthMotorID;

      this.CANCoderOffset = CANCoderOffset;

      this.kPDrive = kPDrive;
      this.kIDrive = kIDrive;
      this.kDDrive = kDDrive;
      this.kSDrive = kSDrive;
      this.kVDrive = kVDrive;

      this.kPAzimuth = kPAzimuth;
      this.kIAzimuth = kIAzimuth;
      this.kDAzimuth = kDAzimuth;
      this.kSAzimuth = kSAzimuth;

      this.driveGearRatio = driveGearRatio;

      //   TODO check NEO/SparkMAX defaults to ccw+

      boolean reversed = !shaftFacingUp;
      if (azimuthReverseCount % 2 == 0)
        reversed = !reversed;

      azimuthReversed = reversed;
    }

    private void calculateModulePosition(Distance moduleDistance) {
      double distance = moduleDistance.div(2).in(Units.Meters);
      this.physicalModulePosition =
          new Translation2d(distance, distance).rotateBy(Rotation2d.kCCW_90deg.times(corner.index));
    }

    public String getModuleName() {
      return corner.getName();
    }
  }


  public enum SwerveModuleCornerPosition {
    FRONT_RIGHT("Front Right", 0),

    FRONT_LEFT("Front Left", 1),

    BACK_LEFT("Back Left", 2),

    BACK_RIGHT("Back Right", 3);


    private String name;
    private int index;

    SwerveModuleCornerPosition(String name, int id) {
      this.name = name;
      this.index = id;
    }

    public String getName() {
      return name;
    }

    public int getIndex() {
      return index;
    }
  }
}
