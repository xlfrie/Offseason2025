package frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MathUtils;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hardware.gyroscope.GyroIO;
import frc.robot.subsystems.hardware.module.ModuleIO;
import org.dyn4j.geometry.Vector2;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class SwerveDrive extends SubsystemBase {
  private final GyroIO gyro;
  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private final ModuleIO frontLeft;
  private final ModuleIO frontRight;
  private final ModuleIO backLeft;
  private final ModuleIO backRight;
  private final StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> currentModuleStatesPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Current Module States", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> desiredModuleStatesPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Desired Module States", SwerveModuleState.struct).publish();

  // This is used to calculate what the heading is relative to the driver station, changes based
  // on alliance
  private double headingOffset;

  // Real pose in simulation (if not real)
  private StructPublisher<Pose2d> realPosePublisher;

  public SwerveDrive(GyroIO gyro, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    this.gyro = gyro;

    this.frontLeft = fl;
    this.frontRight = fr;
    this.backLeft = bl;
    this.backRight = br;

    this.swerveDrivePoseEstimator =
        new SwerveDrivePoseEstimator(RobotContainer.swerveDriveKinematics, this.gyro.getRotation(),
            getModulePositions(), Pose2d.kZero);

    if (Robot.isSimulation()) {
      this.realPosePublisher =
          NetworkTableInstance.getDefault().getStructTopic("Real Pose", Pose2d.struct).publish();
    }

    RobotConfig ppConfig;
    try {
      ppConfig = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
      throw new RuntimeException("Failed to get PathPlanner config from GUI");
    }

    // TODO move pid constants to constants file (pls help i dont wanna do this)
    AutoBuilder.configure(this::getPose, this::setPose, this::getChassisSpeed,
        (ChassisSpeeds speeds) -> this.drive(speeds, false),
        new PPHolonomicDriveController(new PIDConstants(10, 2, 0), new PIDConstants(4, 8, 0.3)),
        ppConfig, () -> {
          Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();

          // Flip position to red if on red alliance
          if (allianceOptional.isPresent() && allianceOptional.get()
              .equals(DriverStation.Alliance.Red)) {
            // Origin is set at blue alliance, if we're on red then we need to flip
            // the heading
            headingOffset = Math.PI;
            return true;
          }

          headingOffset = 0;

          return false;
        });

    initTelemetry();
  }

  public void drive(ChassisSpeeds chassisSpeeds, boolean absolute) {
    // TODO make use swerve kinematics class
    double heading = getHeading().getRadians() + headingOffset;
    calculateState(chassisSpeeds, heading, frontRight, absolute);
    calculateState(chassisSpeeds, heading, frontLeft, absolute);
    calculateState(chassisSpeeds, heading, backRight, absolute);
    calculateState(chassisSpeeds, heading, backLeft, absolute);

  }

  @Override
  public void periodic() {
    swerveDrivePoseEstimator.update(gyro.getRotation(), getModulePositions());

    publishTelemetry();

    // Some PIDs need to be calculated, do that here
    frontRight.tickPID();
    frontLeft.tickPID();
    backRight.tickPID();
    backLeft.tickPID();
  }

  private void publishTelemetry() {
    posePublisher.set(swerveDrivePoseEstimator.getEstimatedPosition());
    currentModuleStatesPublisher.set(getModuleStates());
    desiredModuleStatesPublisher.set(
        new SwerveModuleState[] {frontLeft.getDesiredState(), frontRight.getDesiredState(),
            backLeft.getDesiredState(), backRight.getDesiredState()});

    // TODO move this into robot container
    if (realPosePublisher != null && RobotContainer.swerveDriveSimulation != null) {
      realPosePublisher.set(RobotContainer.swerveDriveSimulation.getSimulatedDriveTrainPose());
    }
  }

  // TODO add skew compensation constant
  private void calculateState(ChassisSpeeds chassisSpeeds, double heading, ModuleIO module,
      boolean absolute) {
    // VY is the desired left velocity, vx is the desired forward velocity
    Vector2 translationVector =
        new Vector2(-chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond);

    // Accounts for heading in absolute movement, this is all the absolute flag does
    if (absolute)
      translationVector.rotate(-heading);

    translationVector.multiply(3);

    /*
      Swerve drive kinematics are fairly simple.
      There are two components to each module's desired state.
        1. The translation vector - this is simply the desired translation velocity
        2. The rotation vector - this is the desired angular velocity times the rotation unit
        vector (which should be tangent to a line drawn from the module to the center of the
        drivetrain)

      The desired state is the sum of the two.
     */


    // Calculates rotation vector as described
    Vector2 rotationVector = module.getUnitRotationVec().copy()
        .multiply(chassisSpeeds.omegaRadiansPerSecond * 1 * Math.PI);

    // This will be the desired state
    translationVector.add(rotationVector);

    // Gets the angle of the vector, 90 degrees is subtracted from the calculated angle because 
    // heading angle's zero is set 90 degrees counterclockwise
    Angle vecAngle = Radians.of(translationVector.getDirection() - Math.PI / 2);
    double vecMagnitude = translationVector.getMagnitude();

    // Current angle that wheel is facing.
    Angle steeringAngle = module.getSteerAngle().getMeasure();

    // Calculates differences between opposite angle and angle calculated earlier
    Angle originalDifference = MathUtils.subtractAngles(vecAngle, steeringAngle);
    Angle oppositeDifference =
        MathUtils.subtractAngles(vecAngle.copy().plus(Radians.of(Math.PI)), steeringAngle);

    // Determines if it's shorter to turn the opposite and run the motor backwards, or the 
    // original calculated angle
    if (Math.abs(originalDifference.magnitude()) > Math.abs(oppositeDifference.magnitude())) {
      vecMagnitude *= -1;
      vecAngle = vecAngle.plus(Radians.of(Math.PI));
    }

    // TODO find a place in Constants for max translation speed
    // TODO multiplication should probably be moved up to be independent of rotation
    module.setDesiredState(MetersPerSecond.of(vecMagnitude),
        vecMagnitude == 0 ? null : new Rotation2d(vecAngle));
  }

  public ChassisSpeeds getChassisSpeed() {
    ChassisSpeeds chassisSpeeds =
        RobotContainer.swerveDriveKinematics.toChassisSpeeds(getModuleStates());
    chassisSpeeds.omegaRadiansPerSecond = gyro.getAngularVelocity().in(RadiansPerSecond);
    return chassisSpeeds;
  }

  public Pose2d getPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    if (Robot.isSimulation()) {
      RobotContainer.swerveDriveSimulation.setSimulationWorldPose(pose);
    }
    swerveDrivePoseEstimator.resetPosition(gyro.getRotation(), getModulePositions(), pose);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
        backLeft.getPosition(), backRight.getPosition()};
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(),
        backLeft.getState(), backRight.getState()};
  }


  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d rotation2d) {
    setPose(new Pose2d(getPose().getTranslation(), rotation2d));
  }

  public void zeroHeading() {
    setHeading(Rotation2d.kZero);
  }

  public void initTelemetry() {
    SmartDashboard.putData("SwerveDriveTelemetry", (builder) -> {
      builder.setSmartDashboardType("SwerveDriveTelemetry");
    });
  }
}
