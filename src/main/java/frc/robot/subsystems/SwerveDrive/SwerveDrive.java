package frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MathUtils;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hardware.gyroscope.GyroIO;
import frc.robot.subsystems.hardware.module.ModuleIO;
import org.dyn4j.geometry.Vector2;

import static edu.wpi.first.units.Units.Radians;

public class SwerveDrive extends SubsystemBase {
  private GyroIO gyro;
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private ModuleIO frontLeft;
  private ModuleIO frontRight;
  private ModuleIO backLeft;
  private ModuleIO backRight;

  private StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();
  private StructArrayPublisher<SwerveModuleState> currentModuleStatesPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Current Module States", SwerveModuleState.struct).publish();

  private StructPublisher<Pose2d> realPosePublisher;

  //  TODO make sure disabling stops everything

  public SwerveDrive(GyroIO gyro, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    this.gyro = gyro;

    this.frontLeft = fl;
    this.frontRight = fr;
    this.backLeft = bl;
    this.backRight = br;

    //    TODO ensure pose2d doesn't need to be mutable
    this.swerveDrivePoseEstimator =
        new SwerveDrivePoseEstimator(RobotContainer.swerveDriveKinematics, this.gyro.getRotation(),
            getModulePositions(), Pose2d.kZero);

    if (Robot.isSimulation()) {
      this.realPosePublisher =
          NetworkTableInstance.getDefault().getStructTopic("Real Pose", Pose2d.struct).publish();
    }

    initTelemetry();
  }

  public void drive(ChassisSpeeds chassisSpeeds, boolean absolute) {
    //    TODO make use swerve kinematics class
    if (absolute) {
      double heading = getHeading().getRadians();
      calculateState(chassisSpeeds, heading, frontRight);
      calculateState(chassisSpeeds, heading, frontLeft);
      calculateState(chassisSpeeds, heading, backRight);
      calculateState(chassisSpeeds, heading, backLeft);
    }
  }

  @Override
  public void periodic() {
    swerveDrivePoseEstimator.update(gyro.getRotation(), getModulePositions());

    publishTelemetry();

    frontRight.tickPID();
    frontLeft.tickPID();
    backRight.tickPID();
    backLeft.tickPID();
  }

  private void publishTelemetry() {
    posePublisher.set(swerveDrivePoseEstimator.getEstimatedPosition());
    currentModuleStatesPublisher.set(getModuleStates());

    //    TODO move this into robot container
    if (realPosePublisher != null && RobotContainer.swerveDriveSimulation != null) {
      realPosePublisher.set(RobotContainer.swerveDriveSimulation.getSimulatedDriveTrainPose());
    }
  }

  private void calculateState(ChassisSpeeds chassisSpeeds, double heading, ModuleIO module) {
    Vector2 translationVector =
        new Vector2(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).rotate(
            -heading);
    Vector2 rotationVector =
        module.getNormalRotationVec().copy().multiply(chassisSpeeds.omegaRadiansPerSecond);

    translationVector.add(rotationVector).multiply(-1);

    Angle vecAngle = Radians.of(translationVector.getDirection());
    double vecMagnitude = translationVector.getMagnitude();

    Angle steeringAngle = module.getSteerAngle().getMeasure();

    Angle originalDifference = MathUtils.subtractAngles(vecAngle, steeringAngle);
    Angle oppositeDifference =
        MathUtils.subtractAngles(vecAngle.copy().plus(Radians.of(Math.PI)), steeringAngle);

    if (Math.abs(originalDifference.magnitude()) > Math.abs(oppositeDifference.magnitude())) {
      vecMagnitude *= -1;
      vecAngle = vecAngle.plus(Radians.of(Math.PI));
    }

    module.setDesiredState(vecMagnitude,
        translationVector.getMagnitude() == 0 ? null : new Rotation2d(vecAngle));
  }



  public Pose2d getPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public Pose2d setPose(Pose2d pose) {
    swerveDrivePoseEstimator.resetPosition(gyro.getRotation(), getModulePositions(), pose);
    return null;
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
