// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;
import frc.robot.Camera;
import frc.robot.AprilTagIDs.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import java.util.List;
import java.util.Optional;
import java.util.ArrayList;

public class PhotonVision extends SubsystemBase {

  private final Camera[] allCameras;

  // private Camera robotBLCamera;
  private Camera robotRightCamera;
  private Camera robotLeftCamera;
  private Camera robotBackCamera;

  private Pose3d pose3d;
  private Pose2d pose2d;
  private Pose2d turretPose2d;

  private Field2d field;

  private double timestamp;

  /** Creates a new PhotonVision. */
  public PhotonVision() {
    // robotBLCamera = new Camera(ROBOT_BL_CAMERA, ROBOT_BL_CAMERA_TRANSFORM_3D);
    robotRightCamera = new Camera(ROBOT_RIGHT_CAMERA, ROBOT_RIGHT_CAMERA_TRANSFORM_3D);
    robotLeftCamera = new Camera(ROBOT_LEFT_CAMERA, ROBOT_LEFT_CAMERA_TRANSFORM_3D);
    robotBackCamera = new Camera(ROBOT_BACK_CAMERA, RIGHT_BACK_CAMERA_TRANSFORM_3D);

    allCameras = new Camera[3];
    // allCameras[0] = robotBLCamera;
    allCameras[0] = robotRightCamera;
    allCameras[1] = robotLeftCamera;
    allCameras[2] = robotBackCamera;

    pose3d = new Pose3d();
    pose2d = new Pose2d();
    turretPose2d = new Pose2d();

    field = new Field2d();
  }

  /**Returns all tags seen by all cameras.
   * 
   * @return The list of tags seen.
   */
  public List<PhotonTrackedTarget> getAllTagsSeen() {
    List<PhotonTrackedTarget> tags = new ArrayList<>();
    for(int i = 0; i < allCameras.length; i++) {
      tags.addAll(allCameras[i].getTagsSeen());
    }

    return tags;
  }

  /**Returns the requested tag.
   * 
   * @param tagID The ID of the tag to look for.
   * @return The requested tag.
   */
  public PhotonTrackedTarget getSpecificTag(double tagID) {
    List<PhotonTrackedTarget> wantedTags = new ArrayList<>();

    for(PhotonTrackedTarget tag : getAllTagsSeen()) {
      if(tag.getFiducialId() == tagID) {
        wantedTags.add(tag);
      }
    }

    if(wantedTags.size() > 0) {
      PhotonTrackedTarget firstTag = wantedTags.get(0);
      PhotonTrackedTarget previousTag = wantedTags.get(0);
      PhotonTrackedTarget bestTag = wantedTags.get(0);
      for(PhotonTrackedTarget tag : wantedTags) {
        if(tag.getFiducialId() != firstTag.getFiducialId()) {
          if(tag.getPoseAmbiguity() < previousTag.getPoseAmbiguity()) {
            bestTag = tag;
          }
        }
        previousTag = tag;
      }

      return bestTag;
    }

    return null;
  }

  /**Gets all pipelines from all cameras.
   * 
   * @return A list of all pipelines from all cameras.
   */
  public List<PhotonPipelineResult> getAllPipelines() {
    List<PhotonPipelineResult> results = new ArrayList<>();
    for(int i = 0; i < allCameras.length; i++) {
      results.addAll(allCameras[i].getPipelines());
    }

    return results;
  }

  /**Checks if there is a tag present in any camera.
   * 
   * @return {@code true} if there is a tag, {@code false} if not.
   */
  public boolean tagIsPresentAcrossAllCameras() {
    if(!getAllTagsSeen().isEmpty()) {
      return true;
    }

    return false;
  }

  /**Checks if there is an alliance hub tag seen by any camera.
   * 
   * @return {@code true} if an alliance hub tag is present, {@code false} if not.
   */
  public boolean allianceHubTagSeen() {
    for(int i = 0; i < allCameras.length; i++) {
      if(allCameras[i].allianceHubTagIsPresent()) {
        return true;
      }
    }

    return false;
  }

  /**Checks if there is an alliance outpost tag seen by any camera.
   * 
   * @return {@code true} if an alliance outpost tag is present, {@code false} if not.
   */
  public boolean allianceOutpostTagSeen() {
    for(int i = 0; i < allCameras.length; i++) {
      if(allCameras[i].allianceOutpostTagIsPresent()) {
        return true;
      }
    }

    return false;
  }

  /**Checks if there is an alliance tower tag seen by any camera.
   * 
   * @return {@code true} if an alliance tower tag is present, {@code false} if not.
   */
  public boolean allianceTowerTagSeen() {
    for(int i = 0; i < allCameras.length; i++) {
      if(allCameras[i].allianceTowerTagIsPresent()) {
        return true;
      }
    }

    return false;
  }

  /**Gets the pose3d of the robot.
   * 
   * @return The pose3d of the robot.
   */
  public Optional<Pose3d> getRobotPose3d() {
    return Optional.of(pose3d);
  }

  /**Gets the pose2d of the robot.
   * 
   * @return The pose2d of the robot.
   */
  public Optional<Pose2d> getRobotPose2d() {
    return Optional.of(pose2d);
  }

  /**Gets the pose2d of the turret.
   * 
   * @return The pose2d of the turret.
   */
  public Optional<Pose2d> getTurretPose2d() {
    return Optional.of(turretPose2d);
  }

  /**Gets the X of the robot.
   * 
   * @return The X of the robot.
   */
  public Optional<Double> getRobotX() {
    return Optional.of(getRobotPose3d().get().getX());
  }

  /**Gets the Y of the robot.
   * 
   * @return The Y of the robot.
   */
  public Optional<Double> getRobotY() {
    return Optional.of(getRobotPose3d().get().getY());
  }

  /**Gets the Z of the robot.
   * 
   * @return The Z of the robot.
   */
  public Optional<Double> getRobotZ() {
    return Optional.of(getRobotPose3d().get().getZ());
  }

  /**Gets the yaw of the robot.
   * 
   * @return The yaw of the robot.
   */
  public Optional<Double> getRobotYaw() {
    return Optional.of(getRobotPose3d().get().getRotation().getZ());
  }

  /**Gets the pitch of the robot.
   * 
   * @return The pitch of the robot.
   */
  public Optional<Double> getRobotPitch() {
    return Optional.of(getRobotPose3d().get().getRotation().getY());
  }

  /**Gets the roll of the robot.
   * 
   * @return The roll of the robot.
   */
  public Optional<Double> getRobotRoll() {
    return Optional.of(getRobotPose3d().get().getRotation().getX());
  }

  /**Gets the yaw from the robot to the given tag.
   * 
   * @param tag The tag to get the yaw to.
   * @return The yaw from the robot to the given tag.
   */
  public Optional<Double> getRobotToTagYaw(PhotonTrackedTarget tag) {
    int numCamsUsed = 0;
    double yaw = 0;

    for(int i = 0; i < allCameras.length; i++) {
      if(allCameras[i].getRobotPose3d().isPresent()) {
        yaw += allCameras[i].getRobotToTagYaw(tag).get();
        numCamsUsed++;
      }
    }

    return Optional.of(yaw /= numCamsUsed);
  }

  /**Gets the pitch from the robot to the given tag.
   * 
   * @param tag The tag to get the pitch to.
   * @return The pitch from the robot to the given tag.
   */
  public Optional<Double> getRobotToTagPitch(PhotonTrackedTarget tag) {
    int numCamsUsed = 0;
    double pitch = 0;

    for(int i = 0; i < allCameras.length; i++) {
      if(allCameras[i].getRobotPose3d().isPresent()) {
        pitch += allCameras[i].getRobotToTagPitch(tag).get();
        numCamsUsed++;
      }
    }

    return Optional.of(pitch /= numCamsUsed);
  }

  /**Gets the timestamp at which the pose was calculated.
   * 
   * @return The timestamp at which the pose was calculated.
   */
  public double getTimestampOfPose() {
    return timestamp;
  }

  /**Updates the robot pose3d using the best pose from the cameras.
   * Best pose is determined by ambiguity.
   */
  public void updatePose3d() {
    Pose3d bestPose = null;
    double lowestAmbiguity = 10;

    for(int i = 0; i < allCameras.length; i++) {
      allCameras[i].updateRobotPose();

      if(allCameras[i].getRobotPose3d().isPresent()) {
        if(allCameras[i].getPoseAmbiguity().get() != -1 && allCameras[i].getPoseAmbiguity().get() < lowestAmbiguity) {
          bestPose = allCameras[i].getRobotPose3d().get();
          lowestAmbiguity = allCameras[i].getPoseAmbiguity().get();
          timestamp = allCameras[i].getTimestamp();
        }
      }
    }

    if(bestPose == null) {
      
    }else{
      pose3d = bestPose;
    }
  }

  /**Updates the robot pose2d using the robot pose3d.
   * 
   */
  public void updatePose2d() {
    this.pose2d = getRobotPose3d().get().toPose2d();
  }

  /**Sets the robot pose3d to the given pose3d.
   * 
   * @param pose3d The given pose3d. 
   */
  public void setPose3d(Pose3d pose3d) {
    this.pose3d = pose3d;
  }

  /**Sets the robot pose2d to the given pose2d.
   * 
   * @param pose2d The given pose2d.
   */
  public void setPose2d(Pose2d pose2d) {
    this.pose2d = pose2d;
  }

  /**Gets the distance from the robot to the hub.
   * 
   * @return The distance (in meters) from the robot to the hub.
   * If the robot pose2d or the hub tag is not present, this returns 0.0.
   */
  public double getDistanceToHub(Pose2d pose) {
    return PhotonUtils.getDistanceToPose(pose, getHubCenterPose2d());
  }

  /**Gets the pose2d of the center of the hub.
   * This uses the poses of the front and back centered hub tags.
   * 
   * @return Pose2d of the center of the hub.
   */
  public Pose2d getHubCenterPose2d() {
    double x = 0.0;
    double y = 0.0;
    double rotation = 0.0;

    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      x = (FIELD_LAYOUT.getTagPose(centerHubTagsIDs.FRONT.getCenterRedHubTagID()).get().toPose2d().getX() + FIELD_LAYOUT.getTagPose(centerHubTagsIDs.BACK.getCenterRedHubTagID()).get().toPose2d().getX()) / 2;
      y = (FIELD_LAYOUT.getTagPose(centerHubTagsIDs.FRONT.getCenterRedHubTagID()).get().toPose2d().getY() + FIELD_LAYOUT.getTagPose(centerHubTagsIDs.BACK.getCenterRedHubTagID()).get().toPose2d().getY()) / 2;
    }else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      x = (FIELD_LAYOUT.getTagPose(centerHubTagsIDs.FRONT.getCenterBlueHubTagID()).get().toPose2d().getX() + FIELD_LAYOUT.getTagPose(centerHubTagsIDs.BACK.getCenterBlueHubTagID()).get().toPose2d().getX()) / 2;
      y = (FIELD_LAYOUT.getTagPose(centerHubTagsIDs.FRONT.getCenterBlueHubTagID()).get().toPose2d().getY() + FIELD_LAYOUT.getTagPose(centerHubTagsIDs.BACK.getCenterBlueHubTagID()).get().toPose2d().getY()) / 2;
    }
    
    return new Pose2d(x, y, new Rotation2d(rotation));
  }

  public void setTurretPose2d(double x, double y, double rotation) {
    turretPose2d = new Pose2d(x, y, new Rotation2d(rotation));
  }

  //Puts data for vision on Elastic
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("PhotonVision");

    builder.addBooleanProperty("Alliance Hub Tag Seen", () -> allianceHubTagSeen(), null);
    builder.addBooleanProperty("Alliance Tower Tag Seen", () -> allianceTowerTagSeen(), null);
    builder.addBooleanProperty("Alliance Outpost Tag Seen", () -> allianceOutpostTagSeen(), null);
    builder.addBooleanProperty("Tag Seen", () -> tagIsPresentAcrossAllCameras(), null);
    builder.addDoubleProperty("Robot X", () -> getRobotX().get(), null);
    builder.addDoubleProperty("Robot Y", () -> getRobotY().get(), null);
    builder.addDoubleProperty("Robot Z", () -> getRobotZ().get(), null);
    builder.addDoubleProperty("Robot Roll", () -> getRobotRoll().get(), null);
    builder.addDoubleProperty("Robot Pitch", () -> getRobotPitch().get(), null);
    builder.addDoubleProperty("Robot Yaw", () -> getRobotYaw().get(), null);
    builder.addDoubleProperty("Right Robot Camera Ambiguity", () -> allCameras[0].getPoseAmbiguity().get(), null);
    builder.addDoubleProperty("Left Robot Camera Ambiguity", () -> allCameras[1].getPoseAmbiguity().get(), null);
    builder.addDoubleProperty("Back Robot Camera Ambiguity", () -> allCameras[2].getPoseAmbiguity().get(), null);
    builder.addDoubleProperty("Distance to Hub", () -> getDistanceToHub(getTurretPose2d().get()), null);
    builder.addDoubleProperty("Turret X", () -> getTurretPose2d().get().getX(), null);
    builder.addDoubleProperty("Turret Y", () -> getTurretPose2d().get().getY(), null);
    builder.addDoubleProperty("Robot Voltage", () -> RobotController.getBatteryVoltage(), null);
    builder.addDoubleProperty("Center of Hub X", () -> getHubCenterPose2d().getX(), null);
    builder.addDoubleProperty("Center of Hub Y", () -> getHubCenterPose2d().getY(), null);
    builder.addDoubleProperty("Center of Hub Rotation", () -> getHubCenterPose2d().getRotation().getRadians(), null);
    SmartDashboard.putData(field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose3d();
    updatePose2d();
    
    getDistanceToHub(getTurretPose2d().get());

    field.setRobotPose(pose2d);
  }
}
