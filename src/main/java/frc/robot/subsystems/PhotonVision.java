// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;
import frc.robot.Camera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private Camera turretCamera;

  private Pose3d pose3d;
  private Pose2d pose2d;

  private Field2d field;

  /** Creates a new PhotonVision. */
  public PhotonVision() {
    turretCamera = new Camera(LEFT_TURRET_CAMERA, LEFT_TURRET_CAMERA_TRANSFORM_3D);

    allCameras = new Camera[1];
    allCameras[0] = turretCamera;

    pose3d = new Pose3d();
    pose2d = new Pose2d();

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
    return Optional.of(getRobotPose3d().get().toPose2d());
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

  /**Calculates the average pose from all poses given by the cameras.
   * The calculated pose is used to update the robot pose3d.
   */
  public void updatePose3d() {
    int numPosesUsed = 0;
    double robotX = 0;
    double robotY = 0;
    double robotZ = 0;
    double robotRoll = 0;
    double robotPitch = 0;
    double robotYaw = 0;

    for(int i = 0; i < allCameras.length; i++) {
      allCameras[i].updateRobotPose();
    }

    for(int i = 0; i < allCameras.length; i++) {
      if(allCameras[i].getRobotPose3d().isPresent()) {
        robotX += allCameras[i].getRobotX().get();
        robotY += allCameras[i].getRobotY().get();
        robotZ += allCameras[i].getRobotZ().get();
        robotRoll += allCameras[i].getRobotRoll().get();
        robotPitch += allCameras[i].getRobotPitch().get();
        robotYaw += allCameras[i].getRobotPitch().get();

        numPosesUsed++;
      }
    }

    robotX /= numPosesUsed;
    robotY /= numPosesUsed;
    robotZ /= numPosesUsed;
    robotRoll /= numPosesUsed;
    robotPitch /= numPosesUsed;
    robotYaw /= numPosesUsed;

    pose3d = new Pose3d(
      new Translation3d(robotX, robotY, robotZ),
      new Rotation3d(robotRoll, robotPitch, robotYaw)
    );
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
  public Optional<Double> getDistanceToHub() {
    if(getRobotPose2d().isPresent() && FIELD_LAYOUT.getTagPose(allCameras[0].getTrackedHubTag().get().getFiducialId()).isPresent()) {
      return Optional.of(PhotonUtils.getDistanceToPose(getRobotPose2d().get(), FIELD_LAYOUT.getTagPose(allCameras[0].getTrackedHubTag().get().getFiducialId()).get().toPose2d()));
    }
    
    return Optional.of(0.0);
  }

  /**Gets the yaw from the robot to the hub.
   * 
   * @return The yaw (in degrees) from the robot to the hub.
   * If the distance to the hub is not present, this returns 0.0.
   */
  public Optional<Double> getYawToHub() {
    return getDistanceToHub().isPresent() ? Optional.of(allCameras[0].getYawToHub().get()) : Optional.of(0.0);
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
    builder.addDoubleProperty("Left Turret Camera Ambiguity", () -> allCameras[0].getPoseAmbiguity().get(), null);
    builder.addDoubleProperty("Right Turret Camera Ambiguity", () -> allCameras[1].getPoseAmbiguity().get(), null);
    builder.addDoubleProperty("Distance to Hub", () -> getDistanceToHub().get(), null);
    builder.addDoubleProperty("Yaw to Hub", () -> getYawToHub().get(), null);
    SmartDashboard.putData(field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose3d();

    setPose2d(getRobotPose2d().get());

    field.setRobotPose(pose2d);
  }
}
