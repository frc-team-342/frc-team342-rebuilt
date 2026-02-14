// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static frc.robot.Constants.VisionConstants.*;
import frc.robot.AprilTagIDs.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/** Add your docs here. */
public class Camera {
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private List<PhotonTrackedTarget> tags;
    private List<PhotonPipelineResult> cameraResults;
    private Pose3d pose;
    private PhotonTrackedTarget trackedHubTag;

    /**
     * Creates a new Camera.
     * @param cameraName The name of the camera in Photon Client.
     * @param cameraPositionOffset The transform3d from the center of the robot to the camera.
     */
    public Camera(String cameraName, Transform3d cameraPositionOffset) {
        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(FIELD_LAYOUT, cameraPositionOffset);

        cameraResults = new ArrayList<>();
        pose = new Pose3d();

        trackedHubTag = new PhotonTrackedTarget();
    }

    /**Updates the pose3d of the robot.
     * If present, a multi-tag pose is used;
     * otherwise, the lowest ambiguity pose is used.
     */
    public void updateRobotPose() {
        cameraResults = camera.getAllUnreadResults();

        for(PhotonPipelineResult result : cameraResults) {
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);

            if(!estimatedPose.isPresent()) {
                estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);

                if(!estimatedPose.isPresent()) {
                    continue;
                }
            }

            pose = estimatedPose.get().estimatedPose;
        }
    }

    /**Gets the pose3d of the robot.
     * 
     * @return The pose3d of the robot.
     */
    public Optional<Pose3d> getRobotPose3d() {
        return Optional.of(pose);
    }

    /**Gets the pose2d of the robot.
     * 
     * @return The pose2d of the robot.
     */
    public Optional<Pose2d> getRobotPose2d(){
        return Optional.of(pose.toPose2d());
    }

    /**Gets the X (north/south) of the robot.
     * 
     * @return The X of the robot.
     */
    public Optional<Double> getRobotX() {
        return Optional.of(pose.getX());
    }

    /**Gets the Y (east/west) of the robot.
     * 
     * @return The Y of the robot.
     */
    public Optional<Double> getRobotY() {
        return Optional.of(pose.getY());
    }

    /**Gets the Z (up/down) of the robot.
     * 
     * @return The Z of the robot.
     */
    public Optional<Double> getRobotZ() {
        return Optional.of(pose.getZ());
    }

    /**Gets the yaw (rotation around the z axis) of the robot.
     * 
     * @return The yaw of the robot.
     */
    public Optional<Double> getRobotYaw() {
        return Optional.of(pose.getRotation().getZ());
    }

    /**Gets the pitch (rotation around the y axis) of the robot.
     * 
     * @return The pitch of the robot.
     */
    public Optional<Double> getRobotPitch() {
        return Optional.of(pose.getRotation().getY());
    }

    /**Gets the roll (rotation around the x axis) of the robot.
     * 
     * @return The roll of the robot.
     */
    public Optional<Double> getRobotRoll() {
        return Optional.of(pose.getRotation().getX());
    }

    /**Gets the yaw from the robot to the given tag.
     * 
     * @param tag The tag to get the yaw to.
     * @return The yaw from the robot to the given tag.
     */
    public Optional<Double> getRobotToTagYaw(PhotonTrackedTarget tag) {
        return Optional.of(tag.getYaw());
    }

    /**Gets the pitch from the robot to the given tag.
     * 
     * @param tag The tag to get the pitch to.
     * @return The pitch from the robot to the given tag.
     */
    public Optional<Double> getRobotToTagPitch(PhotonTrackedTarget tag) {
        return Optional.of(tag.getPitch());
    }

    /**Gets the average pose ambiguity of all tags seen.
     * 
     * @return The average pose ambiguity of all tags seen.
     */
    public Optional<Double> getPoseAmbiguity() {
        double avgPoseAmbiguity = 0;
        List<PhotonTrackedTarget> tagsUsed = getTagsSeen();

        for(PhotonTrackedTarget tag : tagsUsed) {
            avgPoseAmbiguity += tag.getPoseAmbiguity();
        }

        return Optional.of(avgPoseAmbiguity / tagsUsed.size());
    }

    /**Checks if a tag is usable. If the tag pose is present, meets the ambiguity requirement, and is not too far, it is usable.
     * 
     * @param tag The tag to check
     */
    public boolean tagIsUsable(PhotonTrackedTarget tag) {
        return
            FIELD_LAYOUT.getTagPose(tag.getFiducialId()).isPresent() && 
            tag.getPoseAmbiguity() < AMBIGUITY_CUTOFF &&
            tag.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm() < TAG_CUTOFF_DISTANCE;
        }

    /**Gets all tags seen by the camera.
     * 
     * @return A list containing all the tags seen by the camera.
     */
    public List<PhotonTrackedTarget> getTagsSeen() {
        tags = new ArrayList<>();
        for(PhotonPipelineResult result : cameraResults) {
            for(PhotonTrackedTarget tag: result.getTargets()) {
                if(tagIsUsable(tag)) {
                    tags.add(tag);
                }
            }
        }

        return tags;
    }

    /**Gets all pipelines from the camera.
     * 
     * @return A list containing all the pipelines from the camera.
     */
    public List<PhotonPipelineResult> getPipelines() {
        return cameraResults;
    }

    /**Checks to see if an alliance hub tag is present.
     * 
     * @return {@code true} if an alliance hub tag is present, {@code false} otherwise.
     */
    public boolean allianceHubTagIsPresent() {
        for(PhotonPipelineResult result : getPipelines()) {
            for(PhotonTrackedTarget tag : result.getTargets()) {
                if(tagIsUsable(tag)) {
                    for(hubTagsIDs tagID : hubTagsIDs.values()) {
                        for(int i = 0; i < hubTagsIDs.values().length; i++) {
                            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                                if(tag.getFiducialId() == tagID.getRedHubTagID()) {
                                    trackedHubTag = tag;
                                    return true;
                                }
                            }else{
                                if(tag.getFiducialId() == tagID.getBlueHubTagID()) {
                                    trackedHubTag = tag;
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }

        return false;
    }

    /**Checks to see if an alliance outpost tag is present.
     * 
     * @return {@code true} if an alliance outpost tag is present, {@code false} otherwise.
     */
    public boolean allianceOutpostTagIsPresent() {
        for(PhotonPipelineResult result : getPipelines()) {
            for(PhotonTrackedTarget tag : result.getTargets()) {
                if(tagIsUsable(tag)) {
                    for(outpostTagsIDs tagID : outpostTagsIDs.values()) {
                        for(int i = 0; i < outpostTagsIDs.values().length; i++) {
                            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                                if(tag.getFiducialId() == tagID.getRedOutpostTagID()) {
                                    return true;
                                }
                            }else{
                                if(tag.getFiducialId() == tagID.getBlueOutpostTagID()) {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }

        return false;
    }

    /**Checks to see if an alliance tower tag is present.
     * 
     * @return {@code true} if an alliance tower tag is present, {@code false} otherwise.
     */
    public boolean allianceTowerTagIsPresent() {
        for(PhotonPipelineResult result : getPipelines()) {
            for(PhotonTrackedTarget tag : result.getTargets()) {
                if(tagIsUsable(tag)) {
                    for(towerTagsIDs tagID : towerTagsIDs.values()) {
                        for(int i = 0; i < towerTagsIDs.values().length; i++) {
                            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                                if(tag.getFiducialId() == tagID.getRedTowerTagID()) {
                                    return true;
                                }
                            }else{
                                if(tag.getFiducialId() == tagID.getBlueTowerTagID()) {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }

        return false;
    }

    /**Checks if there is a tag present in a specific camera.
     * 
     * @param camera The camera to check.
     * @return {@code true} if there is a tag, {@code false} if not.
     */
    public boolean tagIsPresentInCamera() {
        if(!getTagsSeen().isEmpty()) {
            return true;
        }

        return false;
    }

    /**Gets the hub tag that is currently being seen.
     * This method is for tracking the hub for the turret and shooter.
     * 
     * @return The hub tag that is currently being seen by the turret camera.
     */
    public Optional<PhotonTrackedTarget> getTrackedHubTag() {
        return Optional.of(trackedHubTag);
    }

    /**Gets the yaw from the robot to the hub.
     * 
     * @return The yaw (in degrees) from the robot to the hub.
     */
    public Optional<Double> getYawToHub() {
        return Optional.of(getRobotToTagYaw(getTrackedHubTag().get()).get());
    }
}