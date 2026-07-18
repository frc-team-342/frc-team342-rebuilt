// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static frc.robot.Constants.VisionConstants.*;
import frc.robot.AprilTagIDs.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/*
 * A lot of the methods in this class return an Optional form of a value. This is because
 * certain values (e.g., poses) do not get assigned a value at startup. This causes an error
 * to be thrown when null is returned. Optional prevents this by returning false when
 * a value is null rather than attempting to return null itself.
 */

/** Add your docs here. */
public class Camera {
    private PhotonCamera camera; //A PhotonCamera
    private PhotonPoseEstimator poseEstimator; //A PhotonPoseEstimator
    private List<PhotonTrackedTarget> tags; //A list of all tags seen
    private List<PhotonPipelineResult> cameraResults; //A list of all camera results
    private Pose3d pose; //The pose from the estimator
    private PhotonTrackedTarget trackedHubTag; //The alliance hub tag that is currently seen
    private boolean currentlyTrackingHub; //Whether an alliance hub tag is seen or not
    private double timestamp; //The timestamp at which the pose was estimated. This allows for latency compensation
    /*
     * Camera intrinsics and distortion coefficients allow for the use of estimateConstrainedSolvepnpPose.
     * The estimateConstrainedSolvepnpPose method calculates a Pose3d by using a
     * 2d image of an april tag, camera intrinsics, and distortion coefficients. If done correctly,
     * it gives more accurate and reliable poses than estimateCoprocMultiTagPose and estimateLowestAmbiguityPose.
     */
    // private Matrix<N3, N3> cameraIntrinsics; //The camera intrinsics pulled from the PhotonVision Client
    // private Matrix<N8, N1> distortionCoefficients; //The distortion coefficients pulled from the PhotonVision Client
    /*
     * Vision standard deviations can be used to determine the reliability of vision readings. This
     * allows SwerveDrivePoseEstimator to determine whether to prioritize odometry readings or
     * vision readings when estimating a pose.
     */
    // private Matrix<N3, N1> visionStandardDeviations; //The calculated vision standard deviations

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
        currentlyTrackingHub = false;

        timestamp = 0.0;
        /*
         * The first number is for the x value, the second for the y value, and the third for the heading.
         * Set the default trust in the x and y to 0.5. Heading values from the gyro will almost
         * always be more reliable than heading values from vision, so default the trust in
         * heading to the max double value.
         */
        // visionStandardDeviations = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
    }

    /**Updates the pose3d of the robot.
     * If present, a multi-tag pose is used;
     * otherwise, the lowest ambiguity pose is used.
     */
    public void updateRobotPose(/*Rotation3d heading*/) {
        cameraResults = camera.getAllUnreadResults();

        /*
         * Since getAllUnreadResults returns a list of pipelines, we want to go through all of them.
         * To do this, we can used an enhanced for loop (or a for-each loop, whichever name you prefer).
         */
        for(PhotonPipelineResult result : cameraResults) {
            /*
             * estimateCoprocMultiTagPose gives more accurate pose estimations than estimateLowestAmbiguityPose,
             * so we want to try getting a multi-tag pose first.
             */
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);

            //If we can't get a multi-tag pose, then we fall back on estimateLowestAmbiguityPose.
            if(!estimatedPose.isPresent()) {
                estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
                //If we can't get a lowest ambiguity pose, then we go to the next iteration of the loop.
                if(!estimatedPose.isPresent()) {
                    continue;
                }

                /*
                 * estimateConstrainedSolvepnpPose should only be used if you have a multi-tag pose.
                 * So, if we only have a lowest ambiguity pose, we want to pull the pose from that
                 * estimation then continue to the next iteration of the loop.
                 */
                // timestamp = estimatedPose.get().timestampSeconds;
                // pose = estimatedPose.get().estimatedPose;
                // visionStandardDeviations = calculateVisionStandardDeviations(pose, getTagsSeen());
                // continue;
            }

            /*
             * If using estimateConstrainedSolvepnpPose, addHeadingData has to be called every loop
             * before a constrained pnp pose can be estimated.
             */
            // poseEstimator.addHeadingData(estimatedPose.get().timestampSeconds, heading);

            // Optional<EstimatedRobotPose> constrainedPose = poseEstimator.estimateConstrainedSolvepnpPose(result, cameraIntrinsics, distortionCoefficients, estimatedPose.get().estimatedPose, true, 0);

            //If the constrained pnp pose cannot be estimated, use the multi-tag pose instead.
            // if(!constrainedPose.isPresent()) {
            //     timestamp = estimatedPose.get().timestampSeconds;
            //     pose = estimatedPose.get().estimatedPose;
            //     visionStandardDeviations = calculateVisionStandardDeviations(pose, getTagsSeen());
            //     continue;
            // }

            //Pull the timestamp and pose from the constrained pnp pose estimation
            // timestamp = constrainedPose.get().timestampSeconds;
            // pose = constrainedPose.get().estimatedPose;
            // visionStandardDeviations = calculateVisionStandardDeviations(pose, getTagsSeen());

            /*
             * If not using estimateConstrainedSolvepnpPose, simply pull the timestamp and pose
             * from the pose that is present.
             */
            timestamp = estimatedPose.get().timestampSeconds;
            pose = estimatedPose.get().estimatedPose;
        }
    }

    // /**Sets the camera intrinsics matrix.
    //  * 
    //  * @param cameraIntrinsics The camera intrinsics matrix to use.
    //  */
    // public void setCameraIntrinsics(Matrix<N3, N3> cameraIntrinsics) {
    //     this.cameraIntrinsics = cameraIntrinsics;
    // }

    // /**Sets the distortion coefficients matrix.
    //  * 
    //  * @param distortionCoefficients The distortion coefficients matrix to use.
    //  */
    // public void setDistortionCoefficients(Matrix<N8, N1> distortionCoefficients) {
    //     this.distortionCoefficients = distortionCoefficients;
    // }

    // /**Gets the camera intrinsics matrix of the camera.
    //  * 
    //  * @return The camera intrinsics.
    //  */
    // public Optional<Matrix<N3, N3>> getCameraIntrinsics() {
    //     return camera.getCameraMatrix();
    // }

    // /**Gets the distortion coefficients matrix of the camera.
    //  * 
    //  * @return The distortion coefficients.
    //  */
    // public Optional<Matrix<N8, N1>> getDistortionCoefficients() {
    //     return camera.getDistCoeffs();
    // }

    // /**Gets the standard deviations of the vision measurements.
    //  * 
    //  * @return A matrix containing the standard deviations of the vision measurements.
    //  */
    // public Matrix<N3, N1> getVisionStandardDeviations() {
    //     return visionStandardDeviations;
    // }

    /**Calculates the standard deviations of the vision measurements.
     * 
     * @param pose The robot pose.
     * @param tags The tags used for the robot pose.
     * @return A matrix containing the standard deviations of the vision measurements.
     */
    public Matrix<N3, N1> calculateVisionStandardDeviations(Pose3d pose, List<PhotonTrackedTarget> tags) {
        int numTags = 0;
        double avgDistance = 0.0;

        //Since we have a list of tags, use an enhanced for loop
        for(PhotonTrackedTarget tag : tags) {
            //Get the pose of the tag
            Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(tag.getFiducialId());

            //If the pose cannot be obtained, go to the next iteration of the loop
            if(tagPose.isEmpty()) {
                continue;
            }

            numTags++; //Add one to the number of tags checked
            //Add the distance from the robot to the tag
            avgDistance += tagPose.get().toPose2d()
                .getTranslation()
                .getDistance(pose.toPose2d().getTranslation());
        }

        /*
         * If no tags were able to be checked, fill the vision standard deviations with
         * the max double value. This will tell the robot that the vision readings are
         * extremely unreliable and should not be used.
         */
        if(numTags == 0) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        //Find the average distance of the tags
        avgDistance /= numTags;
        double standardDeviation;

        //If there is only one tag and it is over 4 meters, do not trust the estimated pose
        if(numTags == 1 && avgDistance >= 4.0) {
            standardDeviation = Double.MAX_VALUE;
        //If there are multiple tags and it is over 4 meters, scale it by distance
        }else if(numTags >= 2 && avgDistance >= 4.0) {
            standardDeviation = 0.2 * (avgDistance * avgDistance);
        //If there are multiple tags and it is less than 4 meters, set it to 0.2
        }else if(numTags >= 2 && avgDistance < 4.0) {
            standardDeviation = 0.2;
        //If there is one tag and it is less than 4 meters, scale it by distance
        }else{
            standardDeviation = 0.5 + (0.3 * (avgDistance * avgDistance));
        }

        //Return a matrix with the calculated values
        return VecBuilder.fill(standardDeviation, standardDeviation, Double.MAX_VALUE);
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

    public double getTimestamp() {
        return timestamp;
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
            for(PhotonTrackedTarget tag : result.getTargets()) {
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
                            //This checks if the current alliance is red or blue
                            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                                //If red, it tries to match the tag IDs with the red tag IDs
                                if(tag.getFiducialId() == tagID.getRedHubTagID()) {
                                    trackedHubTag = tag;
                                    return true;
                                }
                            }else{
                                //If blue, it tries to match the tag IDs with the blue tag IDs
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
                            //This checks if the current alliance is red or blue
                            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                                //If red, it tries to match the tag IDs with the red tag IDs
                                if(tag.getFiducialId() == tagID.getRedOutpostTagID()) {
                                    return true;
                                }
                            }else{
                                //If blue, it tries to match the tag IDs with the blue tag IDs
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
                            //This checks if the current alliance is red or blue
                            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                                //If red, it tries to match the tag IDs with the red tag IDs
                                if(tag.getFiducialId() == tagID.getRedTowerTagID()) {
                                    return true;
                                }
                            }else{
                                //If blue, it tries to match the tag IDs with the blue tag IDs
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

    /**Checks if the current tracked hub tag is the centered tag.
     * 
     * @return {@code true} if the tag is centered, {@code false} otherwise.
     */
    public boolean trackedHubTagIsCentered() {
        //This checks if the current alliance is red or blue
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            for(centerHubTagsIDs tagID : centerHubTagsIDs.values()) {
                //If red, it tries to match the tag IDs with the red tag IDs
                if(trackedHubTag.getFiducialId() == tagID.getCenterRedHubTagID()) {
                    return true;
                }
            }
        }else{
            for(centerHubTagsIDs tagID : centerHubTagsIDs.values()) {
                //If blue, it tries to match the tag IDs with the blue tag IDs
                if(trackedHubTag.getFiducialId() == tagID.getCenterBlueHubTagID()) {
                    return true;
                }
            }
        }

        return false;
    }

    /**Checks if the hub is currently being checked.
     * This is determined by if a centered hub tag is currently being seen.
     * 
     * @return {@code true} if tracking, {@code false} otherwise.
     */
    public boolean isTrackingHub() {
        return currentlyTrackingHub;
    }

    /**Gets the hub tag that is currently being seen.
     * This method is for tracking the hub for the turret and shooter.
     * 
     * @return The hub tag that is currently being seen by the turret camera.
     */
    public Optional<PhotonTrackedTarget> getTrackedHubTag() {
        return Optional.of(trackedHubTag);
    }
}