// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class AprilTagIDs {

    /**Holds all IDs for the hub.
     * 
     */
    public enum hubTagsIDs {
        FRONT_CENTER(25, 9),
        FRONT_OFFSET(26, 10),
        RIGHT_CENTER(27, 11),
        RIGHT_OFFSET(18, 2),
        LEFT_CENTER(21, 5),
        LEFT_OFFSET(24, 8),
        BACK_CENTER(19, 3),
        BACK_OFFSET(20, 4);

        private int blueHubTagID;
        private int redHubTagID;

        hubTagsIDs(int blueHubTagID, int redHubTagID) {
            this.blueHubTagID = blueHubTagID;
            this.redHubTagID = redHubTagID;
        }

        /**
         * Gets the ID for hub tag when on blue alliance.
         */
        public int getBlueHubTagID() {
            return blueHubTagID;
        }

        /**
         * Gets the ID for hub tag when on red alliance.
         */
        public int getRedHubTagID() {
            return redHubTagID;
        }
    }

    /**Holds all center tag IDs for the hub.
     * 
     */
    public enum centerHubTagsIDs {
        FRONT(25, 9),
        RIGHT(27, 11),
        LEFT(21, 5),
        BACK(19, 3);

        private int centerBlueHubTagID;
        private int centerRedHubTagID;

        centerHubTagsIDs(int centerBlueHubTagID, int centerRedHubTagID) {
            this.centerBlueHubTagID = centerBlueHubTagID;
            this.centerRedHubTagID = centerRedHubTagID;
        }

        /**Gets the ID for center hub tag when on red alliance.
         * 
         */
        public int getCenterRedHubTagID() {
            return centerRedHubTagID;
        }

        /**Gets the ID for center hub tag when on blue alliance.
         * 
         */
        public int getCenterBlueHubTagID() {
            return centerBlueHubTagID;
        }
    }

    /**Holds all IDs for the outpost.
     * 
     */
    public enum outpostTagsIDs {
        CENTER(29, 13),
        OFFSET(30, 14);

        private int blueOutpostTagID;
        private int redOutpostTagID;

        outpostTagsIDs(int blueOutpostTagID, int redOutpostTagID) {
            this.blueOutpostTagID = blueOutpostTagID;
            this.redOutpostTagID = redOutpostTagID;
        }

        /**Gets the ID for outpost tag when on blue alliance.
         * 
         */
        public int getBlueOutpostTagID() {
            return blueOutpostTagID;
        }

        /**Gets the ID for outpost tag when on red alliance.
         * 
         */
        public int getRedOutpostTagID() {
            return redOutpostTagID;
        }
    }

    /**Holds all IDs for the tower.
     * 
     */
    public enum towerTagsIDs {
        CENTER(31, 15),
        OFFSET(32, 16);

        private int blueTowerTagID;
        private int redTowerTagID;

        towerTagsIDs(int blueTowerTagID, int redTowerTagID) {
            this.blueTowerTagID = blueTowerTagID;
            this.redTowerTagID = redTowerTagID;
        }

        /**Gets the ID for tower tag when on blue alliance.
         * 
         */
        public int getBlueTowerTagID() {
            return blueTowerTagID;
        }

        /**Gets the ID for tower tag when on red alliance.
         * 
         */
        public int getRedTowerTagID() {
            return redTowerTagID;
        }
    }

}
