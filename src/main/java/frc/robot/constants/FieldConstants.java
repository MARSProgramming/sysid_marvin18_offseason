package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
    
        static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    }

    public static List<Pose2d> getAprilTagPose2dList() {
        int[] tagIntegers = new int[0];
        tagIntegers = isBlueAlliance() ? (tagIntegers = VisionFiducials.BLUE_CORAL_TAGS) : (tagIntegers = VisionFiducials.RED_CORAL_TAGS);
        List<Pose2d> tags = new ArrayList<>();
        for (int i = 0; i < tagIntegers.length; i++) {
            int tagInteger = tagIntegers[i];
            Pose2d tagPose = field.getTagPose(tagInteger).get().toPose2d();
            tags.add(tagPose);
        }
        return tags;
    }


    public static class VisionFiducials {
        // F,B,R,L with respect to their driver stations.
        public static final int RED_RIGHT_FEEDER_TAG = 1;
        public static final int RED_LEFT_FEEDER_TAG = 2;
        public static final int BLUE_RIGHT_FEEDER_TAG = 13;
        public static final int BLUE_LEFT_FEEDER_TAG = 12;

        public static final int RED_PROCESSOR_TAG = 3;
        public static final int BLUE_PROCESSOR_TAG = 16;

        public static final int BLUE_BACK_CLIMBING_TAG = 4;
        public static final int RED_FRONT_CLIMBING_TAG = 5;
        public static final int BLUE_FRONT_CLIMBING_TAG = 14;
        public static final int RED_BACK_CLIMBING_TAG = 15;

        public static final int RED_FL_CORAL_TAG = 6;
        public static final int RED_F_CORAL_TAG = 7;
        public static final int RED_FR_CORAL_TAG = 8;
        public static final int RED_BL_CORAL_TAG = 9;
        public static final int RED_B_CORAL_TAG = 10;
        public static final int RED_BR_CORAL_TAG = 11;

        public static final int BLUE_FR_CORAL_TAG = 17;
        public static final int BLUE_F_CORAL_TAG = 18;
        public static final int BLUE_FL_CORAL_TAG = 19;
        public static final int BLUE_BL_CORAL_TAG = 20;
        public static final int BLUE_B_CORAL_TAG = 21;
        public static final int BLUE_BR_CORAL_TAG = 22;

        public static final int[] RED_CORAL_TAGS = { 6, 7, 8, 9, 10, 11 };
        public static final int[] BLUE_CORAL_TAGS = { 17, 18, 19, 20, 21, 22 };
        public static final int[] RED_FEEDER_TAGS = { 1, 2 };
        public static final int[] BLUE_FEEDER_TAGS = { 12, 13 };
        public static final int[] RED_SIDE_CLIMB_TAGS = { 4, 5 };
        public static final int[] BLUE_SIDE_CLIMB_TAGS = { 14, 15 };
        public static final int[] PROCESSOR_TAGS = { 1, 15 };
    }
}

