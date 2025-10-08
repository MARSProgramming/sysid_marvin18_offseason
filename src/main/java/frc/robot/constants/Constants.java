package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Constants {
        public static final Transform3d feederRobotToCam = new Transform3d(Inches.of(-6),
        Inches.of(9.06), Inches.of(11.55), new Rotation3d(Degrees.of(0),
        Degrees.of(0), Degrees.of(125)));

        public static final Transform3d reefRobotToCam = new Transform3d(Inches.of(-6.3), Inches.of(9.49), Inches.of(13.44),
        new Rotation3d(Degrees.of(-10), Degrees.of(0), Degrees.of(90)));
}
