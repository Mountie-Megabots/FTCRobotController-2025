package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class PedroHelper {
    private static boolean isRed = false;

    public static void onRedAlliance() {
        isRed = true;
    }

    public static void onBlueAlliance() {
        isRed = false;
    }
    public static Path runPath(Pose setPose1, Pose setPose2){
        //if isRed is true, then mirror the pose. Otherwise, just use the original
        Pose pose1 = (isRed) ? setPose1.mirror() : setPose1;
        Pose pose2 = (isRed) ? setPose2.mirror() : setPose2;
        //Create the path and return using the poses
        Path path = new Path(new BezierLine(pose1, pose2));
        path.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());
        return path;
    }

    public static Path runPath(BezierLine line) {
        //if isRed is true, then mirror the pose. Otherwise, just use the original
        Pose pose1 = (isRed) ? line.getFirstControlPoint().mirror() : line.getFirstControlPoint();
        Pose pose2 = (isRed) ? line.getLastControlPoint().mirror() : line.getLastControlPoint();
        //Create the path and return using the poses
        Path path = new Path(new BezierLine(pose1, pose2));
        path.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());
        return path;
    }
}
