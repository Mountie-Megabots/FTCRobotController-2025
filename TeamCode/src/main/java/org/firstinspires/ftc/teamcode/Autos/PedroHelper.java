package org.firstinspires.ftc.teamcode.Autos;

import com.google.gson.JsonObject;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Arrays;

public class PedroHelper {
    private static boolean isRed = false;

    //this will be used for the creation methods where the heading interpolation will be an argument
    public static enum headingInterp {
        linear,
        constant,
        tangential
    }

    public static void onRedAlliance() {
        isRed = true;
    }

    public static void onBlueAlliance() {
        isRed = false;
    }

    public static Path createLine(Pose setPose1, Pose setPose2){
        //if isRed is true, then mirror the pose. Otherwise, just use the original
        Pose pose1 = (isRed) ? setPose1.mirror() : setPose1;
        Pose pose2 = (isRed) ? setPose2.mirror() : setPose2;
        //Create the path and return using the poses
        Path path = new Path(new BezierLine(pose1, pose2));
        path.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());
        return path;
    }

    public static Path createCurve(Pose[] arrayOfPoses){
        //change to array list to make it easier to use
        ArrayList<Pose> pathControlPoints = new ArrayList<>(Arrays.asList(arrayOfPoses));
        //if isRed is true, then mirror the pose. Otherwise, just use the original
        pathControlPoints.trimToSize();
        pathControlPoints.replaceAll((isRed) ? Pose::mirror : Pose::copy);

        //Create the path and return using the poses
        Path path = new Path(new BezierCurve(pathControlPoints));
        path.setLinearHeadingInterpolation(pathControlPoints.get(0).getHeading(), pathControlPoints.get(pathControlPoints.size() - 1).getHeading());
        return path;
    }

    public static Path createLine(BezierLine line) {
        //if isRed is true, then mirror the pose. Otherwise, just use the original
        Pose pose1 = (isRed) ? line.getFirstControlPoint().mirror() : line.getFirstControlPoint();
        Pose pose2 = (isRed) ? line.getLastControlPoint().mirror() : line.getLastControlPoint();
        //Create the path and return using the poses
        Path path = new Path(new BezierLine(pose1, pose2));
        path.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());
        return path;
    }

    public static PathChain readPedroPath(String ppJson) throws JSONException {
        //create all items needed
        JSONObject json = new JSONObject(ppJson);
        JSONObject jsonPart = json.getJSONObject("startPoint");
        ArrayList<Path> pathList = new ArrayList<>();
        JSONArray lineArray = json.getJSONArray("lines");

        //set the last pose, which is the start for now. Last pose will be used for creating the paths
        Pose lastPose = new Pose(jsonPart.getInt("x"), jsonPart.getInt("y"), Math.toRadians(jsonPart.getInt("startDeg")));

        for (int i = 1; i < json.length(); i++) {
            //set the json object for this path
            jsonPart = lineArray.getJSONObject(i);
            //get the endpoint of this path
            Pose endPoint = new Pose(jsonPart.getInt("x"), jsonPart.getInt("y"), Math.toRadians(jsonPart.getInt("startDeg")));
            //get the array of control points
            JSONArray controlPointArray = jsonPart.getJSONArray("controlPoints");

            //if this is a curve (has control points) then go down the path of creating one
            if (!controlPointArray.isNull(0)) {
                Pose[] poseArray = new Pose[controlPointArray.length()];
                poseArray[0] = lastPose.copy();
                //for each object, get it and add it to the array
                for (int e = 1; e < controlPointArray.length(); e++) {
                    JSONObject controlPoint = controlPointArray.getJSONObject(e);
                    poseArray[e] = new Pose(controlPoint.getInt("x"), controlPoint.getInt("y"));
                }
                //add the endpoint
                poseArray[controlPointArray.length() - 1] = endPoint.copy();
                lastPose = endPoint.copy();
                //add the curve to the pathchain
                pathList.add(createCurve(poseArray));
            } else {
                //if it is just a line, set as such and add to pathchain
                pathList.add(createLine(lastPose, endPoint));
            }
        }
        // return the pathchain
        return new PathChain(pathList);
    }
}
