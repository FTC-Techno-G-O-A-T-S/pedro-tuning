package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class makeautowork extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    //making states
    public enum PathState {
        //Start pos_end pos
        //Drive > movement
        //Shoot > try to score
        DriveStartposShootpos,
        Shoot_Pre,

    }

    PathState pathState;

    //setting postions
    private final Pose startPose = new Pose(32,136, Math.toRadians(180));
    private final Pose shootPose = new Pose(58,136,Math.toRadians(180));


    private PathChain driveStartShoot;

    //building paths
    public void buildPaths()  {
        //put in start cords then end cords
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
    }

    //moving through states
    public void statePathUpdate() {
        switch (pathState) {
            case DriveStartposShootpos:
                follower.followPath(driveStartShoot, true);
                telemetry.addLine("drive mode");
                setPathState(PathState.Shoot_Pre); //reset timer and make new state
                break;
            case Shoot_Pre:
                //check if follower done with path
                //and time has passed
                telemetry.addLine("shoot mode");
                if (!follower.isBusy()) {
                    //add shooting code
                    telemetry.addLine("shooting now");
                }
                break;
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    //allows to transition states
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
        pathState = PathState.DriveStartposShootpos;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //add any other things that need to be init
        telemetry.addLine("Robot Init");
        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading: deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time: sec", pathTimer.getElapsedTimeSeconds());
    }
}
