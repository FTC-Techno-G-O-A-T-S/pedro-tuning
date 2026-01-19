package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Disabled
public class pedroAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(32, 136, 0); // Start Pose of our robot
    private final Pose shootPose = new Pose(3, 136, 0);

    private Path shootPre;
    //private PathChain shootPre;

    public void buildPaths() {
        shootPre = new Path(new BezierLine(startPose, shootPose));
    }
public void autoPathUpdate(){
    switch (pathState) {
        case 0:
            follower.followPath(shootPre);
            //setPathState(1);
            break;
        case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            if (!follower.isBusy()) {
                /* Score Preload */
                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                //follower.followPath(grabPickup1,true);
                //setPathState(2);
                telemetry.addLine("next path");
            }
            break;

        }
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autoPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        telemetry.addLine("Robot Init");
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
       // setPathState(0);
        // ^dont know when setPathState errors
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
