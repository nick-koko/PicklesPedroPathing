package org.firstinspires.ftc.teamcode.PedroAuton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.actions.ClawActions;
import org.firstinspires.ftc.teamcode.actions.DualSlideActions;
import org.firstinspires.ftc.teamcode.actions.FollowPathActions;
import org.firstinspires.ftc.teamcode.actions.IntakeArmActions;
import org.firstinspires.ftc.teamcode.actions.IntakeSlideAction;
import org.firstinspires.ftc.teamcode.actions.IntakeservoSpinnerActions;
import org.firstinspires.ftc.teamcode.actions.OuttakeDumpActions;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Four Specimen Auton Pedro Test")
public class FourSpecimenAutoPedro extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    Pose initialPoseRightSideSpecimen = new Pose(9.00, 62.75, Math.toRadians(180));
    Pose startPose = initialPoseRightSideSpecimen;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain Plebeian1, Goober2, OhDip3;

    OuttakeDumpActions outtakeDump = new OuttakeDumpActions();
    DualSlideActions outtakeSlide =  new DualSlideActions();
    ClawActions outtakeClaw = new ClawActions();
    IntakeSlideAction intakeSlide = new IntakeSlideAction();
    IntakeArmActions intakeArm = new IntakeArmActions();
    IntakeservoSpinnerActions intakeSpinner = new IntakeservoSpinnerActions();



    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        Plebeian1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(9.0, 62.75, Point.CARTESIAN),
                                new Point(38, 62.641, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Goober2 = follower.pathBuilder()

                .addPath(
                        new BezierCurve(
                                new Point(41.417, 62.641, Point.CARTESIAN),
                                new Point(7.369, 33.015, Point.CARTESIAN),
                                new Point(53.208, 36.553, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(53.208, 36.553, Point.CARTESIAN),
                                new Point(73.990, 27.415, Point.CARTESIAN),
                                new Point(50.702, 22.845, Point.CARTESIAN),
                                new Point(17.392, 22.993, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(17.392, 22.993, Point.CARTESIAN),
                                new Point(58.809, 29.331, Point.CARTESIAN),
                                new Point(57.629, 22.256, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(57.629, 22.256, Point.CARTESIAN),
                                new Point(61.314, 10.170, Point.CARTESIAN),
                                new Point(14.444, 13.855, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(14.444, 13.855, Point.CARTESIAN),
                                new Point(69.863, 17.097, Point.CARTESIAN),
                                new Point(57.629, 7.222, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(57.629, 7.222, Point.CARTESIAN),
                                new Point(9.138, 7.517, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();

        // Feedback to Driver Hub
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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        outtakeSlide.init(hardwareMap);
        outtakeDump.init(hardwareMap);
        intakeArm.init(hardwareMap);
        intakeSpinner.init(hardwareMap);
        intakeSlide.init(hardwareMap);
        intakeSlide.resetSlide();
        outtakeSlide.resetSlide();
        outtakeClaw.init(hardwareMap);

        buildPaths();

        Actions.runBlocking(outtakeClaw.close());
        Actions.runBlocking(intakeArm.armDrive());
        Actions.runBlocking(outtakeDump.downPosition());

        globalRobotDataPedro.hasAutonRun = true;

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

        FollowPathActions followPlebian1Path = new FollowPathActions(Plebeian1, follower);
        FollowPathActions followGoober2Path = new FollowPathActions(Goober2, follower);

        Actions.runBlocking(
                new SequentialAction(
                        followPlebian1Path,
                        new SleepAction(5),
                        followGoober2Path
                        )
        );

        globalRobotDataPedro.autonPose = follower.getPose();

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

