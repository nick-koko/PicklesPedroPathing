package org.firstinspires.ftc.teamcode.PedroAuton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actions.ClawActions;
import org.firstinspires.ftc.teamcode.actions.DualSlideActions;
import org.firstinspires.ftc.teamcode.actions.FollowPathActions;
import org.firstinspires.ftc.teamcode.actions.IntakeArmActions;
import org.firstinspires.ftc.teamcode.actions.IntakeSlideAction;
import org.firstinspires.ftc.teamcode.actions.IntakeWristActions;
import org.firstinspires.ftc.teamcode.actions.IntakeservoSpinnerActions;
import org.firstinspires.ftc.teamcode.actions.OuttakeArmMoverActions;

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
@Config
@Autonomous(name = "Five Specimen Auton Pedro Test")
public class FiveSpecimenAutoPedro extends OpMode {

    private Follower follower;
    private Telemetry telemetryA;

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
    public static Pose dropFirstSpecimen = new Pose(39.0, 62.641, Math.toRadians(180));
    public static Pose moveAndTurnToFirstSample1 = new Pose(35.8, 43.6, Math.toRadians(270));
    public static Pose moveAndTurnToFirstSample1_CP = new Pose(33.46, 56.0, Math.toRadians(270));
    public static Pose moveAroundSubToFirstSample = new Pose(52.8, 34.5, Math.toRadians(270));
    public static Pose moveAroundSubControlPoint1 = new Pose(36.1, 34.9, Math.toRadians(270));
    public static Pose moveAndTurnToFirstSample2 = new Pose(57.5, 24.6, Math.toRadians(0));
    public static Pose moveAndTurnToFirstSample2_CP = new Pose(66.32, 31.8, Math.toRadians(0));

    Pose startPose = initialPoseRightSideSpecimen;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain Plebeian1, Goober2, Goober2Returns, OhDip3, DumbName4, OhDip5, DumbName6,  OhDip7, DumbName8, OhDip9, BackToZone;

    OuttakeArmMoverActions outtakeDump = new OuttakeArmMoverActions();
    DualSlideActions outtakeSlide =  new DualSlideActions();
    ClawActions outtakeClaw = new ClawActions();
    IntakeSlideAction intakeSlide = new IntakeSlideAction();
    IntakeArmActions intakeArm = new IntakeArmActions();
    IntakeservoSpinnerActions intakeSpinner = new IntakeservoSpinnerActions();
    IntakeWristActions intakeWrist = new IntakeWristActions();



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
                                new Point(initialPoseRightSideSpecimen),
                                new Point(dropFirstSpecimen)
                        )
                )
                .setConstantHeadingInterpolation(initialPoseRightSideSpecimen.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        Goober2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(dropFirstSpecimen),
                                new Point(moveAndTurnToFirstSample1_CP),
                                new Point(moveAndTurnToFirstSample1)
                        )
                )
                .setLinearHeadingInterpolation(dropFirstSpecimen.getHeading(), moveAndTurnToFirstSample1.getHeading())

                .addPath(
                        new BezierCurve(
                                new Point(moveAndTurnToFirstSample1),
                                new Point(moveAroundSubControlPoint1),
                                new Point(moveAroundSubToFirstSample)
                        )
                )
                .setConstantHeadingInterpolation(moveAroundSubToFirstSample.getHeading())

                .addPath(
                        new BezierCurve(
                                new Point(moveAroundSubToFirstSample),
                                new Point(moveAndTurnToFirstSample2_CP),
                                new Point(moveAndTurnToFirstSample2)
                        )
                )
                .setLinearHeadingInterpolation(moveAroundSubToFirstSample.getHeading(), moveAndTurnToFirstSample2.getHeading())

                .addPath(
                        new BezierCurve(
                                new Point(moveAndTurnToFirstSample2),
                                new Point(47.312, 21.814, Point.CARTESIAN),
                                new Point(30.805, 23.140, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(30.805, 23.140, Point.CARTESIAN),
                                new Point(54.092, 24.319, Point.CARTESIAN),
                                new Point(55.419, 17.982, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                .addPath(
                        new BezierCurve(
                                new Point(55.419, 17.982, Point.CARTESIAN),
                                new Point(58.956, 11.644, Point.CARTESIAN),
                                new Point(31.5, 14.149, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                .addPath(
                        new BezierCurve(
                                new Point(31.5, 14.149, Point.CARTESIAN),
                                new Point(64.852, 15.329, Point.CARTESIAN),
                                new Point(60.135, 7.812, Point.CARTESIAN),
                                new Point(54.387, 8.549, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                .addPath(
                        new BezierLine(
                                new Point(54.387, 8.549, Point.CARTESIAN),
                                new Point(31.5, 8.254, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                .addPath(
                        new BezierCurve(
                                new Point(31.5, 8.254, Point.CARTESIAN),
                                new Point(15.771, 6.780, Point.CARTESIAN),
                                new Point(24.172, 36.111, Point.CARTESIAN),
                                new Point(19.433, 34.637, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(7)
                .build();

        Goober2Returns = follower.pathBuilder()


                .addPath(
                        new BezierLine(
                                new Point(19.433, 34.637, Point.CARTESIAN),
                                new Point(12.033, 34.637, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .setZeroPowerAccelerationMultiplier(3)
                .build();

        OhDip3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(10.433, 34.637, Point.CARTESIAN),
                                new Point(22.5, 70.9, Point.CARTESIAN),
                                new Point(39.0, 72, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();


        DumbName4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(39.000, 72.000, Point.CARTESIAN),
                                new Point(21.372, 33.310, Point.CARTESIAN),
                                new Point(19.433, 34.637, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))

                .addPath(
                        new BezierLine(
                                new Point(19.433, 34.637, Point.CARTESIAN),
                                new Point(12.033, 34.637, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(3)
                .build();
        OhDip5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(10.433, 34.637, Point.CARTESIAN),
                                new Point(22.5, 70.9, Point.CARTESIAN),
                                new Point(39.0, 72, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();


        DumbName6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(39.000, 72.000, Point.CARTESIAN),
                                new Point(21.372, 33.310, Point.CARTESIAN),
                                new Point(19.433, 34.637, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))

                .addPath(
                        new BezierLine(
                                new Point(19.433, 34.637, Point.CARTESIAN),
                                new Point(12.033, 34.637, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(3)
                .build();
        OhDip7 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(10.433, 34.637, Point.CARTESIAN),
                                new Point(22.5, 70.9, Point.CARTESIAN),
                                new Point(39.0, 72, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();


        DumbName8 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(39.000, 72.000, Point.CARTESIAN),
                                new Point(21.372, 33.310, Point.CARTESIAN),
                                new Point(19.433, 34.637, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))

                .addPath(
                        new BezierLine(
                                new Point(19.433, 34.637, Point.CARTESIAN),
                                new Point(12.033, 34.637, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(3)
                .build();
        OhDip9 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(10.433, 34.637, Point.CARTESIAN),
                                new Point(22.5, 70.9, Point.CARTESIAN),
                                new Point(39.0, 72, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();


        BackToZone = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(39.000, 72.000, Point.CARTESIAN),
                                new Point(17.834, 47.312, Point.CARTESIAN),
                                new Point(12.823, 28.299, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(7)
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

        follower = new Follower(hardwareMap, FConstants.class,LConstants.class);
        follower.setStartingPose(startPose);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        outtakeSlide.init(hardwareMap);
        outtakeDump.init(hardwareMap);
        intakeArm.init(hardwareMap);
        intakeSpinner.init(hardwareMap);
        intakeSlide.init(hardwareMap);
        intakeSlide.resetSlide();
        outtakeSlide.resetSlide();
        outtakeClaw.init(hardwareMap);
        intakeWrist.init(hardwareMap);

        buildPaths();

        Actions.runBlocking(outtakeClaw.close());
        Actions.runBlocking(intakeArm.armDrive());
        Actions.runBlocking(outtakeDump.downPosition());
        intakeWrist.wristDrive();

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

        FollowPathActions followPlebian1Path = new FollowPathActions(Plebeian1, follower, true, telemetryA);
        FollowPathActions followGoober2Path = new FollowPathActions(Goober2, follower, false, telemetryA);
        FollowPathActions followGoober2ReturnsPath = new FollowPathActions(Goober2Returns, follower, false, telemetryA);
        FollowPathActions followOhDip3Path = new FollowPathActions(OhDip3, follower, true, telemetryA);
        FollowPathActions followDumbName4Path = new FollowPathActions(DumbName4, follower, false, telemetryA);
        FollowPathActions followOhDip5Path = new FollowPathActions(OhDip5, follower, true, telemetryA);
        FollowPathActions followDumbName6Path = new FollowPathActions(DumbName6, follower, false, telemetryA);
        FollowPathActions followOhDip7Path = new FollowPathActions(OhDip7, follower, true, telemetryA);
        FollowPathActions followDumbName8Path = new FollowPathActions(DumbName8, follower, false, telemetryA);
        FollowPathActions followOhDip9Path = new FollowPathActions(OhDip9, follower, true, telemetryA);
        FollowPathActions followBackToZonePath = new FollowPathActions(BackToZone, follower, false, telemetryA);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                followPlebian1Path,         //Drive to bar to drop First Specimen
                                outtakeSlide.specimenDrop(),
                                intakeArm.armTransfer()
                        ),
                        //outtakeClaw.dropPosition(),
                        outtakeClaw.dropPosition(),
                        outtakeSlide.specimenDropDown(),    //Place first specimen
                        new ParallelAction(
                                followGoober2Path,          //Push in samples
                                new SequentialAction(
                                        new SleepAction(.1),
                                        outtakeClaw.open()
                                )
                        ),
                        followGoober2ReturnsPath, //😎👌👌👌 ༼ つ ◕_◕ ༽つ //Drive to pickup second spec
                        outtakeClaw.close(),
                        outtakeSlide.extendAction(),
                        new ParallelAction(
                                followOhDip3Path,           //Drive to drop 2nd Spec
                                outtakeSlide.specimenDrop(),
                                outtakeClaw.dropPosition()
                        ),
                        outtakeSlide.specimenDropDown(),    //Place 2nd Specimen
                        new ParallelAction(
                                followDumbName4Path,        //Drive to get 3rd Spec
                                new SequentialAction(
                                        new SleepAction(.1),
                                        outtakeClaw.open()
                                )
                        ),
                        outtakeClaw.close(),
                        outtakeSlide.extendAction(),
                        new ParallelAction(
                                followOhDip5Path,           //Drive to place 3rd Spec
                                outtakeSlide.specimenDrop(),
                                outtakeClaw.dropPosition()
                        ),
                        outtakeSlide.specimenDropDown(),    //Place 3rd specimen
                        new ParallelAction(
                                followDumbName6Path,        //Drive to get 4th specimen
                                new SequentialAction(
                                        new SleepAction(.1),
                                        outtakeClaw.open()
                                )
                        ),
                        outtakeClaw.close(),
                        outtakeSlide.extendAction(),
                        new ParallelAction(
                                followOhDip7Path,
                                outtakeSlide.specimenDrop(),
                                outtakeClaw.dropPosition()
                        ),
                        outtakeSlide.specimenDropDown(),
                        new ParallelAction(
                                followDumbName8Path,
                                new SequentialAction(
                                        new SleepAction(.1),
                                        outtakeClaw.open()
                                )
                        ),
                        outtakeClaw.close(),
                        outtakeSlide.extendAction(),
                        new ParallelAction(
                                followOhDip9Path,
                                outtakeSlide.specimenDrop(),
                                outtakeClaw.dropPosition()
                        ),
                        outtakeSlide.specimenDropDown(),
                        new ParallelAction(
                                followBackToZonePath,
                                new SequentialAction(
                                        new SleepAction(.1),
                                        outtakeClaw.open()
                                )
                        )
                )
        );

        globalRobotDataPedro.autonPose = follower.getPose();

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

