package org.firstinspires.ftc.teamcode.PedroAuton;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.mechanisms.ClimbingHooks;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeWrist;

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

@Autonomous(name = "Four Sample Auton Pedro Test")
public class FourSampleAutoPedro extends OpMode {

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
    Pose initialPoseLeftSideSample = new Pose(7.5, 79.5, Math.toRadians(-90));
    Pose dumpPoseLeftSideSample = new Pose(18.500, 126.500, Math.toRadians(-45));
    Pose dumpPoseLeftSideFromSamples = new Pose(16.500, 125.500, Math.toRadians(-40));
    Pose firstSamplePoseLeftSideSample1 = new Pose(45.544, 106.0, Math.toRadians(90));
    Pose firstSamplePoseLeftSideSample2 = new Pose(45.544, 107.447, Math.toRadians(90));
    Pose secondSamplePoseLeftSideSample = new Pose(45.544, 120.447, Math.toRadians(90));
    Pose thirdSamplePoseLeftSideSample = new Pose(45.544, 128.447, Math.toRadians(90));
    Pose goToSubPose1 = new Pose(62, 98, Math.toRadians(-90));
    Pose goToSubPose1_CP = new Pose(62.5, 124, Math.toRadians(-90));
    Pose startPose = initialPoseLeftSideSample;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain StartToBucket1, BucketToSample2, StopAtSample, SampleToBucket3, BucketToSample4, SampleToBucket5, BucketToSample6, SampleToBucket7, GotoSub8;
    private PathChain StartToBucketUseCallbacks1, BucketToSampleUseCallbacks2, SampleToBucketUseCallbacks3;

    OuttakeArmMoverActions outtakeDump = new OuttakeArmMoverActions();
    DualSlideActions outtakeSlide =  new DualSlideActions();
    ClawActions outtakeClaw = new ClawActions();
    IntakeSlideAction intakeSlide = new IntakeSlideAction();
    IntakeArmActions intakeArm = new IntakeArmActions();
    IntakeservoSpinnerActions intakeSpinner = new IntakeservoSpinnerActions();
    IntakeWristActions intakeWrist = new IntakeWristActions();
    ClimbingHooks climbingServo = new ClimbingHooks();


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
        StartToBucket1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine( //BezierLines are lines!!!
                                new Point(initialPoseLeftSideSample),
                                new Point(dumpPoseLeftSideSample)
                        )
                )
                .setLinearHeadingInterpolation(initialPoseLeftSideSample.getHeading(), dumpPoseLeftSideSample.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        BucketToSample2 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(dumpPoseLeftSideSample),
                                new Point(firstSamplePoseLeftSideSample1)
                        )
                )
                .setLinearHeadingInterpolation(dumpPoseLeftSideSample.getHeading(), firstSamplePoseLeftSideSample1.getHeading())
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(firstSamplePoseLeftSideSample1),
                                new Point(firstSamplePoseLeftSideSample2)
                        )
                )
                .setConstantHeadingInterpolation(firstSamplePoseLeftSideSample1.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();
        BucketToSample4 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(dumpPoseLeftSideFromSamples),
                                new Point(firstSamplePoseLeftSideSample2)
                        )
                )
                .setLinearHeadingInterpolation(dumpPoseLeftSideFromSamples.getHeading(), firstSamplePoseLeftSideSample2.getHeading())
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(firstSamplePoseLeftSideSample2),
                                new Point(secondSamplePoseLeftSideSample)
                        )
                )
                .setConstantHeadingInterpolation(firstSamplePoseLeftSideSample2.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        BucketToSample6 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(dumpPoseLeftSideFromSamples),
                                new Point(secondSamplePoseLeftSideSample)
                        )
                )
                .setLinearHeadingInterpolation(dumpPoseLeftSideFromSamples.getHeading(), secondSamplePoseLeftSideSample.getHeading())
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(secondSamplePoseLeftSideSample),
                                new Point(thirdSamplePoseLeftSideSample)
                        )
                )
                .setConstantHeadingInterpolation(secondSamplePoseLeftSideSample.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        StopAtSample = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(firstSamplePoseLeftSideSample2),
                                new Point(firstSamplePoseLeftSideSample2)
                        )
                )
                .setLinearHeadingInterpolation(firstSamplePoseLeftSideSample2.getHeading(), firstSamplePoseLeftSideSample2.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        SampleToBucket3 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(firstSamplePoseLeftSideSample2),
                                new Point(dumpPoseLeftSideFromSamples)
                        )
                )
                .setLinearHeadingInterpolation(firstSamplePoseLeftSideSample2.getHeading(), dumpPoseLeftSideFromSamples.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        SampleToBucket5 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(secondSamplePoseLeftSideSample),
                                new Point(dumpPoseLeftSideFromSamples)
                        )
                )
                .setLinearHeadingInterpolation(secondSamplePoseLeftSideSample.getHeading(), dumpPoseLeftSideFromSamples.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        SampleToBucket7 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(thirdSamplePoseLeftSideSample),
                                new Point(dumpPoseLeftSideFromSamples)
                        )
                )
                .setLinearHeadingInterpolation(thirdSamplePoseLeftSideSample.getHeading(), dumpPoseLeftSideFromSamples.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        GotoSub8 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(dumpPoseLeftSideFromSamples),
                                new Point(goToSubPose1_CP),
                                new Point(goToSubPose1)
                        )
                )
                .setLinearHeadingInterpolation(dumpPoseLeftSideFromSamples.getHeading(), goToSubPose1.getHeading())
                .setZeroPowerAccelerationMultiplier(5)
                .build();
        // The following use callbacks to schedule things
        StartToBucketUseCallbacks1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(initialPoseLeftSideSample),
                                new Point(dumpPoseLeftSideSample)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(initialPoseLeftSideSample.getHeading()), dumpPoseLeftSideSample.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .addParametricCallback(0.0, ()-> outtakeSlide.high())
                .addParametricCallback(0.9, () -> outtakeDump.bucketPosition())
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
        climbingServo.init(hardwareMap);

        buildPaths();

        Actions.runBlocking(outtakeClaw.open());
        Actions.runBlocking(intakeArm.armDrive());
        Actions.runBlocking(outtakeDump.downPosition());
        Actions.runBlocking(intakeWrist.wristDrive());
        climbingServo.hookPositionDown();
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

        FollowPathActions followStartToBucket1 = new FollowPathActions(StartToBucket1, follower, true, telemetryA);
        FollowPathActions followBucketToSample2 = new FollowPathActions(BucketToSample2, follower, true, telemetryA);
        FollowPathActions followSampleToBucket3 = new FollowPathActions(SampleToBucket3, follower, true, telemetryA);
        FollowPathActions followBucketToSample4 = new FollowPathActions(BucketToSample4, follower, true, telemetryA);
        FollowPathActions followSampleToBucket5 = new FollowPathActions(SampleToBucket5, follower, true, telemetryA);
        FollowPathActions followBucketToSample6 = new FollowPathActions(BucketToSample6, follower, true, telemetryA);
        FollowPathActions followSampleToBucket7 = new FollowPathActions(SampleToBucket7, follower, true, telemetryA);
        FollowPathActions followBucketToSub8 = new FollowPathActions(GotoSub8, follower, false, telemetryA);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                followStartToBucket1,
                                outtakeSlide.high(),
                                new SequentialAction(
                                        new SleepAction(1),
                                        outtakeDump.bucketPosition()
                                )
                        ),
                        new SleepAction(0.2),
                        new ParallelAction(
                                outtakeSlide.low(),
                                new SequentialAction(
                                        new SleepAction(0.1),
                                        outtakeDump.downPosition()
                                ),
                                followBucketToSample2,
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        new ParallelAction(
                                        intakeSlide.autonAction(),
                                        intakeWrist.wristIntakeAbyss(),
                                        intakeArm.armIntakeAbyss()
                                        )
                                )
                        ),
                        //new SleepAction(2.0),
                        intakeSpinner.intakePosition(),
                        intakeWrist.wristIntakePull(),
                        //new SleepAction(0.2),
                        intakeArm.armIntakePull(),
                        //StayAtSample2
                        new SleepAction(0.4),
                        intakeSlide.transfer(),
                        new ParallelAction(
                                followSampleToBucket3,
                                intakeSpinner.stopPosition(),
                                intakeWrist.wristTransfer(),
                                intakeArm.armTransfer(),
                                new SequentialAction(
                                        new SleepAction(.5),
                                        intakeSpinner.outtakePosition(),
                                        new SleepAction(.3),
                                        intakeSpinner.stopPosition(),
                                        new ParallelAction(
                                                outtakeSlide.high(),
                                                new SequentialAction(
                                                        new SleepAction(1.0),
                                                        outtakeDump.bucketPosition()
                                                )
                                        )
                                )
                        ),
                        new SleepAction(0.3),
                        new ParallelAction(
                                outtakeSlide.low(),
                                new SequentialAction(
                                        new SleepAction(0.1),
                                        outtakeDump.downPosition()
                                ),
                                followBucketToSample4,
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        new ParallelAction(
                                                intakeSpinner.intakePosition(),
                                        intakeWrist.wristIntakePush(),
                                        intakeArm.armIntakePush()
                                        )
                                )
                        ),
                        new SleepAction(0.1),
                        intakeSlide.transfer(),
                        new ParallelAction(
                            followSampleToBucket5,
                            intakeSpinner.stopPosition(),
                            intakeWrist.wristTransfer(),
                            intakeArm.armTransfer(),
                            new SequentialAction(
                                new SleepAction(.5),
                                intakeSpinner.outtakePosition(),
                                new SleepAction(.3),
                                intakeSpinner.stopPosition(),
                                new ParallelAction(
                                        outtakeSlide.high(),
                                        new SequentialAction(
                                                new SleepAction(1.0),
                                                outtakeDump.bucketPosition()
                                        )
                                )
                            )
                        ),
                        new SleepAction(0.5),
                        new ParallelAction(
                                outtakeSlide.low(),
                                new SequentialAction(
                                        new SleepAction(0.1),
                                        outtakeDump.downPosition()
                                ),
                        //😎👌👌👌 ༼ つ ◕_◕ ༽つ
                                followBucketToSample6,
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        new ParallelAction(
                                                intakeSpinner.intakePosition(),
                                                intakeWrist.wristIntakePush(),
                                                intakeArm.armIntakePush()
                                        )
                                )
                        ),
                        new SleepAction(0.1),
                        intakeSlide.transfer(),
                        new ParallelAction(
                                followSampleToBucket7,
                                intakeSpinner.stopPosition(),
                                new SequentialAction(
                                        new SleepAction(.2),
                                        new ParallelAction(
                                                intakeArm.armTransfer(),
                                                intakeWrist.wristTransfer()
                                        )
                                ),
                                new SequentialAction(
                                        new SleepAction(.7),
                                        intakeSpinner.outtakePosition(),
                                        new SleepAction(.3),
                                        intakeSpinner.stopPosition(),
                                        new ParallelAction(
                                                outtakeSlide.high(),
                                                new SequentialAction(
                                                        new SleepAction(1.0),
                                                        outtakeDump.bucketPosition()
                                                )
                                        )
                                )
                        ),
                        new SleepAction(0.5),
                        new ParallelAction(
                                outtakeSlide.low(),
                                new SequentialAction(
                                        new SleepAction(0.1),
                                        outtakeDump.downPosition()
                                ),
                                followBucketToSub8//😎👌👌👌 ༼ つ ◕_◕ ༽つ
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

