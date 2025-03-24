package org.firstinspires.ftc.teamcode.PedroAuton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
@Autonomous(name = "TestNumberBiggieCheeezeRennyPooPoo001", group = "Autonomous")
public class TestNumberBiggieCheeezeRennyPooPoo001 extends OpMode {
    private Follower follower;

    private final Pose startPose = new Pose(8.5,63.4, Math.toRadians(0));

    private Timer pathTimer;
    private PathChain triangle;

    private PathChain Plebeian1;

    private PathChain Goober2;

    private PathChain OhDip3;

    private PathChain Plebithan4;

    private PathChain cheesemonger5;

    private PathChain Cooker6;

    private PathChain HughJass7;

    private PathChain JustSayIt8;
    private Telemetry telemetryA;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();


        follower.telemetryDebug(telemetryA);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void start() {

            follower.followPath(Plebeian1, true);


    }

    /**
     * This initializes the Follower and creates the PathChain for the "triangle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class,LConstants.class);
        follower.setStartingPose(startPose);












        triangle = follower.pathBuilder()


                .addPath(
                        new BezierLine(
                                new Point(8.549, 63.820, Point.CARTESIAN),
                                new Point(29.331, 63.378, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(29.331, 63.378, Point.CARTESIAN),
                                new Point(42.890, 41.417, Point.CARTESIAN),
                                new Point(28.594, 37.290, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Point(28.594, 37.290, Point.CARTESIAN),
                                new Point(4.716, 35.374, Point.CARTESIAN),
                                new Point(32.131, 18.129, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierCurve(
                                new Point(32.131, 18.129, Point.CARTESIAN),
                                new Point(56.303, 9.875, Point.CARTESIAN),
                                new Point(107.742, 19.750, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        Plebeian1 = follower.pathBuilder()


                .addPath(
                        new BezierLine(
                                new Point(8.5, 63.4, Point.CARTESIAN),
                                new Point(38, 62.641, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Goober2 = follower.pathBuilder()

                .addPath(
                        new BezierCurve(
                                new Point(41.417, 62.641, Point.CARTESIAN),
                                new Point(7.369, 33.015, Point.CARTESIAN),
                                new Point(53.208, 36.553, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Point(53.208, 36.553, Point.CARTESIAN),
                                new Point(73.990, 27.415, Point.CARTESIAN),
                                new Point(50.702, 22.845, Point.CARTESIAN),
                                new Point(17.392, 22.993, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
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

       OhDip3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(53.208, 36.553, Point.CARTESIAN),
                                new Point(73.990, 27.415, Point.CARTESIAN),
                                new Point(50.702, 22.845, Point.CARTESIAN),
                                new Point(17.392, 22.993, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Plebithan4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(17.392, 22.993, Point.CARTESIAN),
                                new Point(58.809, 29.331, Point.CARTESIAN),
                                new Point(57.629, 22.256, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        cheesemonger5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(57.629, 22.256, Point.CARTESIAN),
                                new Point(61.314, 10.170, Point.CARTESIAN),
                                new Point(14.444, 13.855, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Cooker6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(14.444, 13.855, Point.CARTESIAN),
                                new Point(69.863, 17.097, Point.CARTESIAN),
                                new Point(57.629, 7.222, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        HughJass7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(57.629, 7.222, Point.CARTESIAN),
                                new Point(9.138, 7.517, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


       // follower.followPath(Plebeian1);
        //follower.followPath(Goober2);



        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run in a roughly triangular shape,"
                + "starting on the bottom-middle point. So, make sure you have enough "
                + "space to the left, front, and right to run the OpMode.");
        telemetryA.update();
    }

}
