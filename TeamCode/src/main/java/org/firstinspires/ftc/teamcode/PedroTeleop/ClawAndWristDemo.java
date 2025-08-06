package org.firstinspires.ftc.teamcode.PedroTeleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.ClawMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.ClawOnArmMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeElbowMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeWrist;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is a modified example teleop that showcases movement and field-centric driving.
 *
 */
@Config
@TeleOp(name = "ClawWristDemo", group = "Aelep")
public class ClawAndWristDemo extends OpMode {
    ClawOnArmMechanism specimenClaw = new ClawOnArmMechanism();
    OuttakeWrist outtakeWrist = new OuttakeWrist();
    OuttakeElbowMechanism outtakeElbow = new OuttakeElbowMechanism();
    Limelight3A limelight;
    private Servo lightColorRGB;

    public static double lightColor = 0.5;
    ElapsedTime intakeArmTime = new ElapsedTime();

    double stateDelayTime = -1.0;

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {

        //Mechanisim Initialization

        specimenClaw.init(hardwareMap);
        outtakeWrist.init(hardwareMap);
        outtakeElbow.init(hardwareMap);
        lightColorRGB = hardwareMap.get(Servo.class, "rgb_color_PWM");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
   }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        outtakeWrist.wristPositionSideways();
        specimenClaw.clawOpen();
        outtakeElbow.outtakeElbowDrivePosition();

    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        // Specimen Claw
        if (gamepad2.a) {
            specimenClaw.clawOpen();
        } else {
            specimenClaw.clawClose();
        }

        // Wrist control with bumpers (incremental movement)
        if (gamepad2.left_bumper) {
            outtakeWrist.moveTowardsUp();
        } else if (gamepad2.right_bumper) {
            outtakeWrist.moveTowardsDown();
        } // Direct wrist position control with buttons
          else if (gamepad2.x) {
            outtakeElbow.outtakeElbowTransferPosition();
        } else if (gamepad2.y) {
            outtakeElbow.outtakeElbowBucketPosition();
        } else if (gamepad2.b) {
              outtakeElbow.outtakeElbowSpecimineGrabPosition();
        } else {
              outtakeWrist.syncServoToLastCommandedPosition();
        }

          lightColorRGB.setPosition(lightColor);

        /* Telemetry Outputs of our Follower */
        telemetry.addData("OuttakeWristServo State: ", outtakeWrist.getWristState());
        telemetry.addData("Wrist Position: ", outtakeWrist.getCurrentPosition());
        telemetry.addData("Left Trigger: ", gamepad2.left_trigger);
        telemetry.addData("Right Trigger: ", gamepad2.right_trigger);
        telemetry.addData("Claw State:    ",  specimenClaw.getClawState());

        /* Update Telemetry to the Driver Hub */
        telemetry.update();
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }


}