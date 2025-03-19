package org.firstinspires.ftc.teamcode.PedroTeleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PedroAuton.globalRobotDataPedro;
import org.firstinspires.ftc.teamcode.mechanisms.ClawMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.ClimbingHooks;
import org.firstinspires.ftc.teamcode.mechanisms.DualSlideMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeArm;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeServoSpinner;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeSlide;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeDumpMechanism;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is a modified example teleop that showcases movement and field-centric driving.
 *
 */

@TeleOp(name = "PicklesPedroFieldCentricTeleop", group = "Teleop")
public class PicklesPedroFieldCentricTeleop extends OpMode {
    private Follower follower;
    IntakeServoSpinner frontIntake = new IntakeServoSpinner();
    ClawMechanism specimenClaw = new ClawMechanism();
    DualSlideMechanism outtakeSlide =  new DualSlideMechanism();  //Mr. Todone
    ClimbingHooks climbingServo = new ClimbingHooks();
    IntakeSlide intakeSlide =  new IntakeSlide();  //Mr. Todone
    IntakeArm intakeArmServo = new IntakeArm();
    OuttakeDumpMechanism dumper = new OuttakeDumpMechanism();

    double intakeSlidePower = 0.0;
    double intakeSlidePowerFactor;
    boolean fieldCentric = true;
    boolean goToTargetAngle;
    double targetAngleDeg = -135.0;
    double targetAngleRad;
    double propAngleGain = -0.5;
    double minAnglePower = 0.075;
    double maxRotate = 0.8;
    double angleAllianceOffset = 0.0;
    ElapsedTime intakeArmTime = new ElapsedTime();
    double stateDelayTime = 0.0;

    private Pose startPose = new Pose(9,62.75,Math.toRadians(180));

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        if (globalRobotDataPedro.hasAutonRun){
            startPose = globalRobotDataPedro.autonPose;
            globalRobotDataPedro.hasAutonRun = false;
        }
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        //Mechanisim Initialization

        frontIntake.init(hardwareMap);
        specimenClaw.init(hardwareMap);
        outtakeSlide.init(hardwareMap);
        climbingServo.init(hardwareMap);
        intakeSlide.init(hardwareMap);
        intakeArmServo.init(hardwareMap);
        dumper.init(hardwareMap);

    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();

        //follower.setPose(startPose); old RR teleop had robot.pose = startingPose; at the start of teleop, not sure if needed

        dumper.DumperPositionDown();
        intakeSlide.slidePositionTransfer();
        intakeArmServo.armPositionDrive();
        specimenClaw.clawOpen();
        climbingServo.hookPositionDown();

    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: gamepad1.left_stick_x
        - Turn Left/Right Movement: gamepad1.right_stick_x
        - Robot-Centric Mode: false
        */

        double driving = -gamepad1.right_stick_y;
        double strafe = -gamepad1.right_stick_x;
        double rotate = (-gamepad1.left_stick_x) * 0.5;

        double botHeadingRad = follower.getPose().getHeading();

        if (gamepad1.back) {
            follower.setPose(new Pose(9, 62.75, Math.toRadians(180)));
        }
        if (gamepad1.left_bumper) {
            targetAngleDeg = -45.0 + angleAllianceOffset;
            goToTargetAngle = true;
        } else if (gamepad1.dpad_down) {
            targetAngleDeg = 180.0 + angleAllianceOffset;
            goToTargetAngle = true;
        } else if (gamepad1.dpad_right) {
            targetAngleDeg = -90.0 + angleAllianceOffset;
            goToTargetAngle = true;
        } else if (gamepad1.dpad_left) {
            targetAngleDeg = 90.0 + angleAllianceOffset;
            goToTargetAngle = true;
        } else if (gamepad1.dpad_up) {
            targetAngleDeg = 0.0 + angleAllianceOffset;
            goToTargetAngle = true;
        } else {
            goToTargetAngle = false;
        }

        targetAngleRad = Math.toRadians(targetAngleDeg);

        if (goToTargetAngle) {
            double targetAngleDiff = botHeadingRad - targetAngleRad;
            if (targetAngleDiff > Math.PI) {
                targetAngleDiff = (targetAngleDiff - 2 * (Math.PI));
            } else if (targetAngleDiff < -Math.PI) {
                targetAngleDiff = (2 * (Math.PI) + targetAngleDiff);
            }
            rotate = targetAngleDiff * propAngleGain;
            if (rotate > 0.0) {
                rotate = rotate + minAnglePower;
            } else if (rotate < 0.0) {
                rotate = rotate - minAnglePower;
            }
            rotate = Math.max(Math.min(rotate, maxRotate), -maxRotate);
        }

        follower.setTeleOpMovementVectors(driving, strafe, rotate, false);
        follower.update();

        // INTAKE CONDITIONS
        if(-gamepad2.right_stick_y > 0){
            intakeSlidePowerFactor = 0.20;
        }
        else{
            intakeSlidePowerFactor = 0.4;
        }
        intakeSlidePower = -(gamepad2.right_stick_y * intakeSlidePowerFactor);

        if (intakeSlidePower > 0.05) {
            if ((intakeArmServo.getARMState() == IntakeArm.INTAKE_ARM_STATES.INTAKE_ARM_TRANSFER_POS) ||
                    (intakeArmServo.getARMState() == IntakeArm.INTAKE_ARM_STATES.INTAKE_ARM_DRIVE_POS)) {
                intakeArmTime.reset();
                stateDelayTime = 0.5;
            }
            if (!gamepad2.right_stick_button) {
                if (intakeArmTime.time() > stateDelayTime) {
                    if (intakeSlide.getSlideMotorPos() > 380) {
                        intakeArmServo.armPositionFarIntake();
                    } else {
                        intakeArmServo.armPositionIntake();
                    }
                } else {
                    intakeArmServo.armPositionAbyss();
                }
            }
            //intakeArmServo.armPositionIntake();
            frontIntake.Intake();
            intakeSlide.extendSlide(intakeSlidePower);
        } else if (intakeSlidePower < -0.05) {
            if (intakeSlide.getSlideState() != IntakeSlide.SLIDE_STATES.SLIDE_INTAKE_POS){
                intakeArmServo.armPositionTransfer();
                //frontIntake.Stop();
                intakeSlide.retractSlide(intakeSlidePower);
            } else {
                frontIntake.Stop();
                intakeSlide.slidePositionTransfer();
                intakeArmServo.armPositionAbyss();
                //frontIntake.Outtake();
            }
        } else {
            if (intakeSlide.getSlideState() == IntakeSlide.SLIDE_STATES.SLIDE_TRANSFER_POS)
            {
                intakeSlide.slidePositionTransfer();
            } else {
                intakeSlide.stopSlide();
            }

            if (frontIntake.getIntakeState() != IntakeServoSpinner.INTAKE_SPINNER_STATES.SPINNER_INTAKING) {
                frontIntake.Stop();
            }
        }

        if (gamepad2.dpad_left) {
            frontIntake.Intake();
            //       } else if (gamepad2.dpad_down) {
            //           frontIntake.Stop();
        } else if (gamepad2.dpad_right) {
            frontIntake.Outtake();
        }

        // Specimen Claw
        if (gamepad2.left_trigger > 0.05) {
            specimenClaw.clawClose();
        } else if (gamepad2.right_trigger > 0.05) {
            specimenClaw.clawOpen();

        }


        // SLIDE CONDITIONS

        if (gamepad2.y) {
            intakeArmServo.armPositionDrive();
            outtakeSlide.slidePositionHigh();
        }
        else if (gamepad2.a) {
            if (outtakeSlide.getSlideState() == DualSlideMechanism.SLIDE_STATES.SLIDE_SPECIMENDROP_POS){
                specimenClaw.clawDropPosition();
            }
            intakeArmServo.armPositionDrive();
            outtakeSlide.slidePositionLow();
        }
        else if (gamepad2.x) {
            intakeArmServo.armPositionDrive();
            outtakeSlide.slidePositionMiddle();
        }
        else if (gamepad2.b) {
            intakeArmServo.armPositionDrive();
            outtakeSlide.slidePositionSpecimenDrop();
        }
        else if (gamepad2.start) {
            intakeArmServo.armPositionDrive();
            specimenClaw.clawClose();
            outtakeSlide.slidePositionClimb();
            climbingServo.hookPositionHigh();
        } else if (gamepad2.back) {
            climbingServo.hookPositionHigh();
            outtakeSlide.slidePositionEndHang();
        }
        else {
            if ((outtakeSlide.getSlideState() == DualSlideMechanism.SLIDE_STATES.SLIDE_LOW_POS) &&
                    (outtakeSlide.getNextSlideState() == DualSlideMechanism.SLIDE_STATES.SLIDE_LOW_POS)) {
                outtakeSlide.stopSlide();
            }
        }
        // ARM CONDITIONS


        if (gamepad2.right_bumper) {
            if (intakeArmServo.getARMState() == IntakeArm.INTAKE_ARM_STATES.INTAKE_ARM_TRANSFER_POS) {
                intakeArmServo.armPositionDrive();
            }
            dumper.DumperPositionDump();
        } else {
            dumper.DumperPositionDown();
        }
        if (gamepad2.left_bumper) {
            frontIntake.Outtake();
        } else if (frontIntake.getIntakeState() != IntakeServoSpinner.INTAKE_SPINNER_STATES.SPINNER_INTAKING){
            frontIntake.Stop();
        }

        telemetry.addData("Intake Arm State: ", intakeArmServo.getARMState());
        telemetry.addData("Intake Slide State: ", intakeSlide.getSlideState());
        telemetry.addData("Dumper State: ", dumper.getDumperState());
        telemetry.addData("Outtake Slide State: ", outtakeSlide.getSlideState());
        telemetry.addData("Intake Slide Power:", intakeSlidePower);
        telemetry.addData("Slide Motor Position:", intakeSlide.getSlideMotorPos());
        telemetry.addData("Slide Motor R Position:", outtakeSlide.getSlideRMotorPos());
        telemetry.addData("Slide Motor L Position:", outtakeSlide.getSlideLMotorPos());
        telemetry.addData("Left Trigger: ", gamepad2.left_trigger);
        telemetry.addData("Right Trigger: ", gamepad2.right_trigger);
        telemetry.addData("Claw State:    ",  specimenClaw.getClawState());

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }


}