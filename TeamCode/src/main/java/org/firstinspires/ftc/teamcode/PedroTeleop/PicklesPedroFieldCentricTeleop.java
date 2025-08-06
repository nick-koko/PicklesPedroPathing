package org.firstinspires.ftc.teamcode.PedroTeleop;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.mechanisms.IntakeWrist;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeArmMoverMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeElbowMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.OuttakeWrist;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is a modified example teleop that showcases movement and field-centric driving.
 * Mr.TODONE 😎👌👌 Mr.TODONE 😎👌👌 Mr.TODONE 😎👌👌 Mr.TODONE 😎👌👌
 */
@Config
@TeleOp(name = "PicklesPedroFieldCentricTeleop", group = "Aelep")
public class PicklesPedroFieldCentricTeleop extends OpMode {
    private Follower follower;
    IntakeServoSpinner frontIntake = new IntakeServoSpinner();
    ClawMechanism specimenClaw = new ClawMechanism();
    DualSlideMechanism outtakeSlide =  new DualSlideMechanism();  //Mr. Todone
    ClimbingHooks climbingServo = new ClimbingHooks();
    IntakeSlide intakeSlide =  new IntakeSlide();  //Mr. Todone
    IntakeArm intakeArmServo = new IntakeArm();
    OuttakeArmMoverMechanism outtakeArmServo = new OuttakeArmMoverMechanism();
    IntakeWrist intakeWrist = new IntakeWrist();
    //OuttakeElbowMechanism outtakeElbow = new OuttakeElbowMechanism();
    //OuttakeWrist outtakeWrist = new OuttakeWrist();

    double intakeSlidePower = 0.0;
    double intakeLeftStickSlidePower = 0.0;
    double intakeSlidePowerFactor;
    double intakeLeftStickSlidePowerFactor;
    public static double normDrivePower = 1;
    public static double slowedDrivePower = 0.5;
    double drivePower;

    boolean pushIntake = true;
    boolean goToTargetAngle;
    double targetAngleDeg = -135.0;
    double targetAngleRad;
    double propAngleGain = -0.5;
    double minAnglePower = 0.075;
    double maxRotate = 0.8;
    double angleAllianceOffset = 0.0;
    ElapsedTime intakeArmTime = new ElapsedTime();
    double stateDelayTime = -1.0;
    boolean retractingOuttakeSlide = false;
    boolean retractingIntakeSlide = false;
    private Pose startPose = new Pose(9,62.75,Math.toRadians(180));

    /** 
     * INIT FUNCTION - This runs ONCE when you press INIT on the driver station
     * Think of this like setting up all your equipment before a soccer game!
     **/
    @Override
    public void init() {
        
        // ===== STEP 1: Figure out where the robot is on the field =====
        // If we just finished autonomous, use the ending position from autonomous
        // If not, use our default starting position (9, 62.75 inches, facing backwards)
        if (globalRobotDataPedro.hasAutonRun){
            startPose = globalRobotDataPedro.autonPose;  // Use where autonomous ended
            globalRobotDataPedro.hasAutonRun = false;    // Reset the flag
        }
        // NOTE: This is super important for field-centric driving to work correctly!
        
        // ===== STEP 2: Set up Pedro Pathing (like giving the robot a GPS) =====
        // FConstants = "Follower" constants - tells Pedro HOW to drive (motor names, PID settings, etc.)
        // LConstants = "Localization" constants - tells Pedro HOW to track position (sensor locations, etc.) 
        follower = new Follower(hardwareMap, FConstants.class,LConstants.class);
        follower.setStartingPose(startPose);  // Tell Pedro where we are starting
        // This helps the robot know which direction is "forward" on the field
        
        // ===== STEP 3: Connect all robot mechanisms to actual hardware =====
        // Each .init() finds the motor/servo in the hardware map and gets it ready
        // Think of this like plugging controllers into a gaming console
        
        frontIntake.init(hardwareMap);        // The spinning thing that picks up samples
        specimenClaw.init(hardwareMap);       // The claw that grabs specimens 
        outtakeSlide.init(hardwareMap);       // The slides that lift up to score
        climbingServo.init(hardwareMap);      // The hooks for climbing at the end
        intakeSlide.init(hardwareMap);        // The slides that extend the intake out
        intakeArmServo.init(hardwareMap);     // The arm that moves the intake up/down
        outtakeArmServo.init(hardwareMap);    // The arm that tips the bucket to score
        intakeWrist.init(hardwareMap);        // The wrist that aims the intake
        
        // These are commented out because we decided not to use them this year:
        //outtakeElbow.init(hardwareMap);     // Extra outtake joint we didn't need
        //outtakeWrist.init(hardwareMap);     // Extra outtake wrist we didn't need

    }

    /** 
     * INIT_LOOP FUNCTION - This runs REPEATEDLY while waiting for the match to start
     * Think of this like the time between when the robot is ready and when the referee says "GO!"
     * This is perfect for emergency fixes or last-minute adjustments before the match starts
     **/
    @Override
    public void init_loop() {
        
        // ===== EMERGENCY SLIDE RESET (available during init) =====
        // If the driver pulls the right trigger on gamepad1, reset both slides to zero position
        // This is helpful if the slides got moved during setup and need to be recalibrated
        if (gamepad1.right_trigger > 0.2) {
            intakeSlide.resetSlide();     // Reset intake slide encoder to 0
            outtakeSlide.resetSlide();    // Reset outtake slide encoder to 0  
            gamepad1.rumble(1000);        // Buzz the controller to confirm it worked
        }
    }

    /** 
     * START FUNCTION - This runs ONCE when the match begins (when you press the PLAY button)
     * Think of this like getting into your starting stance before a basketball game!
     * It puts all the robot mechanisms in safe, ready-to-drive positions
     **/
    @Override
    public void start() {
        
        // ===== ACTIVATE PEDRO PATHING FOR TELEOP DRIVING =====
        follower.startTeleopDrive();  // Tell Pedro we're starting teleop (not autonomous)
        
        // Old Road Runner code - not needed anymore since we use Pedro Pathing:
        //follower.setPose(startPose); old RR teleop had robot.pose = startingPose; at the start of teleop, not sure if needed

        // ===== SET ALL MECHANISMS TO SAFE STARTING POSITIONS =====
        // Put everything in "drive" position - safe and out of the way for driving around
        
        outtakeArmServo.armMoverDrivePosition();   // Outtake arm down and safe
        intakeSlide.slidePositionTransfer();       // Intake slides pulled back inside robot
        intakeArmServo.armPositionDrive();         // Intake arm in safe driving position  
        intakeWrist.wristPositionDrive();          // Intake wrist pointing forward
        specimenClaw.clawOpen();                   // Claw open and ready to grab specimens
        climbingServo.hookPositionDown();          // Climbing hooks down and out of the way
        
        // These mechanisms were removed from this year's robot design:
        //outtakeElbow.outtakeElbowDrivePosition();  // Not used this year
        //outtakeWrist.wristPositionSideways();      // Not used this year

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

        if (gamepad1.right_bumper) {
            drivePower = slowedDrivePower;
        } else {
            drivePower = normDrivePower;
        }

        double driving = (-gamepad1.right_stick_y) * drivePower;
        double strafe = (-gamepad1.right_stick_x) * drivePower;
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


//mR. TODONE 😎👌👌
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

        intakeLeftStickSlidePowerFactor = 0.4;

        if (-gamepad2.left_stick_y < -0.05) {
            pushIntake = false;
            intakeLeftStickSlidePower = -(gamepad2.left_stick_y * intakeLeftStickSlidePowerFactor);
        } else {
            pushIntake = true;
            intakeLeftStickSlidePower = 0.0;
        }

        if ((-gamepad2.right_stick_y > 0) || (!pushIntake)) {
            intakeSlidePowerFactor = 0.400;
        } else {
            intakeSlidePowerFactor = 0.800;
        }

        intakeSlidePower = -(gamepad2.right_stick_y * intakeSlidePowerFactor);

        if (gamepad2.dpad_down) {
            outtakeSlide.retractSlideIgnoreEncoderPosition(-0.3);
            retractingOuttakeSlide = true;
        } else if (retractingOuttakeSlide) {
            outtakeSlide.stopSlide();
            retractingOuttakeSlide = false;
        }
        if (gamepad2.dpad_left) {
            intakeSlide.retractSlideIgnoreEncoderPosition(-0.3);
            retractingIntakeSlide = true;
        } else if (retractingIntakeSlide) {
            intakeSlide.stopSlide();
            retractingIntakeSlide = false;
        }

        if (gamepad2.right_stick_button) {
            intakeWrist.wristPositionAbyss();
            intakeArmServo.armPositionAbyss();
        }
        if (gamepad1.right_trigger > 0.2) {
            intakeSlide.resetSlide();
            outtakeSlide.resetSlide();
            gamepad1.rumble(1000);
        }

        if (intakeSlidePower > 0.05) {
            if ((intakeArmServo.getARMState() == IntakeArm.INTAKE_ARM_STATES.INTAKE_ARM_TRANSFER_POS) ||
                    (intakeArmServo.getARMState() == IntakeArm.INTAKE_ARM_STATES.INTAKE_ARM_DRIVE_POS)) {
                intakeArmTime.reset();
                stateDelayTime = -1.0;
            }

            if (!gamepad2.right_stick_button) {
                if (intakeArmTime.time() > stateDelayTime) {
                    if (intakeSlide.getSlideMotorPos() > 380) {
                        if (pushIntake == true) {
                            intakeWrist.wristPositionPushIntake();
                            intakeArmServo.armPositionFarIntake();
                        } else {
                            intakeWrist.wristPositionPullIntake();
                            intakeArmServo.armPositionPullIntake();
                        }
                    } else {
                        if (pushIntake == true) {
                            intakeWrist.wristPositionPushIntake();
                            intakeArmServo.armPositionIntake();
                        } else {
                            intakeWrist.wristPositionPullIntake();
                            intakeArmServo.armPositionPullIntake();
                        }
                    }
                } else {
                    intakeArmServo.armPositionAbyss();
                }
            }
            //intakeArmServo.armPositionIntake();
            frontIntake.Intake();
            if (pushIntake == true) {
                intakeSlide.extendSlide(intakeSlidePower);
            } else {
                intakeSlide.extendSlide(intakeSlidePower);
                //intakeSlide.extendSlide(intakeLeftStickSlidePower);
            }
        } else if (intakeSlidePower < -0.05) {
            if (!gamepad2.right_stick_button) {
                if (pushIntake) {
                    if (intakeSlide.getSlideState() != IntakeSlide.SLIDE_STATES.SLIDE_INTAKE_POS) {
                        intakeArmServo.armPositionTransfer();
                        intakeWrist.wristPositionTransfer();
                        //frontIntake.Stop();
                        //mouthIntake.eatPosition
                        intakeSlide.retractSlide(intakeSlidePower);
                    } else {
                        frontIntake.Stop();
                        intakeSlide.slidePositionTransfer();
                        intakeArmServo.armPositionAbyss();
                        intakeWrist.wristPositionDrive();
                        //intakeArmServo.armPositionTransfer(); switch to Transfer if faster is better?
                        //intakeWrist.wristPositionTransfer();

                        //frontIntake.Outtake();
                    }
                } else {
                    frontIntake.Intake();
                    intakeWrist.wristPositionPullIntake();
                    intakeArmServo.armPositionPullIntake();
                    intakeSlide.retractSlide(intakeLeftStickSlidePower);
                }
            } else {
                intakeSlide.retractSlide(intakeSlidePower);
            }
        } else if (!retractingIntakeSlide) {
            if (intakeSlide.getSlideState() == IntakeSlide.SLIDE_STATES.SLIDE_TRANSFER_POS)
            {
                intakeSlide.slidePositionTransfer();
            } else {
                if ((!pushIntake) && (!gamepad2.right_stick_button)) {
                    intakeWrist.wristPositionPullIntake();
                    intakeArmServo.armPositionPullIntake();
                    intakeSlide.retractSlide(intakeLeftStickSlidePower);
                } else {
                    intakeSlide.stopSlide();
                }
            }

            if (frontIntake.getIntakeState() != IntakeServoSpinner.INTAKE_SPINNER_STATES.SPINNER_INTAKING) {
                frontIntake.Stop();
            }
        }

        // Specimen Claw
        if (gamepad2.left_trigger > 0.05) {
            specimenClaw.clawOpen();
        } else if (gamepad2.right_trigger > 0.05) {
            specimenClaw.clawClose();
        }

        // SLIDE CONDITIONS

        if (gamepad2.y) {
            intakeArmServo.armPositionTransfer();
            intakeWrist.wristPositionDrive();
            outtakeSlide.slidePositionHigh();
        }
        else if (gamepad2.a) {
            if (outtakeSlide.getSlideState() == DualSlideMechanism.SLIDE_STATES.SLIDE_SPECIMENDROP_POS){
                specimenClaw.clawDropPosition();
                specimenClaw.clawOff();
            }
            intakeArmServo.armPositionTransfer();
            intakeWrist.wristPositionDrive();
            outtakeSlide.slidePositionLow();
        }
        else if (gamepad2.x) {
            intakeArmServo.armPositionTransfer();
            intakeWrist.wristPositionDrive();
            outtakeSlide.slidePositionMiddle();
        }
        else if (gamepad2.b) {
            intakeArmServo.armPositionTransfer();
            intakeWrist.wristPositionDrive();
            outtakeSlide.slidePositionSpecimenDrop();
        }
        else if (gamepad2.start) {
            intakeArmServo.armPositionTransfer();
            intakeWrist.wristPositionDrive();
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
            if (intakeWrist.getWRISTState() == IntakeWrist.INTAKE_WRIST_STATES.INTAKE_WRIST_TRANSFER_POS) {
                intakeWrist.wristPositionDrive();
                intakeArmServo.armPositionTransfer();
            }
            outtakeArmServo.armMoverBucketPosition();
        } else {
            outtakeArmServo.armMoverDrivePosition();
        }
        if (gamepad2.left_bumper) {
            frontIntake.Outtake();
        } else if (frontIntake.getIntakeState() != IntakeServoSpinner.INTAKE_SPINNER_STATES.SPINNER_INTAKING){
            frontIntake.Stop();
        }

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Target Heading in Degrees", targetAngleDeg);

        telemetry.addData("Intake Push State: ",  pushIntake);
        telemetry.addData("Intake Arm State: ", intakeArmServo.getARMState());
        telemetry.addData("Intake Slide State: ", intakeSlide.getSlideState());
        telemetry.addData("OuttakeArmServo State: ", outtakeArmServo.getArmMoverState());
        telemetry.addData("Outtake Slide State: ", outtakeSlide.getSlideState());
        telemetry.addData("Intake Slide Power:", intakeSlidePower);
        telemetry.addData("Intake Motor Position:", intakeSlide.getSlideMotorPos());
        telemetry.addData("Out Slide Motor R Position:", outtakeSlide.getSlideRMotorPos());
        telemetry.addData("Out Slide Motor L Position:", outtakeSlide.getSlideLMotorPos());
        telemetry.addData("Out Slide Motor R Current:", outtakeSlide.getSlideRMotorCurr());
        telemetry.addData("Out Slide Motor L Current:", outtakeSlide.getSlideLMotorCurr());
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