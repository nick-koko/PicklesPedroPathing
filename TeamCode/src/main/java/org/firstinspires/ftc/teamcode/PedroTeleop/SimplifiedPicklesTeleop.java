package org.firstinspires.ftc.teamcode.PedroTeleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PedroAuton.globalRobotDataPedro;
import org.firstinspires.ftc.teamcode.mechanisms.*;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "Simplified Pickles Teleop", group = "Main")
public class SimplifiedPicklesTeleop extends OpMode {
    
    // Pedro Pathing
    private Follower follower;
    
    // Robot Mechanisms
    private IntakeServoSpinner frontIntake = new IntakeServoSpinner();
    private ClawMechanism specimenClaw = new ClawMechanism();
    private DualSlideMechanism outtakeSlide = new DualSlideMechanism();
    private IntakeSlide intakeSlide = new IntakeSlide();
    private IntakeArm intakeArm = new IntakeArm();
    private IntakeWrist intakeWrist = new IntakeWrist();
    private OuttakeArmMoverMechanism outtakeArm = new OuttakeArmMoverMechanism();
    private ClimbingHooks climbingHooks = new ClimbingHooks();
    
    // Drive Settings
    public static double NORMAL_SPEED = 1.0;
    public static double SLOW_SPEED = 0.5;
    public static double ROTATION_SPEED = 0.8;
    
    // Angle Snapping Settings
    public static double ANGLE_P_GAIN = 0.5;
    public static double MIN_ANGLE_POWER = 0.075;
    public static double MAX_ANGLE_POWER = 0.8;
    
    // Angle snapping state
    private boolean angleSnapping = false;
    private double targetAngle = 0.0;
    
    // Intake state management
    private boolean pushIntake = true;
    private double intakeLeftStickPower = 0.0;
    private double intakeSlidePowerFactor = 0.8;
    
    // Emergency retraction state tracking
    private boolean retractingOuttakeSlide = false;
    private boolean retractingIntakeSlide = false;
    
    // Starting position
    private Pose startPose = new Pose(9, 62.75, Math.toRadians(180));
    
    @Override
    public void init() {
        // Initialize Pedro Pathing
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        
        // Set starting pose
        if (globalRobotDataPedro.hasAutonRun) {
            startPose = globalRobotDataPedro.autonPose;
            globalRobotDataPedro.hasAutonRun = false;
        }
        follower.setStartingPose(startPose);
        
        // Initialize all mechanisms
        frontIntake.init(hardwareMap);
        specimenClaw.init(hardwareMap);
        outtakeSlide.init(hardwareMap);
        intakeSlide.init(hardwareMap);
        intakeArm.init(hardwareMap);
        intakeWrist.init(hardwareMap);
        outtakeArm.init(hardwareMap);
        climbingHooks.init(hardwareMap);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    
    @Override
    public void start() {
        follower.startTeleopDrive();
        
        // Set starting positions
        outtakeArm.armMoverDrivePosition();
        intakeSlide.slidePositionTransfer();
        intakeArm.armPositionDrive();
        intakeWrist.wristPositionDrive();
        specimenClaw.clawOpen();
        climbingHooks.hookPositionDown();
    }
    
    @Override
    public void loop() {
        // === DRIVING ===
        handleDriving();
        
        // === INTAKE SYSTEM ===
        handleIntake();
        
        // === OUTTAKE SYSTEM ===
        handleOuttake();
        
        // === SPECIMEN CLAW ===
        handleSpecimenClaw();
        
        // === CLIMBING ===
        handleClimbing();
        
        // === UTILITY ===
        handleUtility();
        
        // Update follower and telemetry
        follower.update();
        updateTelemetry();
    }
    
    private void handleDriving() {
        // Get drive inputs (matching original joystick mapping)
        double drive = -gamepad1.right_stick_y;
        double strafe = -gamepad1.right_stick_x;
        double rotate = -gamepad1.left_stick_x * ROTATION_SPEED;
        
        // Speed control
        double speed = gamepad1.right_bumper ? SLOW_SPEED : NORMAL_SPEED;
        drive *= speed;
        strafe *= speed;
        
        // === ANGLE SNAPPING ===
        // Check for angle snapping inputs
        if (gamepad1.dpad_up) {
            targetAngle = 0.0;
            angleSnapping = true;
        } else if (gamepad1.dpad_right) {
            targetAngle = -90.0;
            angleSnapping = true;
        } else if (gamepad1.dpad_down) {
            targetAngle = 180.0;
            angleSnapping = true;
        } else if (gamepad1.dpad_left) {
            targetAngle = 90.0;
            angleSnapping = true;
        } else if (gamepad1.left_bumper) {
            targetAngle = -45.0;
            angleSnapping = true;
        } else {
            angleSnapping = false;
        }
        
        // Apply angle snapping if active
        if (angleSnapping) {
            rotate = calculateAngleCorrection();
        }
        
        // Apply movement
        follower.setTeleOpMovementVectors(drive, strafe, rotate, false);
    }
    
    private void handleIntake() {
        // === ABYSS OVERRIDE (Emergency/Obstacle Avoidance) ===
        if (gamepad2.right_stick_button) {
            intakeWrist.wristPositionAbyss();
            intakeArm.armPositionAbyss();
            // Don't process normal intake logic when in abyss mode
            return;
        }
        
        // Determine intake mode based on left stick
        if (-gamepad2.left_stick_y < -0.05) {
            pushIntake = false;  // Pull mode
            intakeLeftStickPower = -gamepad2.left_stick_y * 0.4;
        } else {
            pushIntake = true;   // Push mode
            intakeLeftStickPower = 0.0;
        }
        
        // Set slide power factor based on mode
        double slidePower = -gamepad2.right_stick_y;
        if (slidePower > 0 || !pushIntake) {
            intakeSlidePowerFactor = 0.4;  // Slower when extending or pulling
        } else {
            intakeSlidePowerFactor = 0.8;  // Faster when retracting in push mode
        }
        
        slidePower *= intakeSlidePowerFactor;
        
        // === EXTENDING INTAKE ===
        if (slidePower > 0.05) {
            handleIntakeExtending(slidePower);
        }
        // === RETRACTING INTAKE ===
        else if (slidePower < -0.05) {
            handleIntakeRetracting(slidePower);
        }
        // === STOPPED OR PULL MODE ===
        else if (!retractingIntakeSlide) {
            handleIntakeStopped();
        }
        
        // Manual intake spinner control
        if (gamepad2.left_bumper) {
            frontIntake.Outtake();
        } else if (frontIntake.getIntakeState() != IntakeServoSpinner.INTAKE_SPINNER_STATES.SPINNER_INTAKING) {
            frontIntake.Stop();
        }
    }
    
    private void handleIntakeExtending(double slidePower) {
        // Set appropriate arm/wrist positions based on extension and mode
        if (intakeSlide.getSlideMotorPos() > 380) {
            // Far extension - use far positions
            if (pushIntake) {
                intakeWrist.wristPositionPushIntake();
                intakeArm.armPositionFarIntake();
            } else {
                intakeWrist.wristPositionPullIntake();
                intakeArm.armPositionPullIntake();
            }
        } else {
            // Close extension - use normal positions
            if (pushIntake) {
                intakeWrist.wristPositionPushIntake();
                intakeArm.armPositionIntake();
            } else {
                intakeWrist.wristPositionPullIntake();
                intakeArm.armPositionPullIntake();
            }
        }
        
        frontIntake.Intake();
        intakeSlide.extendSlide(slidePower);
    }
    
    private void handleIntakeRetracting(double slidePower) {
        if (pushIntake) {
            // Push mode retraction - check if we need to transfer
            if (intakeSlide.getSlideState() != IntakeSlide.SLIDE_STATES.SLIDE_INTAKE_POS) {
                // Still retracting - go to transfer position
                intakeArm.armPositionTransfer();
                intakeWrist.wristPositionTransfer();
                intakeSlide.retractSlide(slidePower);
            } else {
                // Fully retracted - stop and go to drive position
                frontIntake.Stop();
                intakeSlide.slidePositionTransfer();
                intakeArm.armPositionDrive();
                intakeWrist.wristPositionDrive();
            }
        } else {
            // Pull mode retraction - maintain pull positions
            frontIntake.Intake();
            intakeWrist.wristPositionPullIntake();
            intakeArm.armPositionPullIntake();
            intakeSlide.retractSlide(intakeLeftStickPower);
        }
    }
    
    private void handleIntakeStopped() {
        if (intakeSlide.getSlideState() == IntakeSlide.SLIDE_STATES.SLIDE_TRANSFER_POS) {
            // Maintain transfer position
            intakeSlide.slidePositionTransfer();
        } else {
            // Handle pull mode when stopped
            if (!pushIntake) {
                intakeWrist.wristPositionPullIntake();
                intakeArm.armPositionPullIntake();
                intakeSlide.retractSlide(intakeLeftStickPower);
            } else {
                intakeSlide.stopSlide();
            }
        }
        
        // Stop intake spinner if not actively intaking
        if (frontIntake.getIntakeState() != IntakeServoSpinner.INTAKE_SPINNER_STATES.SPINNER_INTAKING) {
            frontIntake.Stop();
        }
    }
    
    private void handleOuttake() {
        // Outtake slide positions
        if (gamepad2.y) {
            outtakeSlide.slidePositionHigh();
            setTransferPosition();
        } else if (gamepad2.x) {
            outtakeSlide.slidePositionMiddle();
            setTransferPosition();
        } else if (gamepad2.a) {
            // Special claw logic for specimen drop -> low transition
            if (outtakeSlide.getSlideState() == DualSlideMechanism.SLIDE_STATES.SLIDE_SPECIMENDROP_POS) {
                specimenClaw.clawDropPosition();
                specimenClaw.clawOff();  // Disable servo to prevent back-drive
            }
            outtakeSlide.slidePositionLow();
            setTransferPosition();
        } else if (gamepad2.b) {
            outtakeSlide.slidePositionSpecimenDrop();
            setTransferPosition();
        } else {
            // Stop slide if at low position and not moving
            if ((outtakeSlide.getSlideState() == DualSlideMechanism.SLIDE_STATES.SLIDE_LOW_POS) &&
                (outtakeSlide.getNextSlideState() == DualSlideMechanism.SLIDE_STATES.SLIDE_LOW_POS)) {
                outtakeSlide.stopSlide();
            }
        }
        
        // Smart outtake arm control (coordinate with intake)
        if (gamepad2.right_bumper) {
            // Move intake out of the way before raising outtake arm
            if (intakeWrist.getWRISTState() == IntakeWrist.INTAKE_WRIST_STATES.INTAKE_WRIST_TRANSFER_POS) {
                intakeWrist.wristPositionDrive();
                intakeArm.armPositionTransfer();
            }
            outtakeArm.armMoverBucketPosition();
        } else {
            outtakeArm.armMoverDrivePosition();
        }
        
        // Emergency slide retract with state tracking
        if (gamepad2.dpad_down) {
            outtakeSlide.retractSlideIgnoreEncoderPosition(-0.3);
            retractingOuttakeSlide = true;
        } else if (retractingOuttakeSlide) {
            outtakeSlide.stopSlide();
            retractingOuttakeSlide = false;
        }
    }
    
    private void handleSpecimenClaw() {
        if (gamepad2.left_trigger > 0.05) {
            specimenClaw.clawOpen();
        } else if (gamepad2.right_trigger > 0.05) {
            specimenClaw.clawClose();
        }
    }
    
    private void handleClimbing() {
        if (gamepad2.start) {
            // Prepare for climb
            specimenClaw.clawClose();
            outtakeSlide.slidePositionClimb();
            climbingHooks.hookPositionHigh();
            setTransferPosition();
        } else if (gamepad2.back) {
            // End hang
            climbingHooks.hookPositionHigh();
            outtakeSlide.slidePositionEndHang();
        }
    }
    
    private void handleUtility() {
        // Reset position
        if (gamepad1.back) {
            follower.setPose(new Pose(9, 62.75, Math.toRadians(180)));
        }
        
        // Reset encoders
        if (gamepad1.right_trigger > 0.2) {
            intakeSlide.resetSlide();
            outtakeSlide.resetSlide();
            gamepad1.rumble(1000);
        }
        
        // Emergency intake slide retract with state tracking
        if (gamepad2.dpad_left) {
            intakeSlide.retractSlideIgnoreEncoderPosition(-0.3);
            retractingIntakeSlide = true;
        } else if (retractingIntakeSlide) {
            intakeSlide.stopSlide();
            retractingIntakeSlide = false;
        }
    }
    
    private void setTransferPosition() {
        intakeArm.armPositionTransfer();
        intakeWrist.wristPositionDrive();
    }
    
    /**
     * Calculate rotation correction for angle snapping
     */
    private double calculateAngleCorrection() {
        double currentHeading = follower.getPose().getHeading();
        double targetRadians = Math.toRadians(targetAngle);
        
        // Calculate the shortest angle difference
        double angleDiff = currentHeading - targetRadians;
        
        // Normalize angle difference to [-π, π]
        while (angleDiff > Math.PI) {
            angleDiff -= 2 * Math.PI;
        }
        while (angleDiff < -Math.PI) {
            angleDiff += 2 * Math.PI;
        }
        
        // Calculate proportional control
        double correction = -angleDiff * ANGLE_P_GAIN;
        
        // Add minimum power to overcome static friction
        if (correction > 0) {
            correction += MIN_ANGLE_POWER;
        } else if (correction < 0) {
            correction -= MIN_ANGLE_POWER;
        }
        
        // Clamp to maximum power
        correction = Math.max(Math.min(correction, MAX_ANGLE_POWER), -MAX_ANGLE_POWER);
        
        return correction;
    }
    
    private void updateTelemetry() {
        Pose pose = follower.getPose();
        
        telemetry.addData("=== ROBOT POSITION ===", "");
        telemetry.addData("X", "%.1f", pose.getX());
        telemetry.addData("Y", "%.1f", pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        
        telemetry.addData("=== MECHANISMS ===", "");
        telemetry.addData("Intake Mode", pushIntake ? "PUSH" : "PULL");
        telemetry.addData("Intake Slide", intakeSlide.getSlideState());
        telemetry.addData("Intake Position", intakeSlide.getSlideMotorPos());
        telemetry.addData("Outtake Slide", outtakeSlide.getSlideState());
        telemetry.addData("Claw", specimenClaw.getClawState());
        
        if (angleSnapping) {
            telemetry.addData("Angle Snap", "Target: %.1f°", targetAngle);
        }
        
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("Gamepad1", "Right Stick: Drive/Strafe, Left Stick X: Rotate");
        telemetry.addData("Gamepad1", "Speed, Reset, Angle Snap");
        telemetry.addData("Gamepad2", "Intake, Outtake, Claw");
        telemetry.addData("Right Stick Button", "Abyss Override");
        
        if (retractingOuttakeSlide || retractingIntakeSlide) {
            telemetry.addData("EMERGENCY", "Slide Retracting!");
        }
        
        telemetry.update();
    }
    
    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
} 