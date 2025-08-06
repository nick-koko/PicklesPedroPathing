package org.firstinspires.ftc.teamcode.PedroTeleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PedroAuton.globalRobotDataPedro;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * Simple Movement Teleop Script
 * Features:
 * - Field-centric driving with Pedro Pathing
 * - Speed control (normal/slow mode)
 * - Angle snapping to cardinal directions
 * - Position reset functionality
 */
@Config
@TeleOp(name = "Simple Movement Teleop", group = "Movement")
public class SimpleMovementTeleop extends OpMode {
    
    // Pedro Pathing follower
    private Follower follower;
    
    // Movement configuration
    public static double NORMAL_DRIVE_POWER = 1.0;
    public static double SLOW_DRIVE_POWER = 0.5;
    public static double ROTATION_SPEED = 0.8;
    
    // Angle snapping configuration
    public static double ANGLE_P_GAIN = 0.5;
    public static double MIN_ANGLE_POWER = 0.075;
    public static double MAX_ANGLE_POWER = 0.8;
    
    // Robot state variables
    private double currentDrivePower = NORMAL_DRIVE_POWER;
    private boolean angleSnapping = false;
    private double targetAngle = 0.0;
    
    // Starting position
    private Pose startPose = new Pose(9, 62.75, Math.toRadians(180));
    
    @Override
    public void init() {
        // Initialize Pedro Pathing follower
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        
        // Set starting pose (use auton pose if available)
        if (globalRobotDataPedro.hasAutonRun) {
            startPose = globalRobotDataPedro.autonPose;
            globalRobotDataPedro.hasAutonRun = false;
        }
        follower.setStartingPose(startPose);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Starting Pose", "X: %.2f, Y: %.2f, Heading: %.1f°", 
                         startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.update();
    }
    
    @Override
    public void start() {
        follower.startTeleopDrive();
        telemetry.addData("Status", "Started - Ready to Drive!");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // === MOVEMENT CONTROLS ===
        
        // Get joystick inputs
        double driving = -gamepad1.left_stick_y;   // Forward/Backward
        double strafe = -gamepad1.left_stick_x;    // Left/Right
        double rotate = -gamepad1.right_stick_x;   // Rotation
        
        // Speed control
        if (gamepad1.right_bumper) {
            currentDrivePower = SLOW_DRIVE_POWER;
        } else {
            currentDrivePower = NORMAL_DRIVE_POWER;
        }
        
        // Apply speed scaling to movement
        driving *= currentDrivePower;
        strafe *= currentDrivePower;
        rotate *= ROTATION_SPEED;
        
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
        
        // === UTILITY CONTROLS ===
        
        // Reset robot position
        if (gamepad1.back) {
            follower.setPose(new Pose(9, 62.75, Math.toRadians(180)));
            telemetry.addData("Action", "Position Reset!");
        }
        
        // Emergency stop
        if (gamepad1.start) {
            driving = 0;
            strafe = 0;
            rotate = 0;
            telemetry.addData("Action", "Emergency Stop!");
        }
        
        // === APPLY MOVEMENT ===
        
        // Update Pedro Pathing with movement vectors
        follower.setTeleOpMovementVectors(driving, strafe, rotate, false);
        follower.update();
        
        // === TELEMETRY ===
        
        Pose currentPose = follower.getPose();
        
        telemetry.addData("=== ROBOT POSITION ===", "");
        telemetry.addData("X Position", "%.2f", currentPose.getX());
        telemetry.addData("Y Position", "%.2f", currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
        
        telemetry.addData("=== MOVEMENT ===", "");
        telemetry.addData("Drive Power", currentDrivePower == SLOW_DRIVE_POWER ? "SLOW" : "NORMAL");
        telemetry.addData("Driving", "%.2f", driving);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Rotate", "%.2f", rotate);
        
        if (angleSnapping) {
            telemetry.addData("Angle Snap", "Target: %.1f°", targetAngle);
        }
        
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("Left Stick", "Drive/Strafe");
        telemetry.addData("Right Stick X", "Rotate");
        telemetry.addData("Right Bumper", "Slow Mode");
        telemetry.addData("D-Pad", "Angle Snap");
        telemetry.addData("Left Bumper", "45° Angle");
        telemetry.addData("Back", "Reset Position");
        telemetry.addData("Start", "Emergency Stop");
        
        telemetry.update();
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
    
    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
} 