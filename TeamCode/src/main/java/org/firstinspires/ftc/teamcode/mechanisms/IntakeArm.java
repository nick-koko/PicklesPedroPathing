package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class IntakeArm {
    // Assuming some motor control library is used, e.g., FTC SDK, but this can be customized
    private Servo intakeArmServo;

    // Target positions for the servo arm
    public static double INTAKE_POSITION = 1; //above group... will kill power .245
    public static double PULL_INTAKE_POSITION = 0.89;
    static double INTAKE_TELEOP_POSITION = 0.211;
   public static double DRIVE_POSITION = 0; //perpindicuar .8
   public static double TRANSFER_POSITION = 0.15; //servo towards slides .88
   public static double ABYSS_POSITION = 0.7; //servo towards slides .4
    public static double INTAKE_FAR_POSITION = 1; //mR TODONE 😎👌👌👌  ༼ つ ◕_◕ ༽つ ༼ ◕_◕ ༽

    private ElapsedTime armTimer = new ElapsedTime();
    public enum INTAKE_ARM_STATES{
        INTAKE_ARM_INTAKE_POS, INTAKE_ARM_DRIVE_POS, INTAKE_ARM_TRANSFER_POS, INTAKE_ARM_ABYSS_POS, INTAKE_ARM_INTAKE_FAR_POS
    }

    private INTAKE_ARM_STATES curARMState = null; //༼ つ ◕_◕ ༽つ ༼ つ ◕_◕ ༽つ ༼ つ ◕_◕ ༽つ
    private INTAKE_ARM_STATES nextARMState = null;

    double stateDelayTime = 0;

    public void init(HardwareMap hwMap) {

        intakeArmServo = hwMap.get(Servo.class, "intake_arm_servo");
        this.intakeArmServo.setDirection(Servo.Direction.FORWARD);
        curARMState = INTAKE_ARM_STATES.INTAKE_ARM_DRIVE_POS;
        nextARMState = INTAKE_ARM_STATES.INTAKE_ARM_DRIVE_POS;
    }

    // Method to move the arm to the intake position
    public void armPositionIntake() {
            intakeArmServo.setPosition(INTAKE_POSITION);
            curARMState = INTAKE_ARM_STATES.INTAKE_ARM_INTAKE_POS;
            nextARMState = INTAKE_ARM_STATES.INTAKE_ARM_INTAKE_POS;
    }
    // Method to move the arm to the intake position
    public void armPositionPullIntake() {
        intakeArmServo.setPosition(PULL_INTAKE_POSITION);
        curARMState = INTAKE_ARM_STATES.INTAKE_ARM_INTAKE_POS;
        nextARMState = INTAKE_ARM_STATES.INTAKE_ARM_INTAKE_POS;
    }
    public void armPositionFarIntake() {
        intakeArmServo.setPosition(INTAKE_FAR_POSITION);
        curARMState = INTAKE_ARM_STATES.INTAKE_ARM_INTAKE_FAR_POS;
        nextARMState = INTAKE_ARM_STATES.INTAKE_ARM_INTAKE_FAR_POS;
    }
    public void armPositionAbyss() {
        intakeArmServo.setPosition(ABYSS_POSITION);
        curARMState = INTAKE_ARM_STATES.INTAKE_ARM_ABYSS_POS;
        nextARMState = INTAKE_ARM_STATES.INTAKE_ARM_ABYSS_POS;
    }

    // Method to move the arm to the intake position
    public void armPositionDrive() {
        intakeArmServo.setPosition(DRIVE_POSITION);
        curARMState = INTAKE_ARM_STATES.INTAKE_ARM_DRIVE_POS;
        nextARMState = INTAKE_ARM_STATES.INTAKE_ARM_DRIVE_POS;
    }

    // Method to move the arm to the intake position
    public void armPositionTransfer() {
        intakeArmServo.setPosition(TRANSFER_POSITION);
        curARMState = INTAKE_ARM_STATES.INTAKE_ARM_TRANSFER_POS;
        nextARMState = INTAKE_ARM_STATES.INTAKE_ARM_TRANSFER_POS;
    }

    public INTAKE_ARM_STATES getARMState() {
        return curARMState;
    }
}