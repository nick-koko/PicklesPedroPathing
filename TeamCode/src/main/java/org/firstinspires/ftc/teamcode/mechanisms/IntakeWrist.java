package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class IntakeWrist {
    // Assuming some motor control library is used, e.g., FTC SDK, but this can be customized
    private Servo intakeWristServo;

    // Target positions for the servo wrist
    public static double PUSH_INTAKE_POSITION = 0.385; //above group... will kill power .245
    public static double PULL_INTAKE_POSITION = 0.1;
    static double INTAKE_TELEOP_POSITION = 0.211;
   public static double DRIVE_POSITION = 0.77; //perpindicuar .8
    public static double TRANSFER_POSITION = 1.0; //servo towards slides .88
    public static double ABYSS_POSITION = 0.37; //servo towards slides .4
    public static double INTAKE_FAR_POSITION = 0.385; //above group... will kill power .245   ༼ つ ◕_◕ ༽つ

    private ElapsedTime wristTimer = new ElapsedTime();
    public enum INTAKE_WRIST_STATES{
        INTAKE_WRIST_INTAKE_PUSH_POS, INTAKE_WRIST_INTAKE_PULL_POS, INTAKE_WRIST_DRIVE_POS, INTAKE_WRIST_TRANSFER_POS, INTAKE_WRIST_ABYSS_POS, INTAKE_WRIST_INTAKE_FAR_POS
    }

    private INTAKE_WRIST_STATES curWRISTState = null;
    private INTAKE_WRIST_STATES nextWRISTState = null;

    double stateDelayTime = 0;

    public void init(HardwareMap hwMap) {

        intakeWristServo = hwMap.get(Servo.class, "intake_wrist_servo");
        this.intakeWristServo.setDirection(Servo.Direction.FORWARD);
        curWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_DRIVE_POS;
        nextWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_DRIVE_POS;
    }

    // Method to move the wrist to the intake position
    public void wristPositionPushIntake() {
            intakeWristServo.setPosition(PUSH_INTAKE_POSITION);
            curWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_INTAKE_PUSH_POS;
            nextWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_INTAKE_PUSH_POS;
    }

    public void wristPositionPullIntake() {
        intakeWristServo.setPosition(PULL_INTAKE_POSITION);
        curWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_INTAKE_PULL_POS;
        nextWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_INTAKE_PULL_POS;
    }
    // Method to move the wrist to the intake position
    public void wristPositionFarIntake() {
        intakeWristServo.setPosition(INTAKE_FAR_POSITION);
        curWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_INTAKE_FAR_POS;
        nextWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_INTAKE_FAR_POS;
    }
    public void wristPositionAbyss() {
        intakeWristServo.setPosition(ABYSS_POSITION);
        curWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_ABYSS_POS;
        nextWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_ABYSS_POS;
    }

    // Method to move the wrist to the intake position
    public void wristPositionDrive() {
        intakeWristServo.setPosition(DRIVE_POSITION);
        curWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_DRIVE_POS;
        nextWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_DRIVE_POS;
    }

    // Method to move the wrist to the intake position
    public void wristPositionTransfer() {
        intakeWristServo.setPosition(TRANSFER_POSITION);
        curWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_TRANSFER_POS;
        nextWRISTState = INTAKE_WRIST_STATES.INTAKE_WRIST_TRANSFER_POS;
    }

    public INTAKE_WRIST_STATES getWRISTState() {
        return curWRISTState;
    }
}