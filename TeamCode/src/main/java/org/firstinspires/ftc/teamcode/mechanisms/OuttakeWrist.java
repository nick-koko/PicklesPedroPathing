package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class OuttakeWrist {
    // Assuming some motor control library is used, e.g., FTC SDK, but this can be customized
    private Servo outtakeWristServo;

    // Target positions for the servo wrist
    public static double SIDEWAYS_POSITION = 1;
    public static double UP_POSITION = 1;
    public static double DOWN_POSITION = 1;

    private ElapsedTime wristTimer = new ElapsedTime();
    public enum OUTTAKE_WRIST_STATES{
        OUTTAKE_WRIST_SIDEWAYS_POS, OUTTAKE_WRIST_UP_POSITION, OUTTAKE_WRIST_DOWN_POSITION, OUTTAKE_WRIST_DRIVE_POS
    }

    private OUTTAKE_WRIST_STATES curWRISTState = null;
    private OUTTAKE_WRIST_STATES nextWRISTState = null;

    double stateDelayTime = 0;

    public void init(HardwareMap hwMap) {

        outtakeWristServo = hwMap.get(Servo.class, "outtake_wrist_servo"); //༼ * ◕_◕ ༽*
        this.outtakeWristServo.setDirection(Servo.Direction.FORWARD);
        curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_DRIVE_POS;
        nextWRISTState =OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_DRIVE_POS;
    }

    // Method to move the wrist to the outtake position
    public void wristPositionSideways() {
        outtakeWristServo.setPosition(SIDEWAYS_POSITION);
        curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_SIDEWAYS_POS;
        nextWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_SIDEWAYS_POS;
    }
    public void wristPositionUp() {
        outtakeWristServo.setPosition(UP_POSITION);
        curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_UP_POSITION;
        nextWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_UP_POSITION;
    }
    public void wristPositionDown() {
        outtakeWristServo.setPosition(DOWN_POSITION);
        curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_DOWN_POSITION;
        nextWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_DOWN_POSITION;
    }
    public OUTTAKE_WRIST_STATES getWristState() {
        return curWRISTState;
    }

}