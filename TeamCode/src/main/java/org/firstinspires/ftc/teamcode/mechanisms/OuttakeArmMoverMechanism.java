package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class OuttakeArmMoverMechanism {
    // Assuming some motor control library is used, e.g., FTC SDK, but this can be customized
    private Servo armMoverServo;

    // Target positions for the servo arm
    public static double DRIVE_POSITION = 1;
    public static double TRANSFER_POSITION = 1;
    public static double BUCKET_POSITION = 1;
    public static double SPECIMINE_GRAB_POSITION = 1;
    public static double SPECIMINE_HANG_POSITION = 1;
    public enum ARMMOVER_STATES{
        ARMMOVER_DRIVE_POS, ARMMOVER_TRANSFER_POS, ARMMOVER_BUCKET_POS, ARMMOVER_SPECIMINE_GRAB_POS, ARMMOVER_SPECIMINE_HANG_POS
    }

    private ElapsedTime armMoverTimer = new ElapsedTime();

    private ARMMOVER_STATES curARMMOVERState = null;
    private ARMMOVER_STATES nextARMMOVERState = null;

    double stateDelayTime = 0;

    public void init(HardwareMap hwMap) {

        armMoverServo = hwMap.get(Servo.class, "arm_mover_servo");
        this.armMoverServo.setDirection(Servo.Direction.FORWARD);
        curARMMOVERState = ARMMOVER_STATES.ARMMOVER_DRIVE_POS;
        nextARMMOVERState = ARMMOVER_STATES.ARMMOVER_DRIVE_POS;
    }

    // Method to move the armMover to the down position
    public void armMoverDrivePosition() {

        armMoverServo.setPosition(DRIVE_POSITION);

        if (curARMMOVERState != ARMMOVER_STATES.ARMMOVER_DRIVE_POS) {
            armMoverTimer.reset();
            stateDelayTime = 0.5;
            nextARMMOVERState = ARMMOVER_STATES.ARMMOVER_DRIVE_POS;
        }
    }

    // Method to move the armMover to the dump position

    public void armMoverBucketPosition() {

        armMoverServo.setPosition(BUCKET_POSITION);
        if (curARMMOVERState != ARMMOVER_STATES.ARMMOVER_BUCKET_POS) {
            armMoverTimer.reset();
            stateDelayTime = 0.5;
            nextARMMOVERState = ARMMOVER_STATES.ARMMOVER_BUCKET_POS;
        }
    }
    public void armMoverTransferPosition() {

        armMoverServo.setPosition(TRANSFER_POSITION);
        if (curARMMOVERState != ARMMOVER_STATES.ARMMOVER_TRANSFER_POS) {
            armMoverTimer.reset();
            stateDelayTime = 0.5;
            nextARMMOVERState = ARMMOVER_STATES.ARMMOVER_TRANSFER_POS;
        }
    }
    public void armMoverSpecimineGrabPosition() {
        armMoverServo.setPosition(SPECIMINE_GRAB_POSITION);
        if (curARMMOVERState != ARMMOVER_STATES.ARMMOVER_SPECIMINE_GRAB_POS) {
            armMoverTimer.reset();
            stateDelayTime = 0.5;
            nextARMMOVERState = ARMMOVER_STATES.ARMMOVER_SPECIMINE_GRAB_POS;
        }
    }
    public void armMoverSpecimineHangPosition() {
        armMoverServo.setPosition(SPECIMINE_HANG_POSITION);
        if (curARMMOVERState != ARMMOVER_STATES.ARMMOVER_SPECIMINE_HANG_POS) {
            armMoverTimer.reset();
            stateDelayTime = 0.5;
            nextARMMOVERState = ARMMOVER_STATES.ARMMOVER_SPECIMINE_HANG_POS;
        }
    }

    public ARMMOVER_STATES getArmMoverState() {
        if (nextARMMOVERState != curARMMOVERState) {
            if (armMoverTimer.time() > stateDelayTime) {
                curARMMOVERState = nextARMMOVERState;
            }
        }
        return curARMMOVERState;
    }
}