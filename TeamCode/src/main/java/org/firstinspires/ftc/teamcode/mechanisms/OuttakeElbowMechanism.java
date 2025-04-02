package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class OuttakeElbowMechanism {
    // Assuming some motor control library is used, e.g., FTC SDK, but this can be customized
    private Servo outtakeElbowServo;

    // Target positions for the servo arm
    public static double DRIVE_POSITION = 1;
    public static double TRANSFER_POSITION = 1;
    public static double BUCKET_POSITION = 1;
    public static double SPECIMINE_GRAB_POSITION = 1;
    public static double SPECIMINE_HANG_POSITION = 1;

    public enum OUTTAKEELBOW_STATES{
        OUTTAKEELBOW_DRIVE_POS, OUTTAKEELBOW_TRANSFER_POS, OUTTAKEELBOW_BUCKET_POS, OUTTALEELBOW_SPECIMINE_GRAB_POS, OUTTAKEELBOW_SPECIMINE_HANG_POS
    }

    private OUTTAKEELBOW_STATES curOUTTAKEELBOWState = null;
    private OUTTAKEELBOW_STATES nextOUTTAKEELBOWState = null;
    private ElapsedTime armMoverTimer = new ElapsedTime();

    double stateDelayTime = 0;
    public void init(HardwareMap hwMap) {

        outtakeElbowServo = hwMap.get(Servo.class, "outtake_elbow_servo");
        this.outtakeElbowServo.setDirection(Servo.Direction.FORWARD);
        curOUTTAKEELBOWState = OUTTAKEELBOW_STATES.OUTTAKEELBOW_DRIVE_POS;
        nextOUTTAKEELBOWState = OUTTAKEELBOW_STATES.OUTTAKEELBOW_DRIVE_POS;
    }

    // Method to move the armMover to the down position
    public void outtakeElbowDrivePosition() {

        outtakeElbowServo.setPosition(DRIVE_POSITION);

        if (curOUTTAKEELBOWState != OUTTAKEELBOW_STATES.OUTTAKEELBOW_DRIVE_POS) {
            armMoverTimer.reset();
            stateDelayTime = 0.5;
            nextOUTTAKEELBOWState = OUTTAKEELBOW_STATES.OUTTAKEELBOW_DRIVE_POS;
        }
    }

    // Method to move the armMover to the dump position

    public void outtakeElbowBucketPosition() {

        outtakeElbowServo.setPosition(BUCKET_POSITION);
        if (curOUTTAKEELBOWState != OUTTAKEELBOW_STATES.OUTTAKEELBOW_BUCKET_POS) {
            armMoverTimer.reset();
            stateDelayTime = 0.5;
            nextOUTTAKEELBOWState = OUTTAKEELBOW_STATES.OUTTAKEELBOW_BUCKET_POS;
        }
    }
    public void outtakeElbowTransferPosition() {

        outtakeElbowServo.setPosition(TRANSFER_POSITION);
        if (curOUTTAKEELBOWState != OUTTAKEELBOW_STATES.OUTTAKEELBOW_TRANSFER_POS) {
            armMoverTimer.reset();
            stateDelayTime = 0.5;
            nextOUTTAKEELBOWState = OUTTAKEELBOW_STATES.OUTTAKEELBOW_TRANSFER_POS;
        }
    }
    public void outtakeElbowSpecimineGrabPosition() {
        outtakeElbowServo.setPosition(SPECIMINE_GRAB_POSITION);
        if (curOUTTAKEELBOWState != OUTTAKEELBOW_STATES.OUTTALEELBOW_SPECIMINE_GRAB_POS) {
            armMoverTimer.reset();
            stateDelayTime = 0.5;
            nextOUTTAKEELBOWState = OUTTAKEELBOW_STATES.OUTTALEELBOW_SPECIMINE_GRAB_POS;
        }
    }
    public void outtakeElbowSpecimineHangPosition() {
        outtakeElbowServo.setPosition(SPECIMINE_HANG_POSITION);
        if (curOUTTAKEELBOWState != OUTTAKEELBOW_STATES.OUTTAKEELBOW_SPECIMINE_HANG_POS) {
            armMoverTimer.reset();
            stateDelayTime = 0.5;
            nextOUTTAKEELBOWState = OUTTAKEELBOW_STATES.OUTTAKEELBOW_SPECIMINE_HANG_POS;
        }
    }

    public OUTTAKEELBOW_STATES getOuttakeElbowState() {
        if (nextOUTTAKEELBOWState != curOUTTAKEELBOWState) {
            if (armMoverTimer.time() > stateDelayTime) {
                curOUTTAKEELBOWState = nextOUTTAKEELBOWState;
            }
        }
        return curOUTTAKEELBOWState;
    }
}