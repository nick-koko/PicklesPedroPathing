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
    public static double SIDEWAYS_POSITION = .468;
    public static double UP_POSITION = .12;
    public static double DOWN_POSITION = .825;
    public static double lastCommandedPosition = SIDEWAYS_POSITION; // Initialize to sideways position
    public static double INCREMENT_AMOUNT = 0.001; // Amount to change position by each loop

    private ElapsedTime wristTimer = new ElapsedTime();
    public enum OUTTAKE_WRIST_STATES{
        OUTTAKE_WRIST_SIDEWAYS_POS, OUTTAKE_WRIST_UP_POSITION, OUTTAKE_WRIST_DOWN_POSITION, OUTTAKE_WRIST_DRIVE_POS, OUTTAKE_WRIST_BETWEEN_POSITIONS
    }

    private OUTTAKE_WRIST_STATES curWRISTState = null;
    private OUTTAKE_WRIST_STATES nextWRISTState = null;

    double stateDelayTime = 0;

    public void init(HardwareMap hwMap) {
        outtakeWristServo = hwMap.get(Servo.class, "outtake_wrist_servo"); //༼ * ◕_◕ ༽*
        this.outtakeWristServo.setDirection(Servo.Direction.FORWARD);
        curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_DRIVE_POS;
        nextWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_DRIVE_POS;
        lastCommandedPosition = SIDEWAYS_POSITION;
    }

    // Method to move the wrist to the outtake position
    public void wristPositionSideways() {
        outtakeWristServo.setPosition(SIDEWAYS_POSITION);
        lastCommandedPosition = SIDEWAYS_POSITION;
        curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_SIDEWAYS_POS;
        nextWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_SIDEWAYS_POS;
    }
    public void wristPositionUp() {
        outtakeWristServo.setPosition(UP_POSITION);
        lastCommandedPosition = UP_POSITION;
        curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_UP_POSITION;
        nextWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_UP_POSITION;
    }
    public void wristPositionDown() {
        outtakeWristServo.setPosition(DOWN_POSITION);
        lastCommandedPosition = DOWN_POSITION;
        curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_DOWN_POSITION;
        nextWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_DOWN_POSITION;
    }
    public OUTTAKE_WRIST_STATES getWristState() {
        return curWRISTState;
    }

    // Get current servo position
    public double getCurrentPosition() {
        return outtakeWristServo.getPosition();
    }

    // Move wrist incrementally towards UP position
    public void moveTowardsUp() {
        double currentPos = lastCommandedPosition;
        if (currentPos > UP_POSITION) {
            double newPosition = Math.max(currentPos - INCREMENT_AMOUNT, UP_POSITION);
            outtakeWristServo.setPosition(newPosition);
            lastCommandedPosition = newPosition;
            if (newPosition == UP_POSITION) {
                curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_UP_POSITION;
            } else {
                curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_BETWEEN_POSITIONS;
            }
        }
    }

    // Move wrist incrementally towards DOWN position
    public void moveTowardsDown() {
        double currentPos = lastCommandedPosition;
        if (currentPos < DOWN_POSITION) {
            double newPosition = Math.min(currentPos + INCREMENT_AMOUNT, DOWN_POSITION);
            outtakeWristServo.setPosition(newPosition);
            lastCommandedPosition = newPosition;
            if (newPosition == DOWN_POSITION) {
                curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_DOWN_POSITION;
            } else {
                curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_BETWEEN_POSITIONS;
            }
        }
    }

    // Sync the servo position with the lastCommandedPosition
    public void syncServoToLastCommandedPosition() {
        outtakeWristServo.setPosition(lastCommandedPosition);
        // Update state based on position
        if (lastCommandedPosition == UP_POSITION) {
            curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_UP_POSITION;
        } else if (lastCommandedPosition == DOWN_POSITION) {
            curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_DOWN_POSITION;
        } else if (lastCommandedPosition == SIDEWAYS_POSITION) {
            curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_SIDEWAYS_POS;
        } else {
            curWRISTState = OUTTAKE_WRIST_STATES.OUTTAKE_WRIST_BETWEEN_POSITIONS;
        }
    }
}