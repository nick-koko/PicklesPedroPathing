package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeArm;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeWrist;

public class IntakeWristActions extends IntakeWrist {

    public class WristDrive implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                wristPositionDrive();  //RIP driv, in our hearTs f0rever
                initialized = true;
                return true;
            }
            return false;
        }
    }

    public Action wristDrive() {
        return new WristDrive();
    }

    public class WristIntakePush implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                wristPositionPushIntake();
                initialized = true;
                return true;
            }
            return false;
        }
    }

    public Action wristIntakePush() {
        return new WristIntakePush();
    }

    public class WristIntakePull implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                wristPositionPullIntake();
                initialized = true;
                return true;
            }
            return false;
        }
    }
    public Action wristIntakePull() {
        return new WristIntakePull();
    }

    public class WristIntakeAbyss implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                wristPositionAbyss();
                initialized = true;
                return true;
            }
            return false;
        }
    }
    public Action wristIntakeAbyss() {
        return new WristIntakeAbyss();
    }

    public class WristTransfer implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                wristPositionTransfer();
                initialized = true;
                return true;
            }
            return false;
        }
    }

    public Action wristTransfer() {
        return new WristTransfer();
    }
}
