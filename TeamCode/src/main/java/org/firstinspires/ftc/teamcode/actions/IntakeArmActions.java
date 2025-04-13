package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeArm;

public class IntakeArmActions extends IntakeArm {

    public class ArmDrive implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armPositionDrive();  //RIP driv, in our hearTs f0rever
                initialized = true;
                return true;
            }
            return false;
        }
    }

    public Action armDrive() {
        return new ArmDrive();
    }

    public class ArmIntakeAbyss implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armPositionAbyss();
                initialized = true;
                return true;
            }
            return false;
        }
    }

    public Action armIntakeAbyss() {
        return new ArmIntakeAbyss();
    }
    public class ArmIntakePush implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armPositionIntake();
                initialized = true;
                return true;
            }
            return false;
        }
    }

    public Action armIntakePush() {
        return new ArmIntakePush();
    }

    public class ArmIntakePull implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armPositionAutonPullIntake();
                initialized = true;
                return true;
            }
            return false;
        }
    }

    public Action armIntakePull() {
        return new ArmIntakePull();
    }
    public class ArmTransfer implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armPositionTransfer();
                initialized = true;
                return true;
            }
            return false;
        }
    }

    public Action armTransfer() {
        return new ArmTransfer();
    }
}
