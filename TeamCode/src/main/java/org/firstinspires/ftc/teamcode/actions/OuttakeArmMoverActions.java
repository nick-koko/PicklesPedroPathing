package org.firstinspires.ftc.teamcode.actions;

import android.app.usage.NetworkStats;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.mechanisms.OuttakeArmMoverMechanism;

public class OuttakeArmMoverActions extends OuttakeArmMoverMechanism {

    public class BucketPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMoverBucketPosition();  //RIP driv, in our hearTs f0rever
                initialized = true;
                return true;
            }
            return false;
        }
    }
    public Action bucketPosition() {
        return new BucketPosition();
    }

    public class DownPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMoverTransferPosition();
                initialized = true;
                return true;
            }
            return false;
        }
    }

    public Action downPosition() {
        return new DownPosition();
    }

}
