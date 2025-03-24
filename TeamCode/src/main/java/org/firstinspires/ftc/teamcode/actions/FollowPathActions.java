package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FollowPathActions implements Action {
    private boolean initialized = false;
    private PathChain _path;
    private Follower _follower;
    private boolean _holdEnd;
    private Telemetry _telemetry;

    public FollowPathActions(PathChain path, Follower follower, boolean holdEnd, Telemetry telemetryA) {
        _path = path;
        _follower = follower;
        _holdEnd = holdEnd;
        _telemetry = telemetryA;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!initialized) {
            _follower.followPath(_path, _holdEnd);
            initialized = true;
        }
        _follower.update();
        _follower.telemetryDebug(_telemetry);

        return _follower.isBusy();
    }
}