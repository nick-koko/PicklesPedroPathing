package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

public class FollowPathActions implements Action {
    private boolean initialized = false;
    private PathChain _path;
    private Follower _follower;
    private boolean _holdEnd;
    public FollowPathActions(PathChain path, Follower follower, boolean holdEnd) {
        _path = path;
        _follower = follower;
        _holdEnd = holdEnd;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!initialized) {
            _follower.followPath(_path, _holdEnd);
            initialized = true;
        }
        _follower.update();
        return _follower.isBusy();
    }
}