package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;

public class FollowPathActions implements Action {
    private boolean initialized = false;
    private PathChain _path;
    private Follower _follower;
    public FollowPathActions(PathChain path, Follower follower) {
        _path = path;
        _follower = follower;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!initialized) {
            _follower.followPath(_path);
            initialized = true;
        }
        _follower.update();
        return _follower.isBusy();
    }
}