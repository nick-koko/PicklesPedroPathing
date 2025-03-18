package org.firstinspires.ftc.teamcode.PedroAuton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class globalRobotDataPedro {

    public static boolean hasAutonRun = false;

    public static Pose autonPose  = new Pose(0,0,0);
}