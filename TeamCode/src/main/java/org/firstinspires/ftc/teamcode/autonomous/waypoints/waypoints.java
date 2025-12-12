package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.roadrunner.Pose2d;


public class waypoints
{
	public static final Pose2d START      = new Pose2d(0.0,   0.0, Math.toRadians(0));
	public static final Pose2d SHOOT_BACK = new Pose2d(11.52, 0, Math.toRadians(0));
	public static final Pose2d PICKUP1    = new Pose2d(23.63, 20.75, Math.toRadians(90));
	public static final Pose2d PARK       = new Pose2d(24.67, 29.47, Math.toRadians(1.0));
	public static final Pose2d FPICKUP1   = new Pose2d(22.8, 50.29, Math.toRadians(88));
	public static final Pose2d FINISH     = new Pose2d(25.55, 3.73, Math.toRadians(-91.16));
}
