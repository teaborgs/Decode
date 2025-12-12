package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.roadrunner.Pose2d;


public class waypointsRED
{
	public static final Pose2d START      = new Pose2d(0.0,   0.0, Math.toRadians(0));
	public static final Pose2d SHOOT_BACK = new Pose2d(15.95, 0, Math.toRadians(0));
	public static final Pose2d PICKUP1    = new Pose2d(25, -23, Math.toRadians(-90));
	public static final Pose2d PARK       = new Pose2d(25, -23, Math.toRadians(1.0));
	public static final Pose2d FPICKUP1   = new Pose2d(24.0, -52.29, Math.toRadians(-90));
	public static final Pose2d FINISH     = new Pose2d(24, -5, Math.toRadians(90));
}
