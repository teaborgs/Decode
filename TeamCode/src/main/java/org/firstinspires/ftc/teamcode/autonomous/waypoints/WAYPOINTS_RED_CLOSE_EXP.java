package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.roadrunner.Pose2d;

public class WAYPOINTS_RED_CLOSE_EXP
{
	public static final Pose2d START =
			new Pose2d(0.0, 0.0, Math.toRadians(0));

	// shoot -27.15 -52.59
	public static final Pose2d SHOOT =
			new Pose2d(-27.15, -47, Math.toRadians(0));

	// pickup1 15.25 -52.59
	public static final Pose2d PICKUP1 =
			new Pose2d(15.25, -47, Math.toRadians(0));

	// opengatestart 2.94 -62.02
	public static final Pose2d OPENGATES =
			new Pose2d(2.94, -62.02, Math.toRadians(0));

	// opengate 15.05 -62.02
	public static final Pose2d OPENGATE =
			new Pose2d(15.05, -62.02, Math.toRadians(0));

	// pickup2s -11.39 74.5
	public static final Pose2d PICKUP2S =
			new Pose2d(-11.39, -72.5, Math.toRadians(0));  // inceput pickup preventiv

	// pickup2 20.4 -75.3
	public static final Pose2d PICKUP2 =
			new Pose2d(20.4, -72.5, Math.toRadians(0));

	// finish 1.1 -63.82
	public static final Pose2d FINISHLINE =
			new Pose2d(1.1, -63.82, Math.toRadians(0));

	// pickup3s -9.2 -98.9
	public static final Pose2d PICKUP3S =
			new Pose2d(-9.2, -93, Math.toRadians(0));

	// pickup3 20.4 -97.95
	public static final Pose2d PICKUP3 =
			new Pose2d(20.4, -93, Math.toRadians(0));

	// restul le-am lăsat neschimbate (nu erau în lista ta)
	public static final Pose2d SHOOT_EXP =
			new Pose2d(19.4, 0.0, Math.toRadians(0));

	public static final Pose2d PICKUP3L =
			new Pose2d(74.28, 51.4, Math.toRadians(138));

	public static final Pose2d FINISHPICKUP =
			new Pose2d(52.7582, 72.198, Math.toRadians(0));
}
