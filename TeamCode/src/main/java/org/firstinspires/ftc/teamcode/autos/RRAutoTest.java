package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.drive.TestingMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "RRAutoTest", group = "autoTest")
public class RRAutoTest extends LinearOpMode {
    public void runOpMode() {
        TestingMecanumDrive drive = new TestingMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(60, 0, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(30, 30), Math.toRadians(90))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(40, 0), Math.toRadians(180))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(60, 0), Math.toRadians(180))
                .build();

        drive.followTrajectory(traj);
        sleep(1000);
        drive.followTrajectory(traj2);
        sleep(1000);
        drive.followTrajectory(traj3);
        sleep(1000);
        drive.followTrajectory(traj4);

    }
}
