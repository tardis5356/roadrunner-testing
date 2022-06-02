package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.TestingMecanumDrive;
public class autoTrajectories {
    public Pose2d startPoseRC = new Pose2d(-31, -65.75, 0);
    public Pose2d startPoseRW = new Pose2d(14, -65.75, 0);
    public Pose2d startPoseBC = new Pose2d(-41, 65.75, Math.toRadians(180));
    public Pose2d startPoseBW = new Pose2d(10, 65.75, Math.toRadians(180));

    public Vector2d redCarousel = new Vector2d(-63, -58.25);
    public Vector2d blueCarousel = new Vector2d(-65, 57.75);
}
