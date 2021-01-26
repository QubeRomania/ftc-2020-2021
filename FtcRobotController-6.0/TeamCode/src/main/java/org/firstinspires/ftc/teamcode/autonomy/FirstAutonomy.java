package org.firstinspires.ftc.teamcode.autonomy;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class FirstAutonomy extends OpMode {

    AutonomyBase bot;

    private enum State {BEGIN, AWAY, PAUSE, PAUSE1, PAUSE2, PAUSE3, PAUSE4, PAUSE5, AWAY1, AWAY2, AWAY3, AWAY4, AWAY5, AWAY6, DONE}
    State state = State.BEGIN;

    NanoClock clock;
    double startPause;

    Trajectory traj;

    public void init(){
        bot = new AutonomyBase(hardwareMap);
        clock = NanoClock.system();
    }

    public void loop() {
        bot.updatePoseEstimate();
        Pose2d currentPose = bot.getPoseEstimate();

        telemetry.addData("POSE", "X = %.2f  Y = %.2f  H = %.1f", currentPose.getX(),
                currentPose.getY(), Math.toDegrees(currentPose.getHeading()));

        switch (state) {
            case BEGIN:
                state = State.AWAY;
                traj = new TrajectoryBuilder(new Pose2d(), bot.constraints)
                        .strafeLeft(5)
                        .splineToConstantHeading(new Vector2d(65,20),0)
                        .build();
                bot.follower.followTrajectory(traj);
                break;
            case AWAY:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE;
                    bot.setDriveSignal(new DriveSignal());
                    startPause = clock.seconds();
                }
                break;
            case PAUSE:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY1;
                    traj = new TrajectoryBuilder(new Pose2d(65, 20, 0), bot.constraints)
                            .strafeTo(new Vector2d(115, -22.5))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY1:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE1;
                    bot.setDriveSignal(new DriveSignal());
                }
                break;
            case PAUSE1:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY2;
                    traj = new TrajectoryBuilder(new Pose2d(115, -22.5, 0), bot.constraints)
                            .strafeTo(new Vector2d(0, -22.5))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY2:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE2;
                    bot.setDriveSignal(new DriveSignal());
                }
                break;
            case PAUSE2:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY3;
                    traj = new TrajectoryBuilder(new Pose2d(0, -22.5, 0), bot.constraints)
                            .strafeTo(new Vector2d(10,0))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY3:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE3;
                    bot.setDriveSignal(new DriveSignal());
                }
                break;
            case PAUSE3:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY4;
                    traj = new TrajectoryBuilder(new Pose2d(10, 0, 0), bot.constraints)
                            .strafeTo(new Vector2d(65,0))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY4:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE4;
                    bot.setDriveSignal(new DriveSignal());
                }
                break;
            case PAUSE4:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY5;
                    traj = new TrajectoryBuilder(new Pose2d(65, 0, 0), bot.constraints)
                            .strafeTo(new Vector2d(115,-22.5))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY5:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.PAUSE5;
                    bot.setDriveSignal(new DriveSignal());
                }
                break;
            case PAUSE5:
                if ((clock.seconds() - startPause) > 2.0) {
                    state = State.AWAY6;
                    traj = new TrajectoryBuilder(new Pose2d(115, -22.5, 0), bot.constraints)
                            .strafeTo(new Vector2d(65,-22.5))
                            .build();
                    bot.follower.followTrajectory(traj);
                }
                break;
            case AWAY6:
                bot.setDriveSignal(bot.follower.update(currentPose));
                if (!bot.follower.isFollowing()) {
                    state = State.DONE;
                    bot.setDriveSignal(new DriveSignal());
                }
                break;
            case DONE:
                break;
        }
    }

}
