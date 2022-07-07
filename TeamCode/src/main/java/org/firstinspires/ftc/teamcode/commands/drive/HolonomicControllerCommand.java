package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.HolonomicDriveController;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class HolonomicControllerCommand extends CommandBase {
    private final ElapsedTime m_timer = new ElapsedTime();
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final HolonomicDriveController m_controller;
    private final Consumer<ChassisSpeeds> m_speeds;
    private final Supplier<Rotation2d> m_desiredRotation;

    public HolonomicControllerCommand(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Supplier<Rotation2d> desiredRotation,
            Consumer<ChassisSpeeds> outputSpeeds,
            Subsystem... requirements
    ) {
        this.m_trajectory = trajectory;
        this.m_pose = pose;
        this.m_controller = new HolonomicDriveController(xController, yController, thetaController);
        this.m_desiredRotation = desiredRotation;
        this.m_speeds = outputSpeeds;

        addRequirements(requirements);
    }


    @SuppressWarnings("ParameterName")
    public HolonomicControllerCommand(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<ChassisSpeeds> outputSpeeds,
            Subsystem... requirements
    ) {
        this(
                trajectory,
                pose,
                xController,
                yController,
                thetaController,
                () -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
                outputSpeeds,
                requirements
        );
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.startTime();
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = m_timer.time();
        Trajectory.State desiredState = m_trajectory.sample(curTime);

        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());

        m_speeds.accept(targetChassisSpeeds);
    }

    @Override
    public boolean isFinished() {
        return m_timer.time() >= m_trajectory.getTotalTimeSeconds();
    }
}
