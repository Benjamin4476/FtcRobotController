// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.utils;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;

@SuppressWarnings("MemberName")
public class HolonomicDriveController {
    private Pose2d m_poseError = new Pose2d();
    private Rotation2d m_rotationError = new Rotation2d();
    private Pose2d m_poseTolerance = new Pose2d();
    private boolean m_enabled = true;

    private final PIDController m_xController;
    private final PIDController m_yController;
    private final ProfiledPIDController m_thetaController;

    private boolean m_firstRun = true;

    @SuppressWarnings("ParameterName")
    public HolonomicDriveController(
            PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
    }

    public boolean atReference() {
        final Translation2d eTranslate = m_poseError.getTranslation();
        final Rotation2d eRotate = m_rotationError;
        final Translation2d tolTranslate = m_poseTolerance.getTranslation();
        final Rotation2d tolRotate = m_poseTolerance.getRotation();
        return Math.abs(eTranslate.getX()) < tolTranslate.getX()
                && Math.abs(eTranslate.getY()) < tolTranslate.getY()
                && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
    }

    public void setTolerance(Pose2d tolerance) {
        m_poseTolerance = tolerance;
    }

    @SuppressWarnings("LocalVariableName")
    public ChassisSpeeds calculate(
            Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, Rotation2d angleRef) {
        // If this is the first run, then we need to reset the theta controller to the current pose's
        // heading.
        if (m_firstRun) {
            m_thetaController.reset(currentPose.getRotation().getRadians());
            m_firstRun = false;
        }

        // Calculate feedforward velocities (field-relative).
        double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
        double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
        double thetaFF =
                m_thetaController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());

        m_poseError = poseRef.relativeTo(currentPose);
        m_rotationError = angleRef.minus(currentPose.getRotation());

        if (!m_enabled) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
        }

        // Calculate feedback velocities (based on position error).
        double xFeedback = m_xController.calculate(currentPose.getX(), poseRef.getX());
        double yFeedback = m_yController.calculate(currentPose.getY(), poseRef.getY());

        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    }

    public ChassisSpeeds calculate(
            Pose2d currentPose, Trajectory.State desiredState, Rotation2d angleRef) {
        return calculate(
                currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, angleRef);
    }

    public void setEnabled(boolean enabled) {
        m_enabled = enabled;
    }
}
