package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Rotation2d;

public class Rotation2dExtended extends Rotation2d {
    @SuppressWarnings("ParameterName")
    public Rotation2d interpolate(Rotation2d endValue, double t) {
        return new Rotation2d(MathUtil.interpolate(this.getRadians(), endValue.getRadians(), t));
    }
}
