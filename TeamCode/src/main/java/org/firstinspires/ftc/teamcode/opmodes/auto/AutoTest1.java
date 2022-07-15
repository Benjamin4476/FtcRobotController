package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name="AutoTest1")
public class AutoTest1 extends CommandOpMode {
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

    @Override
    public void initialize() {

    }
}
