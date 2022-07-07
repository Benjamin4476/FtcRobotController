package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name="AutoTest3")
public class AutoTest1 extends CommandOpMode {
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

    @Override
    public void initialize() {

    }
}
