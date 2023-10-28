package org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
public class Robot {
    public MecanumDrivetrain drive;
    public Robot(CommandOpMode opMode, boolean auto) {
        drive = new MecanumDrivetrain(opMode, auto);
        opMode.register(drive);
    }
}
