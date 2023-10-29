package org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
public class Robot {
    public MecanumDrivetrain drive;
    public Intake intake;
    public Lift lift;
    public Robot(CommandOpMode opMode, boolean auto) {
        drive = new MecanumDrivetrain(opMode, auto);
        intake = new Intake(opMode);
        lift = new Lift(opMode, auto);
        opMode.register(drive, intake, lift);
    }
}
