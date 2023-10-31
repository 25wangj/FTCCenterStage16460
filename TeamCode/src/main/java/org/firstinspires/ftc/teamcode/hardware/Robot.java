package org.firstinspires.ftc.teamcode.hardware;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.StateMachine;
public class Robot {
    public MecanumDrivetrain drive;
    public Intake intake;
    public Lift lift;
    public StateMachine<RobotStateMachine.robotStates> stateMachine;
    public Robot(CommandOpMode opMode, boolean auto) {
        drive = new MecanumDrivetrain(opMode, auto);
        intake = new Intake(opMode);
        lift = new Lift(opMode, auto);
        stateMachine = RobotStateMachine.get(opMode, this, INTAKE_OPEN);
        opMode.register(drive, intake, lift);
        if (auto) {
            intake.setGate(gateUp);
            intake.setRoller(rollerDown);
            lift.setClaw(clawOpen);
        } else {
            intake.setPower(intakeOpen);
            intake.setGate(gateOpen);
            intake.setRoller(rollerDown);
            lift.setClaw(clawOpen);
        }
    }
}
