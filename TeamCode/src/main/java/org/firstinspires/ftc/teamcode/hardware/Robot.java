package org.firstinspires.ftc.teamcode.hardware;
import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;

import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.StateMachine;
public class Robot {
    public MecanumDrive drive;
    public Intake intake;
    public Lift lift;
    public StateMachine<RobotStateMachine.robotStates> stateMachine;
    public Robot(CommandOpMode opMode, boolean auto) {
        for (LynxModule hub : opMode.hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(AUTO);
        }
        drive = new MecanumDrive(opMode, auto);
        intake = new Intake(opMode);
        lift = new Lift(opMode, auto);
        stateMachine = RobotStateMachine.get(opMode, this, INTAKE_OPEN);
        opMode.register(drive, intake, lift);
        if (auto) {
            intake.setGate(gateOpen);
            intake.setRoller(rollerUp);
            lift.setClaw(clawOpen);
        } else {
            intake.setPower(intakeOpen);
            intake.setGate(gateOpen);
            intake.setRoller(rollerDown);
            lift.setClaw(clawOpen);
        }
    }
}
