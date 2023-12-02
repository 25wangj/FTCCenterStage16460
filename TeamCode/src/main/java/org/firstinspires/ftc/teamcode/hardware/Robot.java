package org.firstinspires.ftc.teamcode.hardware;
import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO;
import static org.firstinspires.ftc.teamcode.hardware.RobotStateMachine.robotStates.*;
import static org.firstinspires.ftc.teamcode.hardware.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Lift.*;
import static org.firstinspires.ftc.teamcode.hardware.Climb.*;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.StateMachine;
public class Robot {
    public final MecanumDrive drive;
    public final Intake intake;
    public final Lift lift;
    public final Climb climb;
    public StateMachine<RobotStateMachine.robotStates> stateMachine;
    private LynxModule exhub;
    public Robot(CommandOpMode opMode, boolean auto) {
        for (LynxModule hub : opMode.hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(AUTO);
        }
        exhub = opMode.hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        drive = new MecanumDrive(opMode, auto);
        intake = new Intake(opMode);
        lift = new Lift(opMode, exhub, auto);
        climb = new Climb(opMode, auto);
        stateMachine = RobotStateMachine.get(opMode, this, INTAKE_OPEN);
        opMode.register(drive, intake, lift);
        if (!auto) {
            intake.setPower(intakeOpen);
        }
        intake.setRoller(rollerDown);
        intake.setGate(gateOpen);
        lift.setClaw(clawOpen);
        climb.setLatch(latchClosed);
        climb.setPlane(planeHold);
    }
}