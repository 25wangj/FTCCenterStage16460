package org.firstinspires.ftc.teamcode.command;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class CommandOpMode extends LinearOpMode {
    protected Scheduler scheduler;
    protected ElapsedTime clock;
    public abstract void initOpMode();
    public void waitOpMode() {};
    public void startOpMode() {};
    @Override
    public void runOpMode() {
        scheduler = new Scheduler(clock);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initOpMode();
        while (!isStarted() && !isStopRequested()) {
            waitOpMode();
            scheduler.run(false);
        }
        startOpMode();
        while(opModeIsActive()) {
            scheduler.run(true);
        }
    }
}
