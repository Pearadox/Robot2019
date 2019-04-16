package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
import frc.robot.commands.*;

public class LaunchpadManager {

    double watchdogTimerSeconds = 1.0;
    final String tableName = "Launchpad";
    boolean btns[][] = new boolean[9][9];
    boolean presses[][] = new boolean[9][9];
    boolean lastBtns[][] = new boolean[9][9];

    boolean isConnected = false;
    double lastReceivedPing = 0;
    boolean lastPingValue = false;

    NetworkTableInstance nt;
    NetworkTable table;
    NetworkTableEntry pingEntryRio;
    NetworkTableEntry pingEntryLaunchpad;

    public LaunchpadManager() {
        nt = NetworkTableInstance.getDefault();
        table = nt.getTable(tableName);
        pingEntryRio = table.getEntry("pingValueRio");
        pingEntryLaunchpad = table.getEntry("pingValueLaunchpad");
    }

    public void periodicLoop() {
        // get ping value posted by launchpad
        boolean launchpadPingValue = pingEntryLaunchpad.getBoolean(false);

        // check for ping
        if(lastPingValue != launchpadPingValue) {
            lastPingValue = launchpadPingValue;
            lastReceivedPing = Timer.getFPGATimestamp();
        }
        
        // decide if connected or not
        if(Timer.getFPGATimestamp() - lastReceivedPing > watchdogTimerSeconds) {
            isConnected = false;
        }
        else isConnected = true;
        SmartDashboard.putBoolean("Launchpad Connection", isConnected);

        // send ping
        pingEntryRio.setBoolean(!pingEntryRio.getBoolean(false));

        // get buttons
        for(int r = 0; r < 9; r++) {
            for(int c = 0; c < 9; c++) {
                String key = r + ":" + c;
                lastBtns[r][c] = btns[r][c];
                btns[r][c] = isConnected ? table.getEntry(key).getBoolean(false) : false;
                presses[r][c] = false;
                if(btns[r][c] && !lastBtns[r][c]) presses[r][c] = true;
            }
        }

        // connection button test
        if(btns[8][8]) SmartDashboard.putBoolean("Launchpad Button Test", true);
        else SmartDashboard.putBoolean("Launchpad Button Test", false);
    }

    public void disabledLoop() {
        
    }

    public void teleopLoop() {
        if(btns[0][0] && !climberOpenOverride.isRunning()) climberOpenOverride.start();
        else if(!btns[0][0]) climberOpenOverride.cancel();

        if(btns[0][1] && !climberCloseOverride.isRunning()) climberCloseOverride.start();
        else if(!btns[0][1]) climberCloseOverride.cancel();

        if(btns[0][2] && !armResetZero.isRunning()) armResetZero.start();
        else if(!btns[0][2]) armResetZero.cancel();

        if(presses[0][6]) armDownGroup.start();

        if(presses[0][7]) armUpGroup.start();

        if(btns[1][0] && !climberOpen.isRunning()) climberOpen.start();
        else if(!btns[1][0]) climberOpen.cancel();

        if(btns[1][1] && !climberClose.isRunning()) climberClose.start();
        else if(!btns[1][1]) climberClose.cancel();

        if(btns[1][4] && !limelightAuto.isRunning()) limelightAuto.start();
        else if(!btns[1][4]) limelightAuto.cancel();

        if(btns[1][5] && !limelightHold.isRunning()) limelightHold.start();
        else if(!btns[1][5]) limelightHold.cancel();

        if(presses[1][6]) mothClose.start();

        if(presses[1][7]) mothOpen.start();

        if(presses[1][8]) mothToggle.start();

        if(btns[2][4] && !outtakeBoth.isRunning()) outtakeBoth.start();
        else if(!btns[2][4]) outtakeBoth.cancel();

        if(btns[2][5] && !intakeBoth.isRunning()) intakeBoth.start();
        else if(!btns[2][5]) intakeBoth.cancel();

        if(presses[2][6]) intakeDown.start();

        if(presses[2][7]) intakeUp.start();

        if(presses[2][8]) intakeToggle.start();

        if(presses[3][5]) armSetCamera.start();

        if(btns[3][6] && !armManualDown.isRunning()) armManualDown.start();
        else if(!btns[3][6]) armManualDown.cancel();

        if(btns[3][7] && !armManualUp.isRunning()) armManualUp.start();
        else if(!btns[3][7]) armManualUp.cancel();

        if(presses[3][8]) armSetCargo.start();

        if(presses[4][8]) armSetRocket.start();

        if(presses[5][8]) armSetLow.start();

        if(btns[5][5] && !rightRocketBack.isRunning()) rightRocketBack.start();
        else if(!btns[5][5]) rightRocketBack.cancel();

        if(btns[6][5] && !rightRocketFront.isRunning()) rightRocketFront.start();
        else if(!btns[6][5]) rightRocketFront.cancel();

        if(btns[5][2] && !leftRocketBack.isRunning()) leftRocketBack.start();
        else if(!btns[5][2]) leftRocketBack.cancel();

        if(btns[6][2] && !leftRocketFront.isRunning()) leftRocketFront.start();
        else if(!btns[6][2]) leftRocketFront.cancel();

        // if(presses[6][8]) mothToggle2.start();
        //make open and close buttons
        
    }

    /*

    Mappings:
    0:0 climber open override  --  while
    0:1 climber close override  --  while
    0:2 arm reset zero  --  while
    0:4 cancel auto  --  function
    0:6 arm down group  --  when
    0:7 arm up group  --  when
    1:0 climber open  --  while
    1:1 climber close  --  while
    1:4 limelight auto  --  while
    1:5 limelight hold  --  while
    1:6 moth close  --  when
    1:7 moth open  --  when
    1:8 moth toggle  --  when
    2:4 outtake both  --  while
    2:5 intake both  --  while
    2:6 intake down  --  when
    2:7 intake up  --  when
    2:8 intake toggle  --  when
    3:5 arm set camera -- when
    3:6 arm manual down  --  while
    3:7 arm manual up  --  while
    3:8 arm set cargo  --  when
    4:8 arm set rocket  --  when
    5:8 arm set low  --  when

    */

    // commands
    Command climberOpenOverride = new ClimberSetOverride(0.3);
    Command climberCloseOverride = new ClimberSetOverride(-0.3);
    Command armResetZero = new ArmZeroReset();
    Command armDownGroup = new DriverLowerGroup(false);
    Command armUpGroup = new DriverRaiseGroup();
    Command climberOpen = new ClimberSet(0.5,.3);
    Command climberClose = new ClimberSet(-0.5,0);
    Command limelightAuto = new VisionHatchPlacer();
    Command limelightHold = new VisionHoldOnTarget();
    Command mothClose = new MothClose();
    Command mothOpen = new MothOpen();
    Command mothToggle = new MothToggle();
    Command outtakeBoth = new OuttakeBoth();
    Command intakeBoth = new IntakeBoth(false);
    Command intakeDown = new IntakeLower();
    Command intakeUp = new IntakeRaise();
    Command intakeToggle = new IntakeToggle();
    Command armManualDown = new ArmManualDown();
    Command armManualUp = new ArmManualUp();
    Command armSetCargo = new ArmGoCargo();
    Command armSetRocket = new ArmGoRocket();
    Command armSetLow = new ArmGoLow();
    Command armSetCamera = new ArmGoCamera();
    Command mothToggle2 = new MothToggle2();

    Command rightRocketBack= new PathLSRtoRR(2, false, 1);
    Command rightRocketFront = new PathLSRtoRR(1, false, 1);
    Command leftRocketBack = new PathLSRtoLR(2, false, 1);
    Command leftRocketFront = new PathLSRtoLR(1, false, 1);
}