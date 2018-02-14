
package org.usfirst.frc.team4787.robot;

//2018

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.IterativeRobot;



/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
public class Robot extends SampleRobot {
	// min value in x and y direction for robot to respond to joystick movements
	final double DEADZONEX = 0.01, DEADZONEY = 0.01;
	
    // button configuration
	final int  GEAR_MECH_DOOR_OPEN = 3, GEAR_MECH_DOOR_CLOSE = 5, DISABLE_CLIMBING_MECH = 4, NewBoi = 7, Reduced = 8;
	/**
	 * Change configuration for pneumatics/ clear out mechDoor
	 */
	boolean speedReduced = false;
	boolean newnewboi = false;
    // 0 = joystick usb port in driver station
    Joystick stick = new Joystick(2);
    
    // number in parenthesis for Talon, Servo, and Spark constructor are PWM ports on roborio
    Talon bLeft = new Talon(2);
    Talon fLeft = new Talon(3);
    Talon bRight= new Talon(1);
    Talon fRight = new Talon(0);
//    Servo rightGearDoorServo = new Servo(5);
//    Servo leftGearDoorServo = new Servo(6);
    Spark climbingMech = new Spark(7);
    //paramater is nodeid of the solenoids & compressor 
    DoubleSolenoid sol1 = new DoubleSolenoid(1, 2);
    Compressor c = new Compressor(0);
    /*SendableChooser chooser = new SendableChooser();
    
    chooser.addDefault("Default", new autonomous());
    
/*    
    SendableChooser chooser = new SendableChooser();
    chooser.addObject(a1, String);
    chooser.addObject("Autonomous 2", String);
    chooser.addObject("Autonomous 3", String);
    chooser.addObject("Autonomous 4", String);
    chooser.addObject("Autonomous 5", String);
    chooser.addObject("Autonomous 6", String);
    String selected = (String) chooser.getSelected();
    */
     
    
    //sets up the compressor setting/status
    boolean enabled = c.enabled();
    boolean pressureSwitch = c.getPressureSwitchValue();
    double current = c.getCompressorCurrent();

    // sets RobotDrive obj to null so that auto code works
    RobotDrive arcadeDrive = null;
    
    // variable to handle initialization of Robot Drive obj//i.e., on or off?
    boolean initRobotDrive;
    
    // String variable to determine auto mode, "L" = left starting position, "M" = middle starting position, "R" = right starting posistion
    String autoMode;
    
    // various variables to control robot mechanisms and drive train
    double y, x, expY, expX, autoPower, autoTime, leftDoorAngleStart, rightDoorAngleStart, climbSpeed;
    
    /**
     * Constructs Robot object with necessary configurations
     */
    public Robot() {
    	
    	initRobotDrive = true;
        //CALLS AUTO
    	autoMode = "M";
        UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture(0);
        //2 for 2 cameras or does just one line handle them both? 
        UsbCamera cam2 = CameraServer.getInstance().startAutomaticCapture(1);
       
      
        Timer.delay(0.05);
    	climbSpeed = 0;
    	c.setClosedLoopControl(true);
    	
	    JoystickButton pneumaticOpen= new JoystickButton(stick, 9);
	    JoystickButton pneumaticClose= new JoystickButton(stick, 10);
	    pneumaticOpen.whenPressed(new InstantCommand(){
			protected void execute() {
				System.out.println("pneumatic open");
		    	sol1.set(DoubleSolenoid.Value.kForward);
		    	sol1.set(DoubleSolenoid.Value.kOff);
			}
	    });
	    pneumaticClose.whenPressed(new InstantCommand(){
			protected void execute() {
				System.out.println("pneumatic close");
		    	sol1.set(DoubleSolenoid.Value.kReverse);
		    	sol1.set(DoubleSolenoid.Value.kOff);
			}
	    });
	    
    }
    
    
    /**
     * Method for robot to cross baseline in auto, causes robot to drive approximate 130 inches (unsure of precise measurement)
     */
    public void crossBaseLine() {
    	autoPower = .1;
		autoTime = 60;
		bLeft.set(autoPower);
		fLeft.set(autoPower);
		bRight.set(-autoPower);
		fRight.set(-autoPower);
		Timer.delay(autoTime);
    }
    
    public void drive(double speed, double time, boolean isReverse) {
    	
    	if(speedReduced){
    		speed = 0.5*speed;
    	}
		speed = isReverse ? -speed : speed;
		bLeft.set(speed);
		fLeft.set(speed);
		bRight.set(-speed);
		fRight.set(-speed);
		Timer.delay(time);
    }
    
    
    /**
     * Method for robot to do a 180 degree turn in auto
     */
    public void turnAround() {
    	autoPower = 0.4;
		autoTime = 0.7;
		bLeft.set(autoPower);
		fLeft.set(autoPower);
		bRight.set(autoPower);
		fRight.set(autoPower);
		Timer.delay(autoTime);
    }
    
    /**
     * Method for robot to turn left in auto
     */
    public void turnLeft() {
    	autoPower = 0.4;
		autoTime = 0.35;
		bLeft.set(autoPower);
		fLeft.set(autoPower);
		bRight.set(autoPower);
		fRight.set(autoPower);
		Timer.delay(autoTime);
    }
    
    /**
     * Method for robot to turn right in auto
     */
    public void turnRight() {
    	autoPower = 0.4;
		autoTime = 0.35;
		bLeft.set(-autoPower);
		fLeft.set(-autoPower);
		bRight.set(-autoPower);
		fRight.set(-autoPower);
		Timer.delay(autoTime);
    }
    
    
    
    /**
     * AUTONOMOUS
     */
    public void autonomous() {
    	switch(autoMode) {
    		case "L":
    			this.crossBaseLine();
    			this.stopMotors();
    			break;
    		case "M":
    			this.crossBaseLine();
    			this.stopMotors();
    			break;
    		/**
    		 * Case for robot to cross baseline in auto (maybe not tested thoroughly), causes from to drive approximately 80-85 inches (unsure of precise measurement)
    		 */
    	
    		case "M2":
    			this.drive(0.4, 0.95, false);
    			this.drive(0.2, 0.91, false);
    			this.stopMotors();
    			Timer.delay(1.0);
//    			this.openGearMechDoor();
    			this.drive(0.1, 1.0, true);
    			this.stopMotors();
//    			this.closeGearMechDoor();
    			break;
    		//**
    		 // Method for robot to cross baseline in auto, causes robot to drive approximate 130 inches (unsure of precise measurement)
    		 //*
    		case "M3":
    			this.drive(0.4, 2.0, false);
    			this.stopMotors();
    			break;
    		case "R":
    			this.crossBaseLine();
    			this.stopMotors();
    			break;
    		default:
    			this.stopMotors();
    			break;
    	
    		
    	}		
    }

    /**
     * Method to set angles on servos so gear mech doors open
     */
/*
  
    public void openGearMechDoor() {
    	// post build season code (might need to be adjusted if we switch servos) 
		leftGearDoorServo.setAngle(0); //might have to reverse this with the one above
		rightGearDoorServo.setAngle(180);
		
		// build season code
//		leftGearDoorServo.setAngle(leftGearDoorServo.getAngle() + 90);
//		rightGearDoorServo.setAngle(rightGearDoorServo.getAngle() + 90);
    }
    
*/
    /**
     * Method to set angles on servos so gear mech doors close
     */
/*
    public void closeGearMechDoor() {
    	// post build season code (might need to be adjusted if we switch servos)
		leftGearDoorServo.setAngle(180); //might have to reverse this with the one below
		rightGearDoorServo.setAngle(20);
		
		// build season code
//		leftGearDoorServo.setAngle(leftGearDoorServo.getAngle() - 90);
//		rightGearDoorServo.setAngle(rightGearDoorServo.getAngle() - 90);
    }
*/
    
    /**
     * TELEOPERATED MODE. Runs the motors with arcade steering.
     */
    public void operatorControl() {
    	// code to initialize RobotDrive in tele op mode
    	
    	
    	
    	if(initRobotDrive || arcadeDrive.equals(null)) {
        	arcadeDrive = new RobotDrive(fLeft, bLeft, fRight, bRight);
        	initRobotDrive = false;
        }
    	// teleop code loop
    	while (isOperatorControl() && isEnabled()) {
    		//programs motor controllers to drive with exponential arcade drive (i.e. real values are dampened by exponentiation to make driving smoother)
    		expY = Math.pow(-stick.getY(), 1);
    		expX = Math.pow(-stick.getX(), 1);
    		arcadeDrive.arcadeDrive(expY, expX);
    		
    		
    		// y = drivestick.getY();
             //x = drivestick.getX();

             expX = 0;
             expY = 0;
             if (Math.abs(x) > DEADZONEX) {
                 expX = x * Math.abs(x);
             } else {
                 expX = 0;
             }
             if (Math.abs(y) > DEADZONEY) {
                 expY = y;
             } else {
                 expY = 0;
             }

    		
    		
    		
    		// retrieves whether our button configurations are pressed down or not
//    		boolean openMechDoor = stick.getRawButton(GEAR_MECH_DOOR_OPEN);
//        	boolean closeMechDoor = stick.getRawButton(GEAR_MECH_DOOR_CLOSE);
        	boolean disableClimbingMechMotor = stick.getRawButton(DISABLE_CLIMBING_MECH);
        	//boolean newboidown = stick.getRawButton(NewBoi);
        	
        	// determines whether the POV hat switch is set to correct state to trigger events for climbing
        	boolean accelerateClimbingMech = stick.getPOV() == 0 ? true : false;
        	boolean decceraateClimbingMech = stick.getPOV() == 4 ? true : false;
        	
//        	// if GEAR_MECH_DOOR_OPEN button is pressed, open gear mech door so gear falls out
//        	if (openMechDoor) {
//        		this.openGearMechDoor();
//    		}
//        	
//        	// if GEAR_MECH_DOOR_CLOSE button is pressed, close gear mech door so gears will fall in and stay in place on gear mech
//    		if (closeMechDoor) {
//    			this.closeGearMechDoor();
//    		}
//    		
    		// if the DISABLE_CLIMBING_MECH button is pressed, stop the climbing mech motor
        	if (disableClimbingMechMotor) {
        		climbingMech.set(0);
        	}
        	
        	// if the POV stick (hat switch) is moved to the forward position, accelerate the climbing mech motor
        	if (accelerateClimbingMech) {
				climbSpeed += 0.01;
        		climbingMech.set(climbSpeed); //don't know what to set this to
			}
//        	
//        	// if the POV stick (hat switch) is moved to the forward position, decelerate the climbing mech motor
//        	if (decceraateClimbingMech) {
//        		climbSpeed -= 0.01;
//        		climbingMech.set(climbSpeed);
//        	}
//        	// boolena for opening the solenoid 
//        	if (newnewboi) {
//        		pneumaticOpen();
//        	}
//        	// boolean for closing the solenoid 
//        	if (newnewboi == false){
//        		pneumaticClose();
//        	}
        	//allows toggle for reduced drive 
        	/*
        	if (stick.getRawButtonReleased(Reduced)){
        		speedReduced = !speedReduced;
        	}
        	//allows toggle for solenoid 
        	if (stick.getRawButtonReleased(NewBoi)){
        		newnewboi = !newnewboi;
        	}
        	*/
    			Scheduler.getInstance().run();
    	    Timer.delay(0.005);		// wait for a motor update time
        }
    }

    
    /**
     * Runs during test mode
     */
    public void test() {
    	System.out.println("test function");
    }
 
    //reference: https://github.com/chopshop-166/frc-2017/blob/master/src/org/usfirst/frc/team166/robot/Robot.java
    
  /*  public void selectauto(){
    	switch(selected){
	    	case "Autonomous 1":
	    		this.auto1;
	    		break;
	    	case "Autonomous 2":
	    		this.auto2;
	    		break;
	    	case "Autonomous 3":
	    		this.auto3;
	    		break;
	    	case "Autonomous 4":
	    		this.auto4;
	    		break;
	    	case "Autonomous 5":
	    		this.auto5;
	    		break;
	    	case "Autonomous 6":
	    		this.auto6;
	    		break;
	    	default:
	    		this.auto1;
	    		break;
    	}
    	}
    */
    
    
    /**
     * Method to stop drive train motors
     */
    public void stopMotors() {
    	bLeft.set(0);
		fLeft.set(0);
		bRight.set(0);
		fRight.set(0);
    }
    
    /**
     * Runs while robot is disabled
     */
    public void disabled()
    {
    	// disables all motors and servos on the robot
    	this.stopMotors();
//    	rightGearDoorServo.setDisabled();
//    	leftGearDoorServo.setDisabled();
    	sol1.set(DoubleSolenoid.Value.kReverse);
    	climbingMech.set(0);
    	System.out.println("I prefer differently abled you ableist");
    }
}
