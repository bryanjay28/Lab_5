package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S3");

	// Single navigation instance used by all classes
	private static Navigation navigation;

	// Set vehicle constants
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 11.25;
	private static int startingCorner = 0;

	// Constants for part 2
	private static double lowerLeftX = 0 * USLocalizer.TILESIZE;
	private static double lowerLeftY = 0 * USLocalizer.TILESIZE;
	private static double upperRightX = 0 * USLocalizer.TILESIZE;
	private static double upperRightY = 0 * USLocalizer.TILESIZE;
	private static int targetBlock = 0;


	@SuppressWarnings("deprecation")
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		int buttonChoice = 0;

		do {
			// clear the display
			lcd.clear();

			//
			lcd.drawString("Press any button", 0, 0);
			lcd.drawString("to start color", 0, 1);
			lcd.drawString("classification", 0, 2);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice == 0);

		lcd.clear();

		ColourCalibration colourCalibration = new ColourCalibration();
		Thread colourCalibrationThread = new Thread(colourCalibration);
		colourCalibrationThread.start();

		// Keep the colour detection running until a button is pressed to start part2
		buttonChoice = 0;
		while (buttonChoice == 0) {
			buttonChoice = Button.waitForAnyPress();
		}
			colourCalibrationThread.interrupt();
			

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd); // No need to change

		// usSensor is the instance
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		// usDistance provides samples from this instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");

		navigation = new Navigation(odometer, leftMotor, rightMotor);

		// Start odometer and display threads and correction Threads.
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Create ultrasonic and light localizer objects.
		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, usDistance, startingCorner,
				navigation);
		navigation.usLoc = USLocalizer;
		LightLocalizer lightLocatizer = new LightLocalizer(odometer, leftMotor, rightMotor, navigation);

		// perform the ultrasonic localization
		USLocalizer.localize();
		ultrasonicSensor = null;
		// perform the light sensor localization
		lightLocatizer.localize(1.0,1.0,0.0);
		
		//
		//		// Modified just before executing and loading the code on the machine
		//		// Replace the 0 by the number of tiles representing the position
		//		lowerLeftX = 0 * USLocalizer.TILESIZE;
		//		lowerLeftY = 0* USLocalizer.TILESIZE;
		//		upperRightX = 0* USLocalizer.TILESIZE;
		//		upperRightY = 0 * USLocalizer.TILESIZE;
		//		targetBlock = 0;
		//
		//		// Recreating the thread because its behaviour will be different
		//		// It will check for colours upon request instead of continually
		//		colourCalibrationThread = new Thread(colourCalibration);
		//		colourCalibration.isFieldSearching = true;
		//
		//		colourCalibrationThread.start();
		//
		//		SearchAndLocalize searcher = new SearchAndLocalize(lowerLeftX, lowerLeftY, upperRightX, upperRightY,
		//				targetBlock, navigation, colourCalibration);
		//		
		//		colourCalibration.setFlag(targetBlock);
		//
		//		searcher.fieldTest();

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}