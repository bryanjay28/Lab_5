package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S3");
	private static Navigation navigation;

	// Set vehicle constants
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 15.79;
	private static int startingCorner = 0;
	
	// Constants for part 2
	private static double lowerLeftX, lowerLeftY;
	private static double upperRightX, upperRightY;
	private static int targetBlock;

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice = 0;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd); // No need to change

		@SuppressWarnings("resource") // Because we don't bother to close this resource
		// usSensor is the instance
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		// usDistance provides samples from this instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");
		
		navigation = new Navigation(odometer, leftMotor, rightMotor);

		do {
			// clear the display
			lcd.clear();

			//
			lcd.drawString("Press any button to start ", 0, 0);
			lcd.drawString("color classification ", 1, 0);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice == 0);

		lcd.clear();

		// TODO: gotta fix the colour calibration to work independently for the colour
		// test

		ColourCalibration colourCalibration = new ColourCalibration();

		Thread colourCalibrationThread = new Thread(colourCalibration);
		colourCalibration.start();
		
		buttonChoice = 0;
		while (buttonChoice == 0) {
			buttonChoice = Button.waitForAnyPress();
		}

		// Start odometer and display threads and correction Threads.
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Create ultrasonic and light localizer objects.
		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, usDistance, startingCorner, navigation);
		LightLocalizer lightLocatizer = new LightLocalizer(odometer, leftMotor, rightMotor, navigation);
		
		// perform the ultrasonic localization
		USLocalizer.localize();

		// perform the light sensor localization
		lightLocatizer.localize();
		
		lowerLeftX = 0 * USLocalizer.getTileSize();
		lowerLeftY = 0 * USLocalizer.getTileSize();
		upperRightX = 0 * USLocalizer.getTileSize();
		upperRightY = 0 * USLocalizer.getTileSize();
		targetBlock = 1;

		SearchAndLocalize searcher = new SearchAndLocalize(lowerLeftX, lowerLeftY, upperRightX, upperRightY, targetBlock, navigation, colourCalibration);

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}