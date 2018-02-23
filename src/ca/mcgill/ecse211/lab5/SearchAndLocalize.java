package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;

public class SearchAndLocalize {
	private double lowerLeftX, lowerLeftY;
	private double upperRightX, upperRightY;
	private int targetBlock;
	private Navigation navigation;
	private boolean foundBlock = false;
	private ColourCalibration colourCalib;

	public SearchAndLocalize(double llx, double lly, double urx, double ury, int tb, Navigation nav,
			ColourCalibration cc) {
		this.lowerLeftX = llx;
		this.lowerLeftY = lly;
		this.upperRightX = urx;
		this.upperRightY = ury;
		this.targetBlock = tb;

		this.navigation = nav;

		this.colourCalib = cc;

	}

	public void fieldTest() {
		// Travel to the lower-left corner
		this.navigation.travelTo(this.lowerLeftX, this.lowerLeftY, false, null);
		Sound.beep();

		/*
		 * currentTargetX and currentTargetY represent the successive targets that the
		 * robot travels to in order to go through the entire search area Y is added
		 * 30.48 cm (i.e. one tile) at every iteration, and the X value switches from
		 * one side to the other of the search area
		 */
		double currentTargetX = this.lowerLeftX, currentTargetY = this.lowerLeftY;
		while (targetsArentUpperRight(currentTargetX, currentTargetY) && !foundBlock) {
			currentTargetY += 30.48;
			currentTargetX = switchXValue(currentTargetX);
			this.navigation.travelTo(currentTargetX, currentTargetY, true, null);
			if (foundBlock) {
				// Block was found, stop covering the entire search area and go to UR
				break;
			}
		}
		this.navigation.travelTo(this.upperRightX, this.upperRightY, false, this);
	}

	private boolean targetsArentUpperRight(double currX, double currY) {
		return currX == upperRightX && currY == upperRightY;
	}

	private double switchXValue(double x) {
		// Returns llx if x == urx, returns urx if x == llx
		return this.lowerLeftX + this.upperRightX - x;
	}
	
	public void setFoundBlock(boolean newVal) {
		this.foundBlock = newVal;
	}
}
