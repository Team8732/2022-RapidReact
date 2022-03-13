package frc.team8732.robot.controller;

/**
 * Contains functions for use with the Logitech Controller.
 * @author Joshua Lewis joshlew@trinityforce.org
 */
public class Logitech extends ButtonMap {
//Logitech Button Mapping
public Logitech() {

    this.AXIS_LEFT_X = 0;
    this.AXIS_LEFT_Y = 1;
    this.AXIS_RIGHT_X= 2;
    this.AXIS_RIGHT_Y= 3;
    
    this.BUTTON_X = 3;
    this.BUTTON_A = 1;
    this.BUTTON_B = 2;
    this.BUTTON_Y = 4;
    this.BUTTON_LEFT_BUMPER = 5;	
    this.BUTTON_RIGHT_BUMPER = 6;
    this.BUTTON_SHARE = 7;          //Back Button        
    this.BUTTON_OPTIONS = 8;       //Start Button    
    this.BUTTON_LEFT_JOYSTICK = 9;
    this.BUTTON_RIGHT_JOYSTICK = 10;
       
    }
}