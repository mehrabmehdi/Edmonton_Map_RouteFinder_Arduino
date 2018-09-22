/*
Mehrab Mehdi Islam
Student Id: 1545664

Alexandar Chan
Student Id: 1544806

*/
#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#include <SD.h>
#include "consts_and_types.h"
#include "map_drawing.h"

// the variables to be shared across the project, they are declared here!
shared_vars shared;

Adafruit_ILI9341 tft = Adafruit_ILI9341(clientpins::tft_cs, clientpins::tft_dc);

void setup() {
  // initialize Arduino
  init();

  // initialize zoom pins
  pinMode(clientpins::zoom_in_pin, INPUT_PULLUP);
  pinMode(clientpins::zoom_out_pin, INPUT_PULLUP);

  // initialize joystick pins and calibrate centre reading
  pinMode(clientpins::joy_button_pin, INPUT_PULLUP);
  // x and y are reverse because of how our joystick is oriented
  shared.joy_centre = xy_pos(analogRead(clientpins::joy_y_pin), analogRead(clientpins::joy_x_pin));

  // initialize serial port
  Serial.begin(9600);
  Serial.flush(); // get rid of any leftover bits

  // initially no path is stored
  shared.num_waypoints = 0;

  // initialize display
  shared.tft = &tft;
  shared.tft->begin();
  shared.tft->setRotation(3);
  shared.tft->fillScreen(ILI9341_BLUE); // so we know the map redraws properly

  // initialize SD card
  if (!SD.begin(clientpins::sd_cs)) {
      Serial.println("Initialization has failed. Things to check:");
      Serial.println("* Is a card inserted properly?");
      Serial.println("* Is your wiring correct?");
      Serial.println("* Is the chipSelect pin the one for your shield or module?");

      while (1) {} // nothing to do here, fix the card issue and retry
  }

  // initialize the shared variables, from map_drawing.h
  // doesn't actually draw anything, just initializes values
  initialize_display_values();

  // initial draw of the map, from map_drawing.h
  draw_map();
  draw_cursor();

  // initial status message
  status_message("FROM?");
}

void process_input() {
  // read the zoom in and out buttons
  shared.zoom_in_pushed = (digitalRead(clientpins::zoom_in_pin) == LOW);
  shared.zoom_out_pushed = (digitalRead(clientpins::zoom_out_pin) == LOW);

  // read the joystick button
  shared.joy_button_pushed = (digitalRead(clientpins::joy_button_pin) == LOW);

  // joystick speed, higher is faster
  const int16_t step = 64;

  // get the joystick movement, dividing by step discretizes it
  // currently a far joystick push will move the cursor about 5 pixels
  xy_pos delta(
    // the funny x/y swap is because of our joystick orientation
    (analogRead(clientpins::joy_y_pin)-shared.joy_centre.x)/step,
    (analogRead(clientpins::joy_x_pin)-shared.joy_centre.y)/step
  );
  delta.x = -delta.x; // horizontal axis is reversed in our orientation

  // check if there was enough movement to move the cursor
  if (delta.x != 0 || delta.y != 0) {
    // if we are here, there was noticeable movement

    // the next three functions are in map_drawing.h
    erase_cursor();       // erase the current cursor
    move_cursor(delta);   // move the cursor, and the map view if the edge was nudged
    if (shared.redraw_map == 0) {
      // it looks funny if we redraw the cursor before the map scrolls
      draw_cursor();      // draw the new cursor position
    }
  }
}


void client_to_Server(lon_lat_32 start, lon_lat_32 end){
  //use a finite state machine to communicate between  the server and client
  enum State {Send_Byte, Waiting_N, Waiting_W, End_com};
  //initial state Send_Byte
  State curr_state = Send_Byte;
  Serial.println("Starting");
  // declare initial variables
  bool check = false;
  int32_t N_path = 0;
  int path = 0;
  int32_t W_lat_long = 0;
  int count = 1;
  int starting_timeout;
  int ending_timeout;
  char incomingByte;

  while (true) {

    while (Serial.available() == 0);
// if the state is ending end serial communication
    if (curr_state == End_com){
      Serial.flush();
      break;
    }
// if the state is Waiting_W
    if (curr_state == Waiting_W){
      while(true){
// read the incomingByte from the serial connection
        incomingByte = Serial.read();
// if the incoming byte is next line
        if (incomingByte == '\n') {
          //if check is true
          if (check){
//store the lon
            shared.waypoints[path].lon = -1*W_lat_long;
            // add 1 more point to the path
            path += 1;
            //if the path is equal to the N_path
              if (path == N_path) {
                //change state to End_com
                curr_state = End_com;
                // set both path values to 0
                N_path = 0;
                path = 0;
                // set check to false
                check = false;
            }
            // send A through the serial to the server
            Serial.write('A');
            // set check to false and count to 1 nd the lar_long to 0
            check = false;
            count = 1;
            W_lat_long = 0;

          }
          break;
        }
// ignre the following incomingBytes and continue on
        if (incomingByte == -1) continue;

        if (incomingByte == '%') continue;

        if (incomingByte == '-') continue;
        // if the incomingByte is W , set check to true
        if (incomingByte == 'W') {
          check = true;
          continue;
        }
        // if it is a space and the count is 1, set it to 0
        if ((incomingByte == ' ') && (count == 1)) {
          count = 0;
          continue;
        }
          // if it is a space and the count is 0, set it to 2, and store the latitude point
        if ((incomingByte == ' ') && (count == 0) && (check)) {

          count = 2;
          shared.waypoints[path].lat = W_lat_long;
          W_lat_long = 0;
          continue;
        }
        //shifts the numeric values over and adds the new value
        W_lat_long *= 10;
        W_lat_long += (incomingByte - 48);
      }
    }
//if state is waiting for N
    if (curr_state == Waiting_N){
      // N_path is set to initial value 0
      N_path = 0;
      while (true){
        //while true, read incoming byte from serial
        incomingByte = Serial.read();
        //if incoming Byte is next line
        if (incomingByte == '\n') {
          //if check is true
          if (check){
            //if path_N is equal to zero
            if (N_path == 0){
              //current state is changed to end_com
              curr_state = End_com;
              N_path = 0;
              //check is set to false and the loop broken
              check = false;
              break;

            }
            else
            {
              //else store the N_path as the number of waypoints
              shared.num_waypoints = N_path;
              curr_state = Waiting_W;
              if (N_path > 500) {
                // if the number of N_paths greater than 500 end_com as there are too many paths
                curr_state = End_com;
              }
              // send over acceptence
              Serial.write('A');
              check = false;
            }
          }
          break;
        }
        // continue if the following incomingByte comes up
        if (incomingByte == -1) continue;

        if (incomingByte == '%') continue;

        if (incomingByte == 'N') {
          //if the incoming byte is N then check is true
          check = true;
          continue;

        }

        if (incomingByte == ' ') continue;
        // shift the path and add the new one
        N_path *= 10;
        N_path += (incomingByte - 48);
        // calculate ending_timeout
        ending_timeout = millis() - starting_timeout;
        if (ending_timeout > 10000) {
          //if time out greater than 10secs, change current state to end com
          curr_state = End_com;
        }
      }
    }

    if (curr_state == Send_Byte){
      // we send the start and end latitude
      Serial.print("R ");
      Serial.print(start.lat);
      Serial.print(" ");
      Serial.print(start.lon);
      Serial.print(" ");
      Serial.print(end.lat);
      Serial.print(" ");
      Serial.println(end.lon);
      Serial.flush();
      // change current state to wating for N
      // take starting_timeout
      curr_state = Waiting_N;
      starting_timeout = millis();
    }
  }
}

void draw_route(lon_lat_32 start, lon_lat_32 end)
{
  // draw line connecting from start to the first waypoint
     int32_t starty = latitude_to_y(shared.map_number,start.lat)-shared.map_coords.y;
     int32_t startx = longitude_to_x(shared.map_number,start.lon)-shared.map_coords.x;
     int32_t endy = latitude_to_y(shared.map_number,shared.waypoints[0].lat)-shared.map_coords.y;
     int32_t endx = longitude_to_x(shared.map_number,shared.waypoints[0].lon)-shared.map_coords.x;

     if(((shared.map_coords.x <= (startx+shared.map_coords.x))&&((startx+shared.map_coords.x) <= (shared.map_coords.x+320)))
     && ((shared.map_coords.y <= (starty+shared.map_coords.y))&&((starty+shared.map_coords.y)<=(shared.map_coords.y+216)))
     && ((shared.map_coords.x <= (endx+shared.map_coords.x))&&((endx+shared.map_coords.x) <= (shared.map_coords.x+320)))
     && ((shared.map_coords.y <= (endy+shared.map_coords.y))&&((endy+shared.map_coords.y)<=(shared.map_coords.y+216)))){

       shared.tft-> drawLine(longitude_to_x(shared.map_number, start.lon)-shared.map_coords.x,
       latitude_to_y(shared.map_number,start.lat)-shared.map_coords.y,
       longitude_to_x(shared.map_number,shared.waypoints[0].lon)-shared.map_coords.x,
       latitude_to_y(shared.map_number, shared.waypoints[0].lat)-shared.map_coords.y,ILI9341_BLUE);
     }
     // draw line connecting the way points in the middle to make up the route
     for(int k = 0; k < (shared.num_waypoints-1);k++){

       int32_t start_waypointsy = latitude_to_y(shared.map_number,shared.waypoints[k].lat)-shared.map_coords.y;
       int32_t start_waypointsx = longitude_to_x(shared.map_number,shared.waypoints[k].lon)-shared.map_coords.x;
       int32_t end_waypointsy = latitude_to_y(shared.map_number,shared.waypoints[k+1].lat)-shared.map_coords.y;
       int32_t end_waypointsx = longitude_to_x(shared.map_number,shared.waypoints[k+1].lon)-shared.map_coords.x;


       if(((shared.map_coords.x <= (start_waypointsx+shared.map_coords.x))&&((start_waypointsx+shared.map_coords.x) <= (shared.map_coords.x+320)))
       && ((shared.map_coords.y <= (start_waypointsy+shared.map_coords.y))&&((start_waypointsy+shared.map_coords.y)<=(shared.map_coords.y+216)))
       && ((shared.map_coords.x <= (end_waypointsx+shared.map_coords.x))&&((end_waypointsx+shared.map_coords.x) <= (shared.map_coords.x+320)))
       && ((shared.map_coords.y <= (end_waypointsy+shared.map_coords.y))&&((end_waypointsy+shared.map_coords.y)<=(shared.map_coords.y+216)))){

         shared.tft-> drawLine(start_waypointsx,start_waypointsy,end_waypointsx,end_waypointsy,ILI9341_BLUE);
       }
     }
     // draw line connecting the the route to the end point

     starty = latitude_to_y(shared.map_number,shared.waypoints[shared.num_waypoints-1].lat)-shared.map_coords.y;
     startx = longitude_to_x(shared.map_number,shared.waypoints[shared.num_waypoints-1].lon)-shared.map_coords.x;
     endy = latitude_to_y(shared.map_number,end.lat)-shared.map_coords.y;
     endx = longitude_to_x(shared.map_number,end.lon)-shared.map_coords.x;

     if(((shared.map_coords.x <= (startx+shared.map_coords.x))&&((startx+shared.map_coords.x) <= (shared.map_coords.x+320)))
     && ((shared.map_coords.y <= (starty+shared.map_coords.y))&&((starty+shared.map_coords.y)<=(shared.map_coords.y+216)))
     && ((shared.map_coords.x <= (endx+shared.map_coords.x))&&((endx+shared.map_coords.x) <= (shared.map_coords.x+320)))
     && ((shared.map_coords.y <= (endy+shared.map_coords.y))&&((endy+shared.map_coords.y)<=(shared.map_coords.y+216)))){

       shared.tft-> drawLine(startx,starty,endx,endy,ILI9341_BLUE);
     }

}

int main() {
  setup();

  // very simple finite state machine:
  // which endpoint are we waiting for?
  enum {WAIT_FOR_START, WAIT_FOR_STOP} curr_mode = WAIT_FOR_START;

  // the two points that are clicked
  lon_lat_32 start, end;

  while (true) {
    // clear entries for new state
    shared.zoom_in_pushed = 0;
    shared.zoom_out_pushed = 0;
    shared.joy_button_pushed = 0;
    shared.redraw_map = 0;

    // reads the three buttons and joystick movement
    // updates the cursor view, map display, and sets the
    // shared.redraw_map flag to 1 if we have to redraw the whole map
    // NOTE: this only updates the internal values representing
    // the cursor and map view, the redrawing occurs at the end of this loop
    process_input();

    // if a zoom button was pushed, update the map and cursor view values
    // for that button push (still need to redraw at the end of this loop)
    // function zoom_map() is from map_drawing.h
    if (shared.zoom_in_pushed) {
      zoom_map(1);
      shared.redraw_map = 1;
    }
    else if (shared.zoom_out_pushed) {
      zoom_map(-1);
      shared.redraw_map = 1;
    }

    // if the joystick button was clicked
    if (shared.joy_button_pushed) {

      if (curr_mode == WAIT_FOR_START) {
        // if we were waiting for the start point, record it
        // and indicate we are waiting for the end point
        start = get_cursor_lonlat();
        curr_mode = WAIT_FOR_STOP;
        status_message("TO?");

        // wait until the joystick button is no longer pushed
        while (digitalRead(clientpins::joy_button_pin) == LOW) {}
      }
      else {
        // if we were waiting for the end point, record it
        // and then communicate with the server to get the path
        end = get_cursor_lonlat();

        // TODO: communicate with the server to get the waypoints
         status_message("Recieving Waypoints...");

         client_to_Server(start, end);
        // now we have stored the path length in
        // shared.num_waypoints and the waypoints themselves in
        // the shared.waypoints[] array, switch back to asking for the
        // start point of a new request
        draw_map();
        draw_cursor();
        draw_route(start,end);
        curr_mode = WAIT_FOR_START;
        status_message("FROM?");

        // wait until the joystick button is no longer pushed
        while (digitalRead(clientpins::joy_button_pin) == LOW) {}
      }
    }

    if (shared.redraw_map) {
      // redraw the status message
      if (curr_mode == WAIT_FOR_START) {
        status_message("FROM?");
      }
      else {
        status_message("TO?");
      }

      // redraw the map and cursor
      draw_map();
      draw_cursor();
      // TODO: draw the route if there is one
      draw_route(start,end);
    }
  }

  Serial.flush();
  return 0;
}
