// File:          NMP_Controller.c
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.h>, <webots/Motor.h>, etc.
// and/or to add some other includes
#include <webots/robot.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>


// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  wb_robot_init();
  
   // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  
  //lidar *lidar = robot->getlidar("lidar");
  
  WbDeviceTag lmotor = wb_robot_get_device("left wheel motor");
  WbDeviceTag rmotor = wb_robot_get_device("right wheel motor");

  //Motor *lmotor = robot->getMotor("left wheel motor");
  //Motor *rmotor = robot->getMotor("right wheel motor");
  
  WbDeviceTag frontLeftDs = wb_robot_get_device("ds2");
  WbDeviceTag frontRightDs = wb_robot_get_device("ds1");
  
  //DistanceSensor *frontLeftDs = robot->getDistanceSensor("ds2");
  //DistanceSensor *frontRightDs = robot->getDistanceSensor("ds1");
  
  WbDeviceTag leftDs = wb_robot_get_device("ds3");
  WbDeviceTag rightDs = wb_robot_get_device("ds0");
  
  //DistanceSensor *leftDs = robot->getDistanceSensor("ds3");
  //DistanceSensor *rightDs = robot->getDistanceSensor("ds0");
  
  WbDeviceTag leftFrontLeftDs = wb_robot_get_device("ds5");
  WbDeviceTag rightFrontRightDs = wb_robot_get_device("ds4");
  
  //DistanceSensor *leftFrontLeftDs = robot->getDistanceSensor("ds5");
  //DistanceSensor *rightFrontRightDs= robot->getDistanceSensor("ds4");
  
  WbDeviceTag cam = wb_robot_get_device("camera");
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  
  //Camera * cam = robot->getCamera("camera");
  //Accelerometer * accelerometer = robot->getAccelerometer("accelerometer");
  
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  
  //Gyro *gyro = robot->getGyro("gyro");

  // get the time step of the current world.
  int timeStep = (int)wb_robot_get_basic_time_step();


  // Enable the sensors, feel free to change the sampling rate
  wb_lidar_enable(lidar, 50);
  //lidar->enable(50);
  
  wb_distance_sensor_enable(frontLeftDs, 100);
  //frontLeftDs->enable(100);
  
  wb_distance_sensor_enable(frontRightDs, 100);
  //frontRightDs->enable(100);
  
  wb_distance_sensor_enable(leftDs, 100);
  //leftDs->enable(100);
  
  wb_distance_sensor_enable(rightDs, 100);
  //rightDs->enable(100);
  
  wb_distance_sensor_enable(leftFrontLeftDs, 100);
  //leftFrontLeftDs->enable(100);
  
  wb_distance_sensor_enable(rightFrontRightDs, 100);
  //rightFrontRightDs->enable(100);
  
  wb_accelerometer_enable(accelerometer, 100);
  //accelerometer->enable(100);
  
  wb_gyro_enable(gyro, 100);
  //gyro->enable(100);
  
  wb_camera_enable(cam, 50);
  //cam->enable(50);
  
  wb_motor_set_position(lmotor, INFINITY);
  //lmotor->setPosition(INFINITY);
  
  wb_motor_set_position(rmotor, INFINITY);
  //rmotor->setPosition(INFINITY);
  
  wb_motor_set_velocity(lmotor, 0);
  //lmotor->setVelocity(0);
  
  wb_motor_set_velocity(rmotor, 0);
  //rmotor->setVelocity(0);
  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (wb_robot_step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    wb_motor_set_velocity(lmotor, 10);
    //lmotor->setVelocity(10);
    wb_motor_set_velocity(rmotor, 10);
    //rmotor->setVelocity(10);
     // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  wb_robot_cleanup();
  return 0;
}
