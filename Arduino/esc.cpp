// kompatibilitaetsimport fuer diverse Boards
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#define USB_USBCON
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

// Erstellen einer RosNode
ros::NodeHandle  nodeHandle;
// Empfohlene Einstellungen fuer den 
// TRAXXAS Electronic Speed Controlle
const int minimum_steering = 30;
const int maximum_steering = 150;
const int minimum_throttle = 0;
const int maximum_throttle = 150;
const int esc_pin = 10;
const int steer_pin = 9;
int escAngle = 90;

// Lenkungsservo
Servo steering_servo;
// Geschwindigkeitsservo
Servo esc_servo;  
// Message fuer den Debugger
std_msgs::Int32 str_msg;
ros::Publisher esc_chatter("esc", &str_msg); 

/** 
 * @brief  Map Methode um die empfangenen Daten in den Wertebereich des Servo zu konvertieren
 * 
 * @param  toMap:   ungemappter Wert
 * @param  in_min:  min. Input
 * @param  in_max:  max. Input
 * @param  out_min: min. Output
 * @param  out_max: max. Output
 * @return gemappten Wert
 */
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/** 
 * @brief   Callback, welcher ausgeloest wird, wenn eine nachricht mit einem Twist empfangen wird.
 *          Twists sind eine ROS-Klasse, welche mit Hilfe von 2 3-Dimensionalen Vektoren einer Bewegung abbildet.
 * @param   twistData: Message vom Typ  geometry/Twist
 * @retval  None
 */
void driveCallback ( const geometry_msgs::Twist&  twistData )
{
  // Lenkwinkel auf vorgegebenen WErtebereich mappen
  int steeringAngle = fmap(twistData.angular.z, 0.0, 1.0,minimum_steering,maximum_steering);
  // Debugging:
  // str_msg.data= steeringAngle;
  // esc_chatter.publish(&str_msg);
  // Nochmal Winkel pruefen, bevor der Servo angesteuert wird
  if (steeringAngle <minimum_steering) { 
    steeringAngle =minimum_steering;
  }
  if (steeringAngle >maximum_steering) {
    steeringAngle =maximum_steering;
  }
  // Lenkung ausrichten
  steering_servo.write(steeringAngle);
  // X aus Linearvektor des Twist auslesen
  if (twistData.linear.x >= 0.5) {
    escAngle = (int)fmap(twistData.linear.x, 0.5, 1.0, 90.0, maximum_throttle);
  } else {
    escAngle = (int)fmap(twistData.linear.x, 0.0, 1.0, 0.0, 180.0);
  }
  // Check to make sure throttle command is within bounds
  if (escAngle <minimum_throttle) { 
    escAngle =minimum_throttle;
  }
  if (escAngle > maximum_throttle) {
    escAngle = maximum_throttle;
  }
  // Debug 2.0:
  // str_msg.data= escAngle;
  // esc_chatter.publish(&str_msg);
  esc_servo.write(escAngle);
  digitalWrite(13, HIGH-digitalRead(13));  //Built-in LED   
 
}
// Subscriber fuer Lenk- & Beschleunigungsdaten
ros::Subscriber<geometry_msgs::Twist> driveSubscriber("/jetsoncar_teleop_joystick/cmd_vel", &driveCallback);

void setup(){
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  nodeHandle.initNode();
  // Debugging Node anbieten
  nodeHandle.advertise(esc_chatter);
  // Subscriber aktivieren
  nodeHandle.subscribe(driveSubscriber);
  // Attach the servos to actual pins
  steering_servo.attach(steer_pin); // Steering servo is attached to pin 9
  esc_servo.attach(esc_pin); // ESC is on pin 10
  // Initialize Steering and ESC setting
  // Steering centered is 90, throttle at neutral is 90
  steering_servo.write(90);
  esc_servo.write(90);
  delay(1000);
  
}

void loop(){
  nodeHandle.spinOnce();
  // hoehere Delays lassen weniger Signale in der Sekunde zu, aber bricht nach 1ms die Servobewegung ab.
  // ggf hoeheren Wert nutzen
  delay(15);
}
