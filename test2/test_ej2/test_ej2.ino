#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

// ===================== PINES =====================

#define LED_FORWARD   2
#define LED_BACKWARD  4
#define LED_LEFT     16
#define LED_RIGHT    17
#define LED_STOP      5
#define LED_EMERG    18  //prueba de boton

// Botón de parada de emergencia
#define BTN_EMERG    15   

// ===================== PARÁMETROS =====================

#define LINEAR_SPEED   0.20f   // m/s
#define ANGULAR_SPEED  1.00f   // rad/s

#define GESTURE_TIMEOUT_MS  800

// ===================== micro-ROS VARS =====================

rcl_publisher_t publisher_cmd_vel;        // /cmd_vel
rcl_subscription_t subscriber_gesture;    // /gesture_command
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__String gesture_msg;        

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc;}
#define RCLC_UNUSED(x) (void)(x)

// ===================== ESTADO INTERNO =====================

char current_gesture[16] = "stop"; // último gesto recibido
unsigned long last_gesture_ms = 0; // cuándo llegó el último gesto
bool emergency_active = false;     // estado de emergencia

// ===================== FUNCIONES AUXILIARES =====================

void error_loop(){
  while(1){
    delay(100);
  }
}

void set_all_leds_low()
{
  digitalWrite(LED_FORWARD, LOW);
  digitalWrite(LED_BACKWARD, LOW);
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_RIGHT, LOW);
  digitalWrite(LED_STOP, LOW);
  digitalWrite(LED_EMERG, LOW);
}

void set_led_for_gesture(const char *gesture)
{
  set_all_leds_low();

  if (emergency_active)
  {
    digitalWrite(LED_EMERG, HIGH);
    return;
  }

  if (strcmp(gesture, "forward") == 0)
  {
    digitalWrite(LED_FORWARD, HIGH);
  }
  else if (strcmp(gesture, "backward") == 0)
  {
    digitalWrite(LED_BACKWARD, HIGH);
  }
  else if (strcmp(gesture, "left") == 0)
  {
    digitalWrite(LED_LEFT, HIGH);
  }
  else if (strcmp(gesture, "right") == 0)
  {
    digitalWrite(LED_RIGHT, HIGH);
  }
  else if (strcmp(gesture, "stop") == 0)
  {
    digitalWrite(LED_STOP, HIGH);
  }
}

void fill_twist_for_gesture(const char *gesture)
{
  // poner todo en cero primero
  twist_msg.linear.x  = 0.0;
  twist_msg.linear.y  = 0.0;
  twist_msg.linear.z  = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;

  if (emergency_active)
  {
    // emergencia: siempre velocidad cero
    return;
  }

  if (strcmp(gesture, "forward") == 0)
  {
    twist_msg.linear.x = LINEAR_SPEED;
  }
  else if (strcmp(gesture, "backward") == 0)
  {
    twist_msg.linear.x = -LINEAR_SPEED;
  }
  else if (strcmp(gesture, "left") == 0)
  {
    twist_msg.angular.z = ANGULAR_SPEED;
  }
  else if (strcmp(gesture, "right") == 0)
  {
    twist_msg.angular.z = -ANGULAR_SPEED;
  }
  else if (strcmp(gesture, "stop") == 0)
  {
    // ya está todo en cero >:v
}

// ============= SUBSCRIBER CALLBACK (/gesture_command) =============

void gesture_callback(const void * msgin)
{
  // ignorar cualquier gesto, por boton de emergencia
  if (emergency_active) {
    Serial.println("Gesto ignorado: EMERGENCIA ACTIVA");
    return;
  }

  const std_msgs__msg__String * msg =
      (const std_msgs__msg__String *) msgin;

  size_t len = msg->data.size;
  if (len > 15) len = 15;
  memcpy(current_gesture, msg->data.data, len);
  current_gesture[len] = '\0';

  last_gesture_ms = millis();  // actualizamos tiempo de último gesto

  Serial.print("Gesto recibido: ");
  Serial.println(current_gesture);

}

// ============= TIMER CALLBACK =============

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  // botón de emergencia
  int btn_state = digitalRead(BTN_EMERG);

  if (btn_state == LOW)
  {
    if (!emergency_active)
    {
      Serial.println(">>> EMERGENCIA ACTIVADA");
    }
    emergency_active = true;
    strcpy(current_gesture, "");    // sin gesto
    set_all_leds_low();
    digitalWrite(LED_EMERG, HIGH);  

    twist_msg.linear.x  = 0.0;
    twist_msg.linear.y  = 0.0;
    twist_msg.linear.z  = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
    RCSOFTCHECK(rcl_publish(&publisher_cmd_vel, &twist_msg, NULL));

    last_gesture_ms = millis(); 
    return;
  }
  else
  {
    if (emergency_active)
    {
      Serial.println(">>> EMERGENCIA DESACTIVADA");
      emergency_active = false;
      strcpy(current_gesture, "stop");
      last_gesture_ms = millis();
    }
  }

  // Timeout de gestos
  unsigned long now = millis();
  if ((now - last_gesture_ms) > GESTURE_TIMEOUT_MS)
  {
    if (strcmp(current_gesture, "stop") != 0)
    {
      Serial.println(">>> TIMEOUT, forzando STOP");
    }
    strcpy(current_gesture, "stop");
    last_gesture_ms = now;
  }

  // publicar /cmd_vel
  set_led_for_gesture(current_gesture);
  fill_twist_for_gesture(current_gesture);
  RCSOFTCHECK(rcl_publish(&publisher_cmd_vel, &twist_msg, NULL));
}

// ===================== SETUP =====================

void setup()
{
  Serial.begin(115200);

  pinMode(LED_FORWARD,  OUTPUT);
  pinMode(LED_BACKWARD, OUTPUT);
  pinMode(LED_LEFT,     OUTPUT);
  pinMode(LED_RIGHT,    OUTPUT);
  pinMode(LED_STOP,     OUTPUT);
  pinMode(LED_EMERG,    OUTPUT);

  pinMode(BTN_EMERG,    INPUT_PULLUP);

  set_all_leds_low();

  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Crear soporte
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Crear nodo
  RCCHECK(rclc_node_init_default(
    &node,
    "esp32_gesture_controller",
    "",
    &support));

  // Publisher /cmd_vel
  RCCHECK(rclc_publisher_init_default(
    &publisher_cmd_vel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Subscriber /gesture_command
  RCCHECK(rclc_subscription_init_default(
    &subscriber_gesture,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "gesture_command"));

  // Timer (cada 100 ms)
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber_gesture,
    &gesture_msg,
    &gesture_callback,
    ON_NEW_DATA));

  gesture_msg.data.data = (char *) malloc(50 * sizeof(char));
  gesture_msg.data.size = 0;
  gesture_msg.data.capacity = 50;

  twist_msg.linear.x  = 0.0;
  twist_msg.linear.y  = 0.0;
  twist_msg.linear.z  = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;

  last_gesture_ms = millis();

  Serial.println("micro-ROS gesture node inicializado!");
}

// ===================== LOOP =====================

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}
