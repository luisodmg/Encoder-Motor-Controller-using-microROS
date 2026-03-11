// Este es el codigo chido

#include <micro_ros_arduino.h>
#include <WiFi.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include "esp_system.h"

// ------------ WIFI ------------
char ssid[] = "ElPanelas";
char password[] = "luispapa1";
char agent_ip[] = "192.168.1.112";
uint16_t agent_port = 8888;

// ------------ MOTOR PINS ------------
#define IN1 19
#define IN2 18
#define ENA 16

// ------------ ENCODER ------------
#define ENC_A 32
#define ENC_B 33

volatile long pulse_count = 0;

// ------------ PWM ------------
const int pwmChannel = 0;
const int pwmFreq = 1000;
const int pwmRes = 8;

// ------------ MOTOR PARAMS ------------
const float PPR = 172.0;
const float Ts = 0.01;

const float OMEGA_MAX = 9.15;

// ------------ STATE ------------
float omega = 0;
float omega_filt = 0;
float setpoint = 0;

// ------------ PID ------------
float Kp = 3.0;
float Ki = 1.0;
float Kd = 0.02;

float error_prev = 0;
float integral = 0;

float max_integral = 50;

// ------------ MOTOR MODEL (feedforward) ------------
float ff_gain = 5.5;
float ff_offset = 60;

// ------------ LIMITS ------------
int pwm_dead = 70;
int pwm_max = 255;

// ------------ microROS ------------
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_timer_t timer;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

std_msgs__msg__Float32 msg_in;
std_msgs__msg__Float32 msg_out;


// ------------ ENCODER ISR (QUADRATURE) ------------
void IRAM_ATTR encoder_isr()
{
  bool b = digitalRead(ENC_B);

  if (b)
    pulse_count++;
  else
    pulse_count--;
}


// ------------ SETPOINT CALLBACK ------------
void setpoint_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg =
    (const std_msgs__msg__Float32 *)msgin;

  // el input debe venir en [-1 , 1]
  float sp_norm = constrain(msg->data, -1.0, 1.0);

  // convertir a velocidad real
  setpoint = sp_norm * OMEGA_MAX;

  Serial.print("New SP norm: ");
  Serial.print(sp_norm);
  Serial.print("  | rad/s: ");
  Serial.println(setpoint);
}


// ------------ CONTROL LOOP ------------
void control_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  if (timer == NULL) return;

  // ----- read encoder -----
  portDISABLE_INTERRUPTS();
  long pulses = pulse_count;
  pulse_count = 0;
  portENABLE_INTERRUPTS();

  omega = (pulses * 2.0 * PI) / (PPR * Ts);

  if (abs(omega) > 25)
  {
    omega = omega_filt;
  }

  omega_filt = 0.9 * omega_filt + 0.1 * omega;
  
  // limitar rango
  //omega_filt = constrain(omega_filt, -OMEGA_MAX, OMEGA_MAX);

  // ----- STOP CONDITION -----
  if (abs(setpoint) < 0.01)
  {
    ledcWrite(pwmChannel, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    integral = 0;
    error_prev = 0;

    msg_out.data = omega_filt;
    rcl_publish(&publisher, &msg_out, NULL);

    Serial.print("SP:0 | W:");
    Serial.print(omega_filt);
    Serial.println(" | MOTOR STOP");

    return;
  }

  // ----- PID -----
  float error = setpoint - omega_filt;

  integral += error * Ts;
  integral = constrain(integral, -max_integral, max_integral);

  float derivative = (error - error_prev) / Ts;

  float pid =
    Kp * error +
    Ki * integral +
    Kd * derivative;

  error_prev = error;

  // ----- FEEDFORWARD -----
  float pwm_ff = ff_gain * setpoint + ff_offset;

  // ----- TOTAL CONTROL -----
  float pwm_total = pwm_ff + pid;

  int duty = abs(pwm_total);

  if (duty > pwm_max)
    duty = pwm_max;

  if (duty > 0 && duty < pwm_dead)
    duty = pwm_dead;

  // ----- direction -----
  if (pwm_total > 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  ledcWrite(pwmChannel, duty);

  // publicar velocidad real
  msg_out.data = omega_filt;
  rcl_publish(&publisher, &msg_out, NULL);

  // debug
  Serial.print("SP:");
  Serial.print(setpoint);

  Serial.print(" | W:");
  Serial.print(omega_filt);

  Serial.print(" | PWM:");
  Serial.println(duty);
}

void debug_wifi()
{
  Serial.println("----- WIFI DEBUG -----");

  Serial.print("SSID: ");
  Serial.println(ssid);

  Serial.print("Agent IP: ");
  Serial.println(agent_ip);

  Serial.print("Agent Port: ");
  Serial.println(agent_port);

  Serial.println("Connecting to WiFi...");

  WiFi.begin(ssid, password);

  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");

    attempts++;

    if (attempts > 40)
    {
      Serial.println("\nWiFi FAILED");
      return;
    }
  }

  Serial.println("\nWiFi CONNECTED");

  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("Signal RSSI: ");
  Serial.println(WiFi.RSSI());

  Serial.println("----------------------");
}

// ------------ SETUP ------------
void setup()
{
  Serial.begin(115200);
  Serial.println(esp_reset_reason());
  delay(2000);

  Serial.println("Motor Controller Start");
  debug_wifi();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    encoder_isr,
    RISING);

  ledcSetup(pwmChannel, pwmFreq, pwmRes);
  ledcAttachPin(ENA, pwmChannel);

  // microROS transport
  set_microros_wifi_transports(
    ssid,
    password,
    agent_ip,
    agent_port);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(
    &node,
    "esp32_motor_node",
    "",
    &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/set_point");

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/motor_output");

  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(50),
    control_callback);

  rclc_executor_init(
    &executor,
    &support.context,
    2,
    &allocator);

  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg_in,
    &setpoint_callback,
    ON_NEW_DATA);

  rclc_executor_add_timer(
    &executor,
    &timer);

  Serial.println("System Ready");
}


// ------------ LOOP ------------
void loop()
{
  rclc_executor_spin_some(
    &executor,
    RCL_MS_TO_NS(10));
}