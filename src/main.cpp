#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include "LittleFS.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <cppQueue.h>

// ==================================== Configuration ====================================

// All MQTT topics live below this base, e.g. esp/jalousie/cmd/channel1
#define MQTT_TOPIC_BASE "esp/jalousie"

#define TOPIC_CMD_PREFIX MQTT_TOPIC_BASE "/cmd/channel"
#define TOPIC_CMD_SUBSCRIBE MQTT_TOPIC_BASE "/cmd/#"
#define TOPIC_AVAILABILITY MQTT_TOPIC_BASE "/state/availability"
#define TOPIC_CH_SELECT MQTT_TOPIC_BASE "/state/CH_SELECT"
#define TOPIC_COMMAND_LAST MQTT_TOPIC_BASE "/state/COMMAND_LAST"
#define TOPIC_ERROR MQTT_TOPIC_BASE "/state/ERROR"

// Channel 0 is "all channels", 1..4 are the single channels. The remote cycles 0 -> 1 -> 2 -> 3 -> 4 -> 0.
#define NUMBER_SOMFY_CHANNELS 5

// Send "up" to all channels after power-up (behaviour of the original firmware).
#define STARTUP_ALL_UP 1

// Remote timing. Every CH press of one channel switch must happen while the remote's LEDs are
// still lit, otherwise a press only wakes the remote up instead of advancing the channel.
// One press + observation takes BUTTON_PRESS_MS + LED_SETTLE_MS + LED_MEASURE_MS (~0.5s).
#define BUTTON_PRESS_MS 250
#define MY_PRESS_MS 500
#define LED_SETTLE_MS 150          // wait after releasing CH before sampling the LEDs
#define LED_MEASURE_MS 100         // sampling window on PIN_CH1 (200Hz -> ~20 edges while "all" is lit)
#define ALL_CHANNELS_MIN_EDGES 5   // at least this many edges in the window -> "all" is selected
#define MAX_SYNC_PRESSES (2 * NUMBER_SOMFY_CHANNELS)
#define MAX_SELECT_ATTEMPTS 3

#define STATUS_INTERVAL_MS (30UL * 1000UL)
#define MQTT_RECONNECT_INTERVAL_MS (5UL * 1000UL)

#ifdef ESP32
#define PIN_CH1 32 // 200Hz pulses while all channels lit, low active while channel 1 selected
#define PIN_CH 33
#define PIN_UP 25
#define PIN_MY 26
#define PIN_DWN 27
#else
#define PIN_CH1 D5 // 200Hz pulses while all channels lit, low active while channel 1 selected
#define PIN_CH D0
#define PIN_UP D6
#define PIN_MY D7
#define PIN_DWN D8
#endif

// ==================================== Types ====================================

#define FOREACH_CMD(SCOMMAND) \
  SCOMMAND(s_NONE)            \
  SCOMMAND(s_UP)              \
  SCOMMAND(s_MY)              \
  SCOMMAND(s_STOP)            \
  SCOMMAND(s_DOWN)            \
  SCOMMAND(s_Last)

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

const char *SOMFY_CMD_STRING[] = {
    FOREACH_CMD(GENERATE_STRING)};

typedef enum somfy_cmd
{
  FOREACH_CMD(GENERATE_ENUM)
} eSomfy_Cmd;

// Stored by value in the queue (no heap allocation).
typedef struct somfy_command
{
  eSomfy_Cmd command;
  int channel;
} sSomfy_Command;

// ==================================== Globals ==========================================

char wifiSsid[256] = "";
char wifiPassword[256] = "";
char mqttServer[256] = "";
int mqttPort = 0;
char mqttUser[256] = "";
char mqttPassword[256] = "";
char mqttClientId[256] = "";

WiFiClient espClient;
PubSubClient mqtt_client(espClient);

cppQueue qSomfyCommands(sizeof(sSomfy_Command), 8, FIFO);

volatile uint32_t number_of_edges = 0;
#ifdef ESP32
portMUX_TYPE edges_mux = portMUX_INITIALIZER_UNLOCKED;
#endif

int channel_selected = -1; // last channel confirmed by a successful switch, -1 = unknown
eSomfy_Cmd command_last_executed = s_NONE;
unsigned long last_status_millis = 0;
unsigned long last_reconnect_millis = 0;

// ==================================== Helpers ====================================

const char *somfy_cmd_etoc(eSomfy_Cmd command)
{
  if (command < s_NONE || command >= s_Last)
  {
    return "invalid";
  }
  return SOMFY_CMD_STRING[command];
}

// Exact, case-insensitive match. Returns s_NONE for unknown payloads.
eSomfy_Cmd somfy_cmd_ctoe(const char *command)
{
  if (!strcasecmp(command, "up"))
    return s_UP;
  if (!strcasecmp(command, "down"))
    return s_DOWN;
  if (!strcasecmp(command, "my"))
    return s_MY;
  if (!strcasecmp(command, "stop"))
    return s_STOP;
  return s_NONE;
}

void IRAM_ATTR ch1_edge_counter_ISR()
{
#ifdef ESP32
  portENTER_CRITICAL_ISR(&edges_mux);
#endif
  number_of_edges++;
#ifdef ESP32
  portEXIT_CRITICAL_ISR(&edges_mux);
#endif
}

// Atomically read and reset the edge counter.
uint32_t take_edges()
{
#ifdef ESP32
  portENTER_CRITICAL(&edges_mux);
#else
  noInterrupts();
#endif
  uint32_t n = number_of_edges;
  number_of_edges = 0;
#ifdef ESP32
  portEXIT_CRITICAL(&edges_mux);
#else
  interrupts();
#endif
  return n;
}

// Delay that keeps the MQTT connection serviced. Incoming commands are only queued.
void wait_ms(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms)
  {
    mqtt_client.loop();
    delay(1);
  }
}

// Buttons are active low, idle HIGH.
void press_button(uint8_t pin, unsigned long ms)
{
  digitalWrite(pin, LOW);
  wait_ms(ms);
  digitalWrite(pin, HIGH);
}

void publish_error(const char *message)
{
  Serial.printf("ERROR -> %s\n", message);
  mqtt_client.publish(TOPIC_ERROR, message, true);
}

void publish_status()
{
  char msg[64];
  snprintf(msg, sizeof(msg), "{\"channel\": %d}", channel_selected);
  mqtt_client.publish(TOPIC_CH_SELECT, msg, true);
  snprintf(msg, sizeof(msg), "{\"command\": \"%s\"}", somfy_cmd_etoc(command_last_executed));
  mqtt_client.publish(TOPIC_COMMAND_LAST, msg, true);
  last_status_millis = millis();
}

// ==================================== Remote control ====================================

// Look at the remote's LEDs after a CH press.
// Returns 0 ("all" lit), 1 (channel 1 lit) or -1 (channel 2-4 or LEDs off; not distinguishable).
int observe_remote()
{
  wait_ms(LED_SETTLE_MS);
  take_edges();
  wait_ms(LED_MEASURE_MS);
  uint32_t edges = take_edges();
  int level = digitalRead(PIN_CH1);

  int seen = -1;
  if (edges >= ALL_CHANNELS_MIN_EDGES)
  {
    seen = 0;
  }
  else if (level == LOW)
  {
    seen = 1;
  }
  Serial.printf("observe -> edges: %u, CH1 level: %d => %s\n", (unsigned)edges, level,
                seen == 0 ? "all (0)" : seen == 1 ? "channel 1" : "channel 2-4 or LEDs off");
  return seen;
}

// What observe_remote() must report when the remote really is on `channel`.
int expected_observation(int channel)
{
  return channel <= 1 ? channel : -1;
}

// Select `target` on the remote. The current channel is detected again every time, so it does not
// matter whether the first press only wakes the remote or whether someone used the remote by hand.
bool select_channel(int target)
{
  for (int attempt = 1; attempt <= MAX_SELECT_ATTEMPTS; attempt++)
  {
    Serial.printf("select -> target %d, attempt %d\n", target, attempt);

    // 1. Press CH until the remote shows a channel we can recognise (all or 1).
    int current = -1;
    for (int i = 0; i < MAX_SYNC_PRESSES && current < 0; i++)
    {
      press_button(PIN_CH, BUTTON_PRESS_MS);
      current = observe_remote();
    }
    if (current < 0)
    {
      publish_error("channel sync failed: neither 'all' nor channel 1 seen on the remote");
      return false;
    }

    // 2. Step forward one press at a time; verify whenever we pass a recognisable channel.
    bool consistent = true;
    while (current != target)
    {
      press_button(PIN_CH, BUTTON_PRESS_MS);
      current = (current + 1) % NUMBER_SOMFY_CHANNELS;
      int seen = observe_remote();
      if (seen != expected_observation(current))
      {
        Serial.printf("select -> expected channel %d but remote shows %d, resyncing\n", current, seen);
        consistent = false;
        break;
      }
    }

    if (consistent)
    {
      Serial.printf("select -> channel %d selected\n", target);
      return true;
    }
  }

  publish_error("channel switch failed: remote did not follow the CH presses");
  return false;
}

void execute_command(const sSomfy_Command &cmd)
{
  Serial.printf("execute -> channel %d, command %s\n", cmd.channel, somfy_cmd_etoc(cmd.command));

  uint8_t pin;
  unsigned long press_ms = BUTTON_PRESS_MS;
  switch (cmd.command)
  {
  case s_UP:
    pin = PIN_UP;
    break;
  case s_DOWN:
    pin = PIN_DWN;
    break;
  case s_MY:
    press_ms = MY_PRESS_MS;
    pin = PIN_MY;
    break;
  case s_STOP:
    pin = PIN_MY;
    break;
  default:
    publish_error("invalid command in queue, ignored");
    return;
  }

  if (!select_channel(cmd.channel))
  {
    channel_selected = -1;
    return;
  }
  channel_selected = cmd.channel;

  press_button(pin, press_ms);
  command_last_executed = cmd.command;
  Serial.println("execute -> done");
}

// ==================================== MQTT ====================================

bool mqtt_connect()
{
  if (!mqtt_client.connect(mqttClientId, mqttUser, mqttPassword, TOPIC_AVAILABILITY, 0, true, "offline"))
  {
    return false;
  }
  mqtt_client.publish(TOPIC_AVAILABILITY, "online", true);
  mqtt_client.subscribe(TOPIC_CMD_SUBSCRIBE);
  return true;
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  Serial.printf("MQTT_callback -> topic: %s\n", topic);

  // channel number: everything after the prefix, digits only, nothing behind it
  const size_t prefix_len = strlen(TOPIC_CMD_PREFIX);
  if (strncmp(topic, TOPIC_CMD_PREFIX, prefix_len) != 0)
  {
    Serial.println("MQTT_callback -> not a channel topic, ignored");
    return;
  }
  const char *num = topic + prefix_len;
  char *end = NULL;
  long channel = strtol(num, &end, 10);
  if (end == num || *end != '\0' || channel < 0 || channel >= NUMBER_SOMFY_CHANNELS)
  {
    Serial.printf("MQTT_callback -> invalid channel '%s', ignored\n", num);
    return;
  }

  char c_payload[16];
  if (length >= sizeof(c_payload))
  {
    Serial.println("MQTT_callback -> payload too long, ignored");
    return;
  }
  memcpy(c_payload, payload, length);
  c_payload[length] = '\0';

  eSomfy_Cmd command = somfy_cmd_ctoe(c_payload);
  if (command == s_NONE)
  {
    Serial.printf("MQTT_callback -> unknown command '%s', ignored\n", c_payload);
    return;
  }

  sSomfy_Command req;
  req.command = command;
  req.channel = (int)channel;
  if (!qSomfyCommands.push(&req))
  {
    publish_error("command queue full, command dropped");
    return;
  }
  Serial.printf("MQTT_callback -> queued channel %d, command %s (queue: %d)\n",
                req.channel, somfy_cmd_etoc(req.command), qSomfyCommands.getCount());
}

// ==================================== SETUP ====================================

void halt(const char *message)
{
  Serial.println(message);
  for (;;)
    delay(1000);
}

bool copy_config_string(JsonDocument &doc, const char *key, char *dest, size_t len, bool required)
{
  const char *value = doc[key] | "";
  if (!*value)
  {
    if (required)
    {
      Serial.printf("Config: '%s' missing or empty\n", key);
    }
    return !required;
  }
  if (strlcpy(dest, value, len) >= len)
  {
    Serial.printf("Config: '%s' too long (max %u)\n", key, (unsigned)(len - 1));
    return false;
  }
  return true;
}

void load_config()
{
  if (!LittleFS.begin())
  {
    halt("LittleFS mount failed.");
  }
  File cfile = LittleFS.open("/config.json", "r");
  if (!cfile)
  {
    halt("Config file missing.");
  }

  DynamicJsonDocument cjson(2048);
  DeserializationError err = deserializeJson(cjson, cfile);
  cfile.close();
  if (err)
  {
    Serial.printf("Config: JSON error: %s\n", err.c_str());
    halt("Config file invalid.");
  }

  bool ok = true;
  ok &= copy_config_string(cjson, "wifiSsid", wifiSsid, sizeof(wifiSsid), true);
  ok &= copy_config_string(cjson, "wifiPassword", wifiPassword, sizeof(wifiPassword), false);
  ok &= copy_config_string(cjson, "mqttServer", mqttServer, sizeof(mqttServer), true);
  ok &= copy_config_string(cjson, "mqttUser", mqttUser, sizeof(mqttUser), false);
  ok &= copy_config_string(cjson, "mqttPassword", mqttPassword, sizeof(mqttPassword), false);
  ok &= copy_config_string(cjson, "mqttClientID", mqttClientId, sizeof(mqttClientId), true);

  // accept "1883" as well as 1883
  JsonVariant port = cjson["mqttPort"];
  if (port.is<int>())
  {
    mqttPort = port.as<int>();
  }
  else if (port.is<const char *>())
  {
    mqttPort = atoi(port.as<const char *>());
  }
  if (mqttPort <= 0 || mqttPort > 65535)
  {
    Serial.println("Config: 'mqttPort' missing or invalid");
    ok = false;
  }

  if (!ok)
  {
    halt("Config file incomplete.");
  }
  Serial.println("-- Setup: Config loaded.");
}

void setup()
{
  Serial.begin(115200);
  Serial.println("-- Setup: Start.");

  // DIO: buttons idle HIGH, LED input with edge counter
  pinMode(PIN_CH, OUTPUT);
  pinMode(PIN_UP, OUTPUT);
  pinMode(PIN_MY, OUTPUT);
  pinMode(PIN_DWN, OUTPUT);
  digitalWrite(PIN_CH, HIGH);
  digitalWrite(PIN_UP, HIGH);
  digitalWrite(PIN_MY, HIGH);
  digitalWrite(PIN_DWN, HIGH);
  pinMode(PIN_CH1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_CH1), ch1_edge_counter_ISR, FALLING);

  load_config();

  // WiFi
  Serial.println("-- Setup: Connecting to WiFi");
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
#ifndef ESP32
  WiFi.setOutputPower(10.0);
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
#endif
  WiFi.setAutoReconnect(true);
  WiFi.begin(wifiSsid, wifiPassword);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("...Connecting to WiFi");
    delay(1000);
  }
  Serial.println("-- Setup: Connected to WiFi");

  // MQTT
  mqtt_client.setServer(mqttServer, mqttPort);
  mqtt_client.setCallback(mqtt_callback);
  while (!mqtt_connect())
  {
    Serial.printf("...Connecting to MQTT failed, state: %d\n", mqtt_client.state());
    delay(2000);
  }
  Serial.println("-- Setup: Connected to MQTT");

#if STARTUP_ALL_UP
  sSomfy_Command req;
  req.command = s_UP;
  req.channel = 0;
  qSomfyCommands.push(&req);
#endif

  publish_status();
  Serial.println("-- End of Setup --");
}

// ==================================== LOOP ====================================

void loop()
{
  if (!mqtt_client.connected())
  {
    if (millis() - last_reconnect_millis >= MQTT_RECONNECT_INTERVAL_MS)
    {
      last_reconnect_millis = millis();
      Serial.println("loop -> MQTT disconnected, reconnecting");
      mqtt_connect();
    }
  }
  mqtt_client.loop();

  sSomfy_Command cmd;
  if (qSomfyCommands.pop(&cmd))
  {
    execute_command(cmd);
    publish_status();
  }

  if (millis() - last_status_millis >= STATUS_INTERVAL_MS)
  {
    publish_status();
  }
}
