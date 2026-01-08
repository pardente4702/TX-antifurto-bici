#include <esp_now.h>
#include <WiFi.h>

#define PIR_PIN 13      // Pin PIR
#define LED_PIN 2       // Pin LED
#define MAX_RETRIES 3   // Numero massimo di ritrasmissioni
#define ACK_TIMEOUT 500 // Timeout per ACK in ms

// MAC del ricevitore (da sostituire con il MAC reale)
uint8_t receiverMAC[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

typedef struct
{
  uint8_t id;
  bool motion;
} msg_t;

volatile bool motionDetected = false; // Flag set dall'interrupt
bool ackReceived = false;             // Flag per ACK

// Interrupt PIR
void IRAM_ATTR pirISR()
{
  motionDetected = true;
}

// Callback di invio ESP-NOW (solo conferma radio)
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    Serial.println("Pacchetto inviato fisicamente");
  }
  else
  {
    Serial.println("Errore invio pacchetto");
  }
}

// Callback per ricevere l'ACK dal ricevitore
void onDataReceive(const uint8_t *mac_addr, const uint8_t *data, int len)
{
  if (len == sizeof(msg_t))
  {
    msg_t ackMsg;
    memcpy(&ackMsg, data, sizeof(msg_t));
    if (!ackMsg.motion)
    { // flag motion=false indica ACK
      ackReceived = true;
      Serial.println("ACK ricevuto dal ricevitore");
    }
  }
}

void setup()
{
  Serial.begin(115200);

  // Pin setup
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  attachInterrupt(PIR_PIN, pirISR, RISING); // interrupt sul PIR

  // ESP-NOW setup
  WiFi.mode(WIFI_STA); // necessario per ESP-NOW
  WiFi.channel(1);
  // WiFi.setChannel(1);  // fissare canale

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Errore inizializzazione ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceive);

  // Aggiungi peer (ricevitore)
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = 1;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK)
  {
    Serial.println("Errore aggiunta peer");
  }
}

void loop()
{
  if (motionDetected)
  {
    motionDetected = false;
    digitalWrite(LED_PIN, HIGH); // accendi LED quando PIR rileva

    msg_t msg = {1, true};
    ackReceived = false;

    int retries = 0;
    while (!ackReceived && retries < MAX_RETRIES)
    {
      esp_now_send(receiverMAC, (uint8_t *)&msg, sizeof(msg));

      unsigned long start = millis();
      while (millis() - start < ACK_TIMEOUT)
      {
        if (ackReceived)
          break; // se ricevi ACK esci dal loop
      }

      if (!ackReceived)
      {
        Serial.println("Ritrasmissione...");
        retries++;
      }
    }

    if (ackReceived)
    {
      Serial.println("Messaggio confermato dal ricevitore");
    }
    else
    {
      Serial.println("ACK non ricevuto dopo ritrasmissioni");
    }

    // Mantieni il LED acceso 2 secondi poi spegni
    delay(2000);
    digitalWrite(LED_PIN, LOW);
  }
}
