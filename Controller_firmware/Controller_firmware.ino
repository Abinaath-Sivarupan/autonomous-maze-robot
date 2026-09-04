#include <Keypad.h>
#include <LiquidCrystal.h>
#include <esp_now.h>
#include <WiFi.h>

/* ========== KEYPAD ========== */

const byte ROWS = 4;
const byte COLS = 3;

char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

byte rowPins[ROWS] = {23, 22, 21, 19};
byte colPins[COLS] = {18, 5, 17};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

char qtyBuf[3];     // stores two digits + null
int qtyIndex = 0;   // how many digits entered

/* ========== LCD ========== */

LiquidCrystal lcd(32, 33, 25, 26, 27, 14);

/* ========== COMMAND STORAGE ========== */

int commandList[30];   // movement, qty, movement, qty...
int cmdPtr = 0;        // next free slot
int lastMovement = -1; // 2,4,6,8

/* ========== STATES ========== */

#define INPUT_MOVE 0
#define INPUT_QTY  1

int state = INPUT_MOVE;

/* ========== ESPNOW ========== */

uint8_t broadcastAddress[] = {0x08, 0x3a, 0x8d, 0x0d, 0x92, 0xac};
String success;
int executionSuccess;

esp_now_peer_info_t peerInfo;

/* ========== SETUP ========== */

void setup() {
  Serial.begin(115200);

  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);   // LED off initially

  clearCommands();
  initialiseLCD();
  initialiseESPNOW();
}

/* ========== LOOP ========== */

void loop() {

  handleKey();

}

/* ========== KEY HANDLER ========== */

void handleKey() {
  char key = keypad.getKey();

  if (key) {
    digitalWrite(12, HIGH);   // LED ON when a key is pressed
  } else {
    digitalWrite(12, LOW);    // LED OFF when no key is pressed
  }

  if (!key) return;

  if (key == '#') { clearCommands(); updateLCD(); return; }
  if (key == '*') { sendCommands(); return; }

  if (state == INPUT_MOVE) handleMovementKey(key);
  else handleQtyKey(key);

  updateLCD();
}

/* ========== MOVEMENT HANDLER ========== */

void handleMovementKey(char key) {
  if (key=='2' || key=='4' || key=='6' || key=='8') {
    lastMovement = key - '0';
    commandList[cmdPtr++] = lastMovement;
    state = INPUT_QTY;
  }
}

/* ========== QUANTITY HANDLER ========== */

void handleQtyKeyOLD(char key) {

  // Forward/back: 1–9 distance
  if (lastMovement == 2 || lastMovement == 8) {
    if (key >= '1' && key <= '9') {
      commandList[cmdPtr++] = key - '0';
      state = INPUT_MOVE;
    }
  }

  // Left/right: 7 = 90, 9 = 180
  else if (lastMovement == 4 || lastMovement == 6) {
    if (key == '7') commandList[cmdPtr++] = 90;
    else if (key == '9') commandList[cmdPtr++] = 180;
    state = INPUT_MOVE;
  }
}

void handleQtyKey(char key) {

  // --- 1. TURN SHORTCUTS ALWAYS TAKE PRIORITY ---
  if (lastMovement == 4 || lastMovement == 6) {   // LEFT or RIGHT
    if (key == '7') {
      commandList[cmdPtr++] = 90;
      qtyIndex = 0;
      state = INPUT_MOVE;
      return;
    }
    if (key == '9') {
      commandList[cmdPtr++] = 180;
      qtyIndex = 0;
      state = INPUT_MOVE;
      return;
    }
  }

  // --- 2. TWO-DIGIT ENTRY FOR ALL MOVEMENTS ---
  if (key < '0' || key > '9') return;

  qtyBuf[qtyIndex++] = key;

  if (qtyIndex < 2) {
    return;   // wait for second digit
  }

  qtyBuf[2] = '\0';
  int qty = atoi(qtyBuf);

  commandList[cmdPtr++] = qty;

  qtyIndex = 0;
  state = INPUT_MOVE;
}
/* ========== COMMAND MANAGEMENT ========== */

void clearCommands() {
  for (int i=0; i<30; i++) commandList[i] = -1;
  cmdPtr = 0;
  state = INPUT_MOVE;
}

void sendCommands() {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)commandList, sizeof(commandList));

  lcd.clear();
  lcd.setCursor(0,0);

  if (result == ESP_OK) lcd.print("Sent to EEEbot!");
  else lcd.print("Send Failed");
  Serial.println("Sent to EEEbot!");
  delay(1000);
  updateLCD();
}

/* ========== LCD ========== */

void initialiseLCD() {
  lcd.begin(16,2);
  lcd.clear();
  lcd.print("Input Movement");
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0,0);

  if (state == INPUT_MOVE) lcd.print("Input Movement");
  else lcd.print("Input Quantity");

  displayCMDS();
}

/* ========== CLEAN COMMAND DISPLAY ========== */

void displayCMDS() {
  lcd.setCursor(0, 1);

  // Build a single string of all commands
  String line = "";

  for (int i = 0; i < cmdPtr; i += 2) {
    int mv = commandList[i];
    int qt = commandList[i + 1];

    if (mv == -1 || qt == -1) break;

    // Movement letter
    if (mv == 2) line += "F";
    if (mv == 8) line += "B";
    if (mv == 4) line += "L";
    if (mv == 6) line += "R";

    // Quantity
    line += String(qt);
    line += " ";
  }

  // If longer than 16 chars, show the last 16
  if (line.length() > 16) {
    line = line.substring(line.length() - 16);
  }

  // Print to LCD
  lcd.print(line);
}

/* ========== ESPNOW CALLBACKS ========== */

void initialiseESPNOW() {
  // set the device as a Wi-Fi station
  WiFi.mode(WIFI_STA);

  // initialise ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initialising ESP-NOW");
    return;
  }

  // once ESP-NOW is successfully initialised, we will register for send callback to get the status of transmitted packet
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  
  // register the peer device
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // add the peer device        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  success = (status == ESP_NOW_SEND_SUCCESS) ? "Success" : "Fail";
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  int status;
  memcpy(&status, incomingData, sizeof(status));

  
  if (status == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sent to EEEBot!");
    lcd.setCursor(0, 1);
    lcd.print("ACK Received!");

    Serial.println("Ack Recieved");
  } 
  else if (status > 0) {
    // Display to LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Executing Cmd:");
    lcd.setCursor(0, 1);
    lcd.print(status);

    Serial.println("Current Command: ");
    Serial.println(status);

  } else if (status == -7) { //all complete
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("All Commands");
    lcd.setCursor(0, 1);
    lcd.print("Executed!");
  }

  delay(1000);
  updateLCD();
}