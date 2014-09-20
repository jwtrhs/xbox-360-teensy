#include <IRremote.h>
#include <Bounce.h>

// Take care not to leave this on for production, it slows the repsonse time
// of the IR Remote -> USB HID quite a lot.
#define DEBUG 0

// Input/Output pins
#define ir_pin          0  // data from IR receiver
#define clock_pin       5  // clock line from RF module
#define data_pin        6  // data line to RF module
#define pwr_btn_pin     21 // power button from RF module
#define pwr_stat_pin    20 // power status from motherboard (led- on F_PANEL header)
#define pwr_switch_pin  19 // power switch to motherboard (power+ on F_PANEL header);
#define sync_btn_pin    18 // sync button
#define eject_btn_pin   3  // eject button
#define led_pin         11 // Teensy onboard LED 

// Inputs
#define BOUNCE_INTERVAL 10
Bounce pwr_btn = Bounce(pwr_btn_pin, BOUNCE_INTERVAL);
Bounce pwr_stat = Bounce(pwr_stat_pin, BOUNCE_INTERVAL);
Bounce sync_btn = Bounce(sync_btn_pin, BOUNCE_INTERVAL);
Bounce eject_btn = Bounce(eject_btn_pin, BOUNCE_INTERVAL);

// IR Receiver
IRrecv irrecv(ir_pin);
decode_results results;

// USB HID state
enum HID_STATE
{
  HID_STATE_IDLE,    // sending idle data
  HID_STATE_ACTIVE   // sending actual data
};

// data for USB HID device
unsigned int  hid_state          = HID_STATE_IDLE;
unsigned long last_toggle        = 0;
unsigned long last_transmit_time = micros();

// USB HID timeouts
#define USB_HID_KEY_PRESS_TIMEOUT  200000 // 200 ms
#define USB_HID_IDLE_TIMEOUT       500000 // 500 ms

// Keyboard USB HID modifier masks
#define KEYBOARD_LEFT_CTRL_MASK   0x01
#define KEYBOARD_LEFT_SHIFT_MASK  0x02
#define KEYBOARD_LEFT_ALT_MASK    0x04
#define KEYBOARD_LEFT_GUI_MASK    0x08
#define KEYBOARD_RIGHT_CTRL_MASK  0x10
#define KEYBOARD_RIGHT_SHIFT_MASK 0x20
#define KEYBOARD_RIGHT_ALT_MASK   0x40
#define KEYBOARD_RIGHT_GUI_MASK   0x80

// USB HID report IDs
#define KEYBOARD_REPORT_ID         0x01
#define CONSUMER_CONTROL_REPORT_ID 0x02

// USB HID report data packet
#define DATA_SIZE                  8
byte data[DATA_SIZE];
const byte NULL_DATA[DATA_SIZE] = {0};

// Xbox 360 remote control mask, last byte is command
#define XBOX_360_REMOTE_MASK 0x000F7400UL

// Xbox 360 remote codes (small/basic model)
#define BTN_POWER        0x0C
#define BTN_XBOX_GUIDE   0x64
#define BTN_EJECT        0x28
#define BTN_PLAY         0x16
#define BTN_STOP         0x19
#define BTN_PAUSE        0x18
#define BTN_REWIND       0x15
#define BTN_FAST_FORWARD 0x14
#define BTN_SCAN_PREV    0x1B
#define BTN_SCAN_NEXT    0x1A
#define BTN_DVD_MENU     0x24
#define BTN_DISPLAY      0x4F
#define BTN_TITLE        0x51
#define BTN_BACK         0x23
#define BTN_INFO         0x0F
#define BTN_OK           0x22
#define BTN_UP           0x1E
#define BTN_DOWN         0x1F
#define BTN_LEFT         0x20
#define BTN_RIGHT        0x21
#define BTN_YELLOW       0x26
#define BTN_BLUE         0x13
#define BTN_GREEN        0x12
#define BTN_RED          0x25
#define BTN_MCE          0x0D
#define BTN_REC          0x17

// RF module commands
// See here: http://forums.xbox-experts.com/viewtopic.php?t=4029
// Also here: http://tkkrlab.nl/wiki/XBOX_360_RF_Module (note: some of the binary/hex representations don't match)
#define RF_CMD_SYNC           0x004
#define RF_CMD_CTRLR_OFF      0x009
#define RF_CMD_LED_OFF        0x080
#define RF_CMD_LED_INIT       0x084
#define RF_CMD_BOOTANIM       0x085

// RF module serial timeout
#define RF_MODULE_TIMEOUT 100000  // 100 ms

void setup()
{
  #if DEBUG
  Serial.begin(9600);
  Serial.println();
  Serial.println("setup started");
  #endif
  
  // Start the IR receiver
  irrecv.enableIRIn();
  
  // Setup the RF module pins
  pinMode(data_pin, INPUT);
  pinMode(clock_pin, INPUT);
  
  // Setup outputs
  pinMode(pwr_switch_pin, INPUT); // Though this is an output, we want to leave it floating when not in use
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);
  
  // Setup inputs
  pinMode(pwr_btn_pin, INPUT_PULLUP);
  pinMode(pwr_stat_pin, INPUT); 
  pinMode(sync_btn_pin, INPUT_PULLUP);
  pinMode(eject_btn_pin, INPUT_PULLUP);
  
  // Update inputs
  pwr_btn.update();
  pwr_stat.update();
  sync_btn.update();
  eject_btn.update();
  
  // Initialise RF module LEDs if power is already on
  if (pwr_stat.read())
  {
    // Initialise the LED ring
    send_rf_module_cmd(RF_CMD_LED_INIT);
    delay(50);
    // Display the boot animation
    //send_rf_module_cmd(RF_CMD_BOOTANIM);
    //delay(50);
  }
  
  #if DEBUG
  Serial.println("setup complete");
  #endif
}

void loop() {
  // Check for received IR signals and handle them
  handle_ir();
  
  // Check push buttons and power status
  handle_inputs();
  
  // Set the status led
  digitalWrite(led_pin, (hid_state == HID_STATE_ACTIVE)
    || !pwr_btn.read() || !eject_btn.read() || !sync_btn.read());
}

// Handle IR receiver data
void handle_ir()
{
  if (irrecv.decode(&results))
  {
    #if DEBUG
    Serial.print("IR command recvd: 0x");
    //Serial.print((unsigned long)(results.value >> 32), HEX);
    Serial.print((unsigned long)(results.value), HEX);
    Serial.print(" ");
    Serial.println(results.decode_type);
    #endif DEBUG
    
    // only handle commands from the Xbox 360 remote
    if ((results.value & XBOX_360_REMOTE_MASK) == XBOX_360_REMOTE_MASK
        && results.decode_type == RC6)
    {
      unsigned long toggle = results.value & 0x00008000;
      byte code = results.value & 0x000000FF;
      
      // if toggle has changed, new button press
      //if (toggle != last_toggle)
      {
        // clear the data buffer
        for (int i = 0; i < DATA_SIZE; ++i)
          data[i] = 0x00;
        
        byte report_id = 0x00;
        switch (code)
        {
        case BTN_POWER:
          report_id = CONSUMER_CONTROL_REPORT_ID;
          data[0] = 0x01;
          break;
        case BTN_EJECT:
          //report_id = CONSUMER_CONTROL_REPORT_ID;
          //data[0] = 0x02;
          send_rf_module_cmd(RF_CMD_CTRLR_OFF);
          delay(50);
          break;
        case BTN_PLAY:
          report_id = CONSUMER_CONTROL_REPORT_ID;
          data[0] = 0x04;
          break;
        case BTN_STOP:
          report_id = CONSUMER_CONTROL_REPORT_ID;
          data[0] = 0x08;
          break;
        case BTN_PAUSE:
          report_id = CONSUMER_CONTROL_REPORT_ID;
          data[0] = 0x10;
          break;
        case BTN_REWIND:
          report_id = CONSUMER_CONTROL_REPORT_ID;
          data[0] = 0x20;
          break;
        case BTN_FAST_FORWARD:
          report_id = CONSUMER_CONTROL_REPORT_ID;
          data[0] = 0x40;
          break;
        case BTN_SCAN_PREV:
          report_id = CONSUMER_CONTROL_REPORT_ID;
          data[0] = 0x80;
          break;
        case BTN_SCAN_NEXT:
          report_id = CONSUMER_CONTROL_REPORT_ID;
          data[1] = 0x01;
          break;
        case BTN_OK:
          report_id = KEYBOARD_REPORT_ID;
          data[2] = 0x28;
          break;
        case BTN_UP:
          report_id = KEYBOARD_REPORT_ID;
          data[2] = 0x52;
          break;
        case BTN_DOWN:
          report_id = KEYBOARD_REPORT_ID;
          data[2] = 0x51;
          break;
        case BTN_LEFT:
          report_id = KEYBOARD_REPORT_ID;
          data[2] = 0x50;
          break;
        case BTN_RIGHT:
          report_id = KEYBOARD_REPORT_ID;
          data[2] = 0x4F;
          break;
        case BTN_BACK:
          report_id = KEYBOARD_REPORT_ID;
          data[2] = 0x29;
          break;
        case BTN_XBOX_GUIDE:
          report_id = KEYBOARD_REPORT_ID;
          data[2] = 0xE3;
          break;
        case BTN_MCE:
          report_id = KEYBOARD_REPORT_ID;
          data[0] = KEYBOARD_LEFT_GUI_MASK;
          data[2] = 0x04;
          break;
        default:
          break;
        }
        
        switch (report_id)
        {
        case KEYBOARD_REPORT_ID:
          if (toggle != last_toggle)  // If new button press, then reset the HID message
            send_hid_report(report_id, NULL_DATA, DATA_SIZE);
          send_hid_report(report_id, data, DATA_SIZE);
          break;
        case CONSUMER_CONTROL_REPORT_ID:
          if (toggle != last_toggle)
          {
            send_hid_report(report_id, data, DATA_SIZE);
            // Reset the HID message
            send_hid_report(report_id, NULL_DATA, DATA_SIZE);
          }
          break;
        default:
          break;
        };
        
        hid_state = HID_STATE_ACTIVE;
      }
      
      last_toggle = toggle;
    }
    irrecv.resume(); // Receive the next value
  }
  
  // check keyboard timeout or need to send idle transmission
  if ((hid_state == HID_STATE_ACTIVE && time_diff(last_transmit_time, micros()) > USB_HID_KEY_PRESS_TIMEOUT)
     || (hid_state == HID_STATE_IDLE && time_diff(last_transmit_time, micros()) > USB_HID_IDLE_TIMEOUT))
  {
    send_hid_report(KEYBOARD_REPORT_ID, NULL_DATA, DATA_SIZE);
    send_hid_report(CONSUMER_CONTROL_REPORT_ID, NULL_DATA, DATA_SIZE);
    hid_state = HID_STATE_IDLE;
  }
}

// Transmits a HID report
// report_id is first byte of report packet, followed by data
int send_hid_report(byte report_id, const void* ptr, const int length)
{
  last_transmit_time = micros();
  byte buffer[length + 1];
  buffer[0] = report_id;
  for (int i = 0; i < length; ++i)
    buffer[i + 1] = *((const byte *)ptr + i);
  return RawHID.send(buffer, 100);
}

// Calculate the difference between two times, taking into account overflow
// Assumes that time1 is older or equal to time2
unsigned long time_diff(unsigned long time1, unsigned long time2)
{
  if (time1 > time2)
    return 0xFFFFFFFFUL - time1 + time2;
  else return time2 - time1;
}

// Handle inputs, incluing ipower button, sync button and eject button
void handle_inputs()
{
  // Check the power button
  pwr_btn.update();
  #if DEBUG
  if (pwr_btn.risingEdge()) Serial.println("pwr_btn rising");
  if (pwr_btn.fallingEdge()) Serial.println("pwr_btn falling");
  #endif
  // The pwr_switch+ pin on the front panel is left floating at 3.3V when open,
  // and is driven to GND when closed (i.e. when the button is pushed)
  // Note: the mobo I am using is an ASRock FM2A88X-ITX+, it may be different for others
  if (pwr_btn.read())
  {
    // The power button is open (i.e. not pushed)
    // Leave the pin floating when not active
    pinMode(pwr_switch_pin, INPUT);
  }
  else
  {
    // The power button is closed (i.e. pushed)
    // Emulate a button press by driving the pin to GND
    pinMode(pwr_switch_pin, OUTPUT);
    digitalWrite(pwr_switch_pin, LOW);
  }
  
  // Check computer power status
  pwr_stat.update();
  #if DEBUG
  if (pwr_stat.risingEdge()) Serial.println("pwr_stat rising");
  if (pwr_stat.fallingEdge()) Serial.println("pwr_stat falling");
  #endif
  if (pwr_stat.risingEdge())  // Computer has just been turned on
  {
    // Initialise the RF module
    send_rf_module_cmd(RF_CMD_LED_INIT);
    delay(50);
    // Run the LED ring boot animation
    send_rf_module_cmd(RF_CMD_BOOTANIM);
    delay(50);
  }
  else if (pwr_stat.fallingEdge())  // Computer has just been turned off
  {
    // Turn off any connected controllers
    send_rf_module_cmd(RF_CMD_CTRLR_OFF);
    delay(50);
    // Turn off LED ring
    send_rf_module_cmd(RF_CMD_LED_OFF);
    delay(50);
  }
  
  // Check sync button
  sync_btn.update();
  #if DEBUG
  if (sync_btn.risingEdge()) Serial.println("sync_btn rising");
  if (sync_btn.fallingEdge()) Serial.println("sync_btn falling");
  #endif
  if (sync_btn.fallingEdge())
  {
    // Initiate sync process on RF module
    send_rf_module_cmd(RF_CMD_SYNC);
    delay(50);
  }
  
  // Check eject button
  eject_btn.update();
  #if DEBUG
  if (eject_btn.risingEdge()) Serial.println("eject_btn rising");
  if (eject_btn.fallingEdge()) Serial.println("eject_btn falling");
  #endif
  if (eject_btn.fallingEdge())
  {
    // TODO: Handle eject button
  }
}

// Sends a command through to the Xbox 360 RF module
boolean send_rf_module_cmd(int cmd_do)
{
  #if DEBUG
  Serial.print("Send to RF module: 0x");
  Serial.print(cmd_do, HEX);
  #endif
  
  boolean success = true;
  pinMode(data_pin, OUTPUT);
  digitalWrite(data_pin, LOW);    //start sending data.
  int prev = 1;
  unsigned long timer;
  for(int i = 9; i >= 0; i--){
    timer = micros(); // get the current time to monitor for timeout
    while (prev == digitalRead(clock_pin)) //detects change in clock
    {
      if (time_diff(timer, micros()) > RF_MODULE_TIMEOUT)
      {
        success = false;  // if timeout occurs, abandon serial communication
        break;
      }
    }
    if (!success) break;
    
    prev = digitalRead(clock_pin);
      // should be after downward edge of clock, so send bit of data now
    digitalWrite(data_pin, (cmd_do >> i) & 0x01);
    
    timer = micros(); // get the current time to monitor for timeout
    while (prev == digitalRead(clock_pin)) //detects upward edge of clock
    {
      if (time_diff(timer, micros()) > RF_MODULE_TIMEOUT)
      {
        success = false;  // if timeout occurs, abandon serial communication
        break;
      }
    }
    if (!success) break;
    
    prev = digitalRead(clock_pin);
  }
  digitalWrite(data_pin, HIGH);
  pinMode(data_pin, INPUT);
  
  #if DEBUG
  Serial.println(success ? ", successful." : ", failed.");
  #endif
  
  return success;
}
