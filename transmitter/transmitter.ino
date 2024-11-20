/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 02/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


/*******************************************************************************************************
  Program Operation - This is a minimum setup LoRa test transmitter. A packet containing the ASCII text
  "Hello World 1234567890" is sent using the frequency and LoRa settings specified in the LT.setupLoRa()
  command. The pins to access the lora device need to be defined at the top of the program also.

  The details of the packet sent and any errors are shown on the Arduino IDE Serial Monitor, together with
  the transmit power used and the packet length. The matching receiver program, '4_LoRa_Receiver' can be used
  to check the packets are being sent correctly, the frequency and LoRa settings (in the LT.setupLoRa()
  commands) must be the same for the transmitter and receiver programs. Sample Serial Monitor output;

  10dBm Packet> Hello World 1234567890*  BytesSent,23  PacketsSent,6

  For an example of a more detailed configuration for a transmitter, see program 103_LoRa_Transmitter.

  Serial monitor baud rate is set at 9600
*******************************************************************************************************/

// ============================================================================
//  Header
// ============================================================================

#define HELTEC_POWER_BUTTON   // must be before "#include <heltec_unofficial.h>"
#include <heltec_unofficial.h>
#include <SPI.h>                                //the lora device is SPI based so load the SPI library                                         
#include <SX126XLT.h>                           //include the appropriate library  

SX126XLT LT;                                    //create a library class instance called LT

#define NSS         8                               //select pin on LoRa device
#define NRESET     12                                //reset pin on LoRa device
#define RFBUSY     13                                //busy pin on LoRa device 
#define DIO1       14                                  //DIO1 pin on LoRa device, used for RX and TX done 
#define LORA_DEVICE DEVICE_SX1262               //we need to define the device we are using
#define TXpower 10                              //LoRa transmit power in dBm

#define SONAR_TRIGGER_PIN  45
#define SONAR_ECHO_PIN     46

uint8_t TXPacketL;
uint32_t TXPacketCount;

uint8_t buff[] = "Hello World 1234567890";      //the message to send


// ============================================================================
//  Private Functions
// ============================================================================

double kalman(const double U){
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 10;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P*H/(H*P*H+R);
  U_hat += + K*(U-H*U_hat);
  P = (1-K*H)*P+Q;
  return U_hat;
}

// ============================================================================
//  Setup and Loop
// ============================================================================

void setup()
{
  heltec_setup();
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("3_LoRa_Transmitter Starting"));

  SPI.begin();

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1);
  }

  LT.setupLoRa(433000000, 0, LORA_SF7, LORA_BW_125, LORA_CR_4_5, LDRO_AUTO); //configure frequency and LoRa settings

  pinMode(SONAR_TRIGGER_PIN, OUTPUT);  
	pinMode(SONAR_ECHO_PIN, INPUT);  


  Serial.print(F("Transmitter ready"));
  Serial.println();

  display.clear();
  display.println("Starting");
}

void loop() {
  static uint32_t count = 0;

  digitalWrite(SONAR_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  double duration = pulseIn(SONAR_ECHO_PIN, HIGH);
  double dist_cm = (duration/2) / 29.1;
  double kl_dist_cm = kalman(dist_cm);

  if ( count >= 5 ) {
    static char buffer[64];
    int buffer_len = snprintf(buffer, 64, "%f", dist_cm);
    if (LT.transmit( (uint8_t*) buffer, buffer_len, 10000, TXpower, WAIT_TX)) {
      TXPacketCount++;
      display.print("Dist: ");
      display.print(dist_cm);
      display.print(" : ");
      display.println(kl_dist_cm);
    } else {
      display.println("error");
    }
    count = 0;
  } else {
    count += 1;
  }
  
  delay(1000);
}
