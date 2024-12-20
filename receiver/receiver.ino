/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 02/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a minimum setup LoRa test receiver. The program listens for incoming packets
  using the frequency and LoRa settings in the LT.setupLoRa() command. The pins to access the lora device
  need to be defined at the top of the program also.

  There is a printout on the Arduino IDE serial monitor of the valid packets received, the packet is assumed
  to be in ASCII printable text, if it's not ASCII text characters from 0x20 to 0x7F, expect weird things to
  happen on the Serial Monitor. Sample serial monitor output;

  8s  Hello World 1234567890*,RSSI,-44dBm,SNR,9dB,Length,23,Packets,7,Errors,0,IRQreg,50

  If there is a packet error it might look like this, which is showing a CRC error;

  137s PacketError,RSSI,-89dBm,SNR,-8dB,Length,23,Packets,37,Errors,2,IRQreg,70,IRQ_HEADER_VALID,IRQ_CRC_ERROR,IRQ_RX_DONE

  If there are no packets received in a 10 second period then you should see a message like this;

  112s RXTimeout

  For an example of a more detailed configuration for a receiver, see program 104_LoRa_Receiver.

  Serial monitor baud rate is set at 9600.
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
#define RXBUFFER_SIZE 64                        //RX buffer size

uint32_t RXpacketCount;
uint32_t errors;

uint8_t RXBUFFER[RXBUFFER_SIZE];                //create the buffer that received packets are copied into

uint8_t RXPacketL;                              //stores length of packet received
int8_t  PacketRSSI;                             //stores RSSI of received packet
int8_t  PacketSNR;                              //stores signal to noise ratio (SNR) of received packet

// ============================================================================
//  Private Functions
// ============================================================================

void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  display.print(seconds, 0);
  display.print(F("s "));
}

void packet_is_OK()
{
  uint16_t IRQStatus;

  RXpacketCount++;
  IRQStatus = LT.readIrqStatus();                  //read the LoRa device IRQ status register
  // printElapsedTime();                              //print elapsed time to Serial Monitor

  RXBUFFER[RXPacketL] = '\0';
  display.println( (char*)RXBUFFER );

  /*Serial.print(F("  "));
  LT.printASCIIPacket(RXBUFFER, RXPacketL);        //print the packet as ASCII characters

  Serial.print(F(",RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Length,"));
  Serial.print(RXPacketL);
  Serial.print(F(",Packets,"));
  Serial.print(RXpacketCount);
  Serial.print(F(",Errors,"));
  Serial.print(errors);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);*/
}

void packet_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                   //read the LoRa device IRQ status register

  printElapsedTime();                               //print elapsed time to Serial Monitor

  if (IRQStatus & IRQ_RX_TIMEOUT)                   //check for an RX timeout
  {
    Serial.print(F(" RXTimeout"));
  }
  else
  {
    errors++;
    Serial.print(F(" PacketError"));
    Serial.print(F(",RSSI,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm,SNR,"));
    Serial.print(PacketSNR);
    Serial.print(F("dB,Length,"));
    Serial.print(LT.readRXPacketL());               //get the real packet length
    Serial.print(F(",Packets,"));
    Serial.print(RXpacketCount);
    Serial.print(F(",Errors,"));
    Serial.print(errors);
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);
    LT.printIrqStatus();                            //print the names of the IRQ registers set
  }
}

// ============================================================================
//  Setup and Loop
// ============================================================================

void setup() {
  heltec_setup();
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("4_LoRa_Receiver Starting"));
  Serial.println();

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

  LT.setupLoRa(433000000, 0, LORA_SF7, LORA_BW_125, LORA_CR_4_5, LDRO_AUTO);   //configure frequency and LoRa settings

  Serial.print(F("Receiver ready - RXBUFFER_SIZE "));
  Serial.println(RXBUFFER_SIZE);
  Serial.println();

  display.clear();
}


void loop()
{
  display.print("recv: ");
  RXPacketL = LT.receive(RXBUFFER, RXBUFFER_SIZE, 60000, WAIT_RX); //wait for a packet to arrive with 60seconds (60000mS) timeout

  PacketRSSI = LT.readPacketRSSI();              //read the received packets RSSI value
  PacketSNR = LT.readPacketSNR();                //read the received packets SNR value

  if (RXPacketL == 0)                            //if the LT.receive() function detects an error RXpacketL is 0
  {
    packet_is_Error();
    display.println("error");
  }
  else
  {
    packet_is_OK();
  }

  Serial.println();
}
