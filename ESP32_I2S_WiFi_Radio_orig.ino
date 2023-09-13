/**
 * This software is written for the ESP32 microcontroller. It follows Arduino coding conventions 
 * with functions setup() and loop(). 
 * This software creates an audio player that gets streaming audio data from internet URLs 
 * using a connection to a WiFi network. 
 * Your WiFi network's ssid and password must be configured in the code, before compiling and 
 * uploading the code to your ESP32. 
 * 
 * Notes about the starup process and the LED indicators:
 * 1) At startup, the ESP32 LED blinks once per second while attempting to connect to WiFi.
 * 2) If WiFi connectivity succeeds, the blinking LED will turn off. 
 * 3) If WiFi connectivity fails, the LED will blink S.O.S. continuously to indicate a startup failure. 
 *      - Double check the values of the WiFi ssid and password in the code, if WiFi connectivity fails. 
 * 
 * Messages are printed to the Serial Monitor to show startup progress or failures.
 *
 * This youtube video explains the ESP32 wiring, etc.
 * https://www.youtube.com/watch?v=t4K1HBQUj-k
 */   

///////////////////////////////////////////////////////////////////////////
// Special thanks to Wolle schreibfaul1 (https://github.com/schreibfaul1)
//
// Requires the following ESP32 I2S audio library from github
// https://github.com/schreibfaul1/ESP32-audioI2S
//
// The associated wiki page for this library is found here:
// https://github.com/schreibfaul1/ESP32-audioI2S/wiki
///////////////////////////////////////////////////////////////////////////


// arduino-cli upload -b esp32:esp32:uPesy_wroom -p /dev/tty.usbserial-0001 
//#define FQBN        "esp32:esp32:uPesy_wroom"  //Fully Qualified Board Name (fqbn) for use with "arduino-cli upload -b" 
//#define PORT        "/dev/tty.usbserial-0001"  //the USB port connected to the ESP32. for use with "arduino-cli upload -p" 


#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"

//ESP32 I2S output pins
#define I2S_DOUT      25  //GPIO 25 (DATA - the digital output. connects to DIN pin on MAX98357A I2S amplifier)
#define I2S_LRC       26  //GPIO 26 (SELECT - left/right control. connects to LRC pin on MAX98357A I2S amplifier)
#define I2S_BCLK      27  //GPIO 27 (CLOCK - serial clock. connects to BCLK pin on MAX98357A I2S amplifier)

//NOTES on MAX98357A wiring connections to setup the left and right channel outputs:
//  Right Channel - connect the GAIN pin to VIN pin on the MAX98357A
//  Left Channel  - connect the GAIN pin to GND pin on the MAX98357A

//ESP32 analog input pin -- used for volume control
#define POT_PIN       34  //GPIO 34   //reads the potentiometer voltage (controls audio volume)
#define LED_PIN       2   //GPIO 2    //used by the code to control the ESP32's blue LED 

#define VOLUME_CONTROL_STEPS  100     //100 steps -- the potentiometer (on GPIO34) controls audio volume between zero and 100%
#define WIFI_MAX_TRIES        10      //number of attempts to connect to WiFi during startup

//WiFi account login
const String ssid         = "Aardvark";   //wifi network name 
const String password     = "125125125";   //wifi password

Audio audio;  //class from the ESP32-audioI2S library

void setupAudio(){
  Serial.println("Setting I2S output pins and volume level.");

  //configure the I2S output pins
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);

  //set the initial volume level
  audio.setVolumeSteps(VOLUME_CONTROL_STEPS);
  int volume = map(analogRead(POT_PIN), 0, 4095, 0, VOLUME_CONTROL_STEPS);  // map potentiometer value to a volume percentage
  audio.setVolume(volume);
  Serial.print("Volume set at ");
  Serial.print(volume);
  Serial.println("%");
}

void connectWiFi(){
  WiFi.disconnect();

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid.c_str(), password.c_str());

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < WIFI_MAX_TRIES){
      tries++;
      Serial.print("Attempting to connect to WiFi. Try #");
      Serial.println(tries);
      //blink the LED while we are trying to connect
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
  }

  if (WiFi.status() == WL_CONNECTED){
    //WiFi connection succeeded. Turn the LED OFF. 
    digitalWrite(LED_PIN, LOW);
    Serial.print("SUCCESS: connected to wifi network ");
    Serial.println(ssid);
  } else {
    //WiFi connection FAILED. Blink S.O.S. with the LED.  
    while(true){
      Serial.print("WiFi error: Failed to connect to ");
      Serial.println(ssid);
      Serial.println("Check the code for the WiFi ssid and password.");
      blinkSOS(5);
    }
  }

}

void connect(Audio *audio, int channel) {
  switch (channel){
  //  *** radio streams ***
    case 1:
    (*audio).connecttohost("http://rfcmedia2.streamguys1.com/thirdrock.mp3");   // aac
    break;
  
    case 2: //problems with this channel
    (*audio).connecttohost("http://stream.radioparadise.com/global-192");       // mp3
    break;

    case 3:
    (*audio).connecttohost("http://vis.media-ice.musicradio.com/CapitalMP3");   //horrible music!
    break;
  
    case 4:
    (*audio).connecttohost("http://www.wdr.de/wdrlive/media/einslive.m3u");     // m3u
    break;
  
    case 5:
    (*audio).connecttohost("https://stream.srg-ssr.ch/rsp/aacp_48.asx");        // asx
    break;
  
    case 6:
    (*audio).connecttohost("http://tuner.classical102.com/listen.pls");         // pls
    break;
  
    case 7:
    (*audio).connecttohost("http://stream.radioparadise.com/flac");             // flac
    break;
  
    case 8:
    (*audio).connecttohost("http://stream.sing-sing-bis.org:8000/singsingFlac");  // flac (ogg)
    break;
  
    case 9:
    (*audio).connecttohost("http://s1.knixx.fm:5347/dein_webradio_vbr.opus");   // opus (ogg)
    break;
  
    case 10:
    (*audio).connecttohost("http://stream2.dancewave.online:8080/dance.ogg");   // vorbis (ogg)
    break;
  
    case 11:
    (*audio).connecttohost("http://26373.live.streamtheworld.com:3690/XHQQ_FMAAC/HLSTS/playlist.m3u8");    // HLS
    break;
  
    case 12:
    (*audio).connecttohost("http://eldoradolive02.akamaized.net/hls/live/2043453/eldorado/master.m3u8");   // HLS (ts)
    break;

    //*** web files ***
    case 13:
    (*audio).connecttohost("https://github.com/schreibfaul1/ESP32-audioI2S/raw/master/additional_info/Testfiles/Pink-Panther.wav");        // wav
    break;
  
    case 14:
    (*audio).connecttohost("https://github.com/schreibfaul1/ESP32-audioI2S/raw/master/additional_info/Testfiles/Santiano-Wellerman.flac"); // flac
    break;
  
    case 15:
    (*audio).connecttohost("https://github.com/schreibfaul1/ESP32-audioI2S/raw/master/additional_info/Testfiles/Olsen-Banden.mp3");        // mp3
    break;
  
    case 16:
    (*audio).connecttohost("https://github.com/schreibfaul1/ESP32-audioI2S/raw/master/additional_info/Testfiles/Miss-Marple.m4a");         // m4a (aac)
    break;
  
    case 17:
    (*audio).connecttohost("https://github.com/schreibfaul1/ESP32-audioI2S/raw/master/additional_info/Testfiles/Collide.ogg");             // vorbis
    break;
  
    case 18:
    (*audio).connecttohost("https://github.com/schreibfaul1/ESP32-audioI2S/raw/master/additional_info/Testfiles/sample.opus");             // opus
    break;

    //*** local files ***
  
    case 19:
    (*audio).connecttoFS(SD, "/test.wav");     // SD
    break;
  
    case 20:
    (*audio).connecttoFS(SD_MMC, "/test.wav"); // SD_MMC
    break;
  
    case 21:
    (*audio).connecttoFS(SPIFFS, "/test.wav"); // SPIFFS
    break;

    case 22:
    (*audio).connecttospeech("Wenn die Hunde schlafen, kann der Wolf gut Schafe stehlen.", "de"); // Google TTS
    break;
  }

}

void blinkSOS(int repeats){
  int tries = 0;
  while (tries < repeats){

      for (int i=0; i<3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
      }

      for (int i=0; i<3; i++) {
        delay(200);
        digitalWrite(LED_PIN, HIGH);
        delay(800);
        digitalWrite(LED_PIN, LOW);
      }

      for (int i=0; i<3; i++) {
        delay(200);
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
      }
    delay(1000);
    tries++;
  }
}

void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("--------------------------------------------");
  Serial.println("------- Starting ESP32_I2S_WiFi_Radio-------");
  Serial.println("--------------------------------------------");
 
  pinMode(LED_PIN, OUTPUT); //use the LED to indicate WiFi status

  connectWiFi();

  setupAudio();

  int channel = 1;
  Serial.println("Playing audio...");
  connect(&audio, channel);

}

void loop() {
  int volume = map(analogRead(POT_PIN), 0, 4095, 0, VOLUME_CONTROL_STEPS);
  audio.setVolume(volume);

  audio.loop();
}
