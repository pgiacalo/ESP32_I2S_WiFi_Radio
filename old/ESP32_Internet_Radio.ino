#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"

#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26

#define BOARD_NAME  "esp32:esp32:uPesy_wroom"  //for use with arduino-cli upload -b 
#define PORT        "/dev/tty.usbserial-0001"  //for use with arduino-cli upload -p 

Audio audio;

String ssid =    "Aardvark";
String password = "125125125";

void setup() {

  Serial.begin(115200);

  WiFi.disconnect();

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid.c_str(), password.c_str());

  while (WiFi.status() != WL_CONNECTED)

  delay(1500);

  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);

  audio.setVolume(100);  //percent

  connect(&audio, 3);
//  audio.connecttohost("http://vis.media-ice.musicradio.com/CapitalMP3");
//  audio.connecttohost("http://rfcmedia2.streamguys1.com/thirdrock.mp3");
//  audio.connecttohost("http://stream.radioparadise.com/global-192");

}

void loop()

{

  audio.loop();

}

void connect(Audio* audio, int channel){
  //  *** radio streams ***

  switch (channel){
    case 1:
    (*audio).connecttohost("http://rfcmedia2.streamguys1.com/thirdrock.mp3"); // aac
    break;
  
    case 2: //problems with this channel
    (*audio).connecttohost("http://stream.radioparadise.com/global-192");                                         // mp3
    break;

    case 3:
    (*audio).connecttohost("http://vis.media-ice.musicradio.com/CapitalMP3");   //horrible music!
    break;
  
    case 4:
    (*audio).connecttohost("http://www.wdr.de/wdrlive/media/einslive.m3u");                                // m3u
    break;
  
    case 5:
    (*audio).connecttohost("https://stream.srg-ssr.ch/rsp/aacp_48.asx");                                   // asx
    break;
  
    case 6:
    (*audio).connecttohost("http://tuner.classical102.com/listen.pls");                                    // pls
    break;
  
    case 7:
    (*audio).connecttohost("http://stream.radioparadise.com/flac");                                        // flac
    break;
  
    case 8:
    (*audio).connecttohost("http://stream.sing-sing-bis.org:8000/singsingFlac");                           // flac (ogg)
    break;
  
    case 9:
    (*audio).connecttohost("http://s1.knixx.fm:5347/dein_webradio_vbr.opus");                              // opus (ogg)
    break;
  
    case 10:
    (*audio).connecttohost("http://stream2.dancewave.online:8080/dance.ogg");                              // vorbis (ogg)
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