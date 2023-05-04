/********************************************************************************
 Code for Waitron Robot Obstacle Detection
 Written by: Vignesh Poojary and Team Technocrats,VESIT(2023)
     
 Note: 
  >Microcontroller: ESP32/ESP8266

*********************************************************************************/

#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
//**********************************Wifi Setup****************************************//
const char* ssid     = "Rohan";
const char* password = "password";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

//**********************************Web page****************************************//
const char* htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html>

<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no" />
  <style>
    body {
      background-color: white;
    }

    header {
      width: 100%;
      overflow: hidden;
      display: flex;
      align-items: center;
      justify-content: center;
      background-color: black;
      color: #ececec;
      border-radius: 50px
      /* z-index: 90; */
    }

    .tab {
      display: flex;
      align-items: center;
      justify-content: center;
    }

    .tab button {
      box-shadow: 0 8px 16px 0 rgba(255, 255, 255, 0.5),
        0 6px 20px 0 rgba(70, 70, 70, 0.15);
      background-color: #ececec;
      color: black;
      float: left;
      cursor: pointer;
      border-radius: 50px;
      border: solid 8px #ececec;
      outline: none;
      display: flex;
      justify-content: center;
      align-items: center;
      font-weight: 700;
      font-size: 16px;
      padding: 10px 10px;
      transition: 0.3s;
    }

    #M-ctrl {
      margin-right: 10px;
    }

    .tabcontent {
      display: none;
      color: rgba(0, 0, 0, 0.);
      text-align: center;
      color: #2673dd;
    }

    .tab button.active {
      color: #000000;
      box-shadow: -10px -10px 15px rgba(255, 255, 255, 0.5),
        10px 10px 15px rgba(70, 70, 70, 0.15),
        inset -10px -10px 15px rgba(255, 255, 255, 0.5),
        inset 10px 10px 15px rgba(70, 70, 70, 0.15);
    }

    label.control {
      color: #ffffff;
      margin-right: 8px;
      background: #034684;
      border-style: solid;
      border-color: #120979;
    }

    label.table {
      float: left;
      color: #ffffff;
      background: #034684;
      border-style: solid;
      /* border-color: #120979; */

    }

    label.control:active {
      transform: translate(5px, 5px);
      box-shadow: none;
    }

    label.table:active {
      transform: translate(5px, 5px);
      box-shadow: none;
    }

    .arrows {
      font-size: 70px;
      display: flex;
      align-items: center;
      justify-content: center;
    }

    .blank {
      text-decoration: none;
      border: none;
      box-shadow: none;
      background: none;
    }

    
    td.blank:active {
      box-shadow: none;
    }

    td {
      background: #ececec;
      border-radius: 25%;
      justify-content: center;
      align-items: center;
      border: solid 8px #ececec;
      transition: 0.08s;
      /* width: 60px; */
      /* height: 60px; */
      /* margin: 10px; */
      /* font-size: 20px; */
      cursor: pointer;
      
    }

    
    td:active {
      transform: translate(5px, 5px);
      color: #44cc77;
      box-shadow: -10px -10px 15px rgba(255, 255, 255, 0.5),
        10px 10px 15px rgba(70, 70, 70, 0.15),
        inset -10px -10px 15px rgba(255, 255, 255, 0.5),
        inset 10px 10px 15px rgba(70, 70, 70, 0.15);
    }


    /* dark mode */
    label {
      position: absolute;
      width: 45px;
      height: 22px;
      right: 20px;
      top: 35px;
      border: 2px solid;
      border-radius: 20px;
    }

    label:before {
      position: absolute;
      content: '';
      width: 20px;
      height: 20px;
      left: 1px;
      top: 1px;
      border-radius: 50%;
      background: #000;
      cursor: pointer;
      transition: 0.4s;
    }

    label.active:before {
      left: 24px;
      background: #fff;
    }

    body.night {
      background: #000;
      color: #fff;
  
    }

    .new{
      
    }
  

  </style>
</head>

<body>
  <header>
    <!-- <h1 style=" text-align: center">Technocrats</h1> -->
    <h1 style="text-align: center;">WAITRON</h2>
      <label id="dark-change"></label>

  </header>


  <!-- Tab links -->
  <div class="tab">
    <button style="color: #EE2C4A;" class="tablinks" id="M-ctrl" onclick="openTab(event, 'Manual Control')">
      Manual Control
    </button>

    <button  style="color: #EE2C4A;" class="tablinks" onclick="openTab(event, 'Select the Table Number')">
      Table Number
    </button>

  </div>

  <!-- Tab content -->
  <div class="screen">

    <div id="Manual Control" class="tabcontent">
      <h3 style="font-size: 25px;">Manual Control</h3>
      <table id="mainTable" style="width: 350px; margin: auto; table-layout: fixed" cellspacing="10">
        <tr>
          <td class="blank"></td>
          <td ontouchstart='onTouchStartAndEnd("w")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows">&#8679;</span>
          </td>
          <td class="blank"></td>
        </tr>

        <tr>
          <td ontouchstart='onTouchStartAndEnd("a")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows">&#8678;</span>
          </td>
          <td ontouchstart='onTouchStartAndEnd("v")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows" style="
                    font-size: 45px;
                    color: #EE2C4A;
                    display: flex;
                    align-items: center;
                    justify-content: center;
                  ">&#10006;</span>
          </td>
          <td ontouchstart='onTouchStartAndEnd("d")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows">&#8680;</span>
          </td>
        </tr>

        <tr>
          <td class="blank"></td>
          <td ontouchstart='onTouchStartAndEnd("s")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows">&#8681;</span>
          </td>
          <td class="blank"></td>
        </tr>
      </table>
    </div>

    <div id="Select the Table Number" class="tabcontent">
      <h3 style="font-size: 25px;">Select the Table Number</h3>
      <table id="mainTable" style="width: 350px; margin: auto; table-layout: fixed" cellspacing="10">
        <tr>
          <td ontouchstart='onTouchStartAndEnd("v")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows">5</span>
          </td>

          <td class="blank"></td>

          <td ontouchstart='onTouchStartAndEnd("i")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows">6</span>
          </td>
        </tr>
        
        <tr>
          <td ontouchstart='onTouchStartAndEnd("u")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows">3</span>
          </td>
          
          <td class="blank"></td>
          
          <td ontouchstart='onTouchStartAndEnd("a")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows">4</span>
          </td>
        </tr>

        <tr>
          <td class="Table1" ontouchstart='onTouchStartAndEnd("t")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows">1</span>
          </td>

          <td class="blank"></td>
          
          <td ontouchstart='onTouchStartAndEnd("y")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows">2</span>
          </td>
        </tr>
        
        <tr>
          <td style="color: #EE2C4A; font-size: 10px;"  ontouchstart='onTouchStartAndEnd("r")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows">R</span>
          </td>
          

          <td ontouchstart='onTouchStartAndEnd("k")' ontouchend='onTouchStartAndEnd("v")'>
            <span class="arrows">K</span>
          </td>

          <td style="color: #EE2C4A;"  ontouchstart='onTouchStartAndEnd("z")' ontouchend='onTouchStartAndEnd("0")'>
            <span class="arrows">Z</span>
          </td>        
        </tr>
      </table>
    </div>
  </div>
  </div>
  </div>

  <script>
    function openTab(evt, tab) {
      var i, tabcontent, tablinks;
      tabcontent = document.getElementsByClassName("tabcontent");
      for (i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = "none";
      }
      tablinks = document.getElementsByClassName("tablinks");
      for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" active", "");
      }
      document.getElementById(tab).style.display = "block";
      evt.currentTarget.className += " active";
    }

    // orginal js

    var webSocketUrl = "ws:\/\/" + window.location.hostname + "/ws";
    var websocket;

    function initWebSocket() {
      websocket = new WebSocket(webSocketUrl);
      websocket.onopen = function (event) { };
      websocket.onclose = function (event) {
        setTimeout(initWebSocket, 2000);
      };
      websocket.onmessage = function (event) { };
    }
    function onTouchStartAndEnd(value) {
      websocket.send(value);
    }

    window.onload = initWebSocket;
    document
      .getElementById("mainTable")
      .addEventListener("touchend", function (event) {
        event.preventDefault();
      });

    //Dark mode 

    var content = document.getElementsByTagName('body')[0];
    var darkMode = document.getElementById('dark-change');
    darkMode.addEventListener('click', function () {
      darkMode.classList.toggle('active');
      content.classList.toggle('night');
    })


  </script>

</body>
</html>
)HTMLHOMEPAGE";

//**********************************Web page****************************************//
void processCarMovement(String inputValue)
{
  Serial.println(inputValue.c_str()); 
}

void handleRoot(AsyncWebServerRequest *request) 
{
  request->send_P(200, "text/html", htmlHomePage);
}

void handleNotFound(AsyncWebServerRequest *request) 
{
    request->send(404, "text/plain", "File Not Found");
}


void onWebSocketEvent(AsyncWebSocket *server, 
                      AsyncWebSocketClient *client, 
                      AwsEventType type,
                      void *arg, 
                      uint8_t *data, 
                      size_t len) 
{                      
  switch (type) 
  {
    case WS_EVT_CONNECT:
      //Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      //client->text(getRelayPinsStatusJson(ALL_RELAY_PINS_INDEX));
      break;
    case WS_EVT_DISCONNECT:
      //Serial.printf("WebSocket client #%u disconnected\n", client->id());
      processCarMovement("0");
      break;
    case WS_EVT_DATA:
      AwsFrameInfo *info;
      info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) 
      {
        std::string myData = "";
        myData.assign((char *)data, len);
        processCarMovement(myData.c_str());       
      }
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
    default:
      break;  
  }
}

//**********************************Main Code****************************************//
void setup(void) 
{
  Serial.begin(9600);

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  //Serial.print("AP IP address: ");
  //Serial.println(IP);

  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);
  
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  
  server.begin();
  //Serial.println("HTTP server started");
}

void loop() 
{
  ws.cleanupClients(); 
}