#include "Wifi.hpp"
#include "Sensors.hpp"
#include <WiFi.h>
#include <WebServer.h>

static WebServer server(80);

RobotWifi::RobotWifi(bool& running, int& baseSpeed, int& regSpeed,
                     float& kp, float& ki, float& kd,
                     Sensor& sensor, Logger& logger)
    : running(running), baseSpeed(baseSpeed), regSpeed(regSpeed),
      kp(kp), ki(ki), kd(kd), sensor(sensor), logger(logger) {}

void RobotWifi::begin() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("LinjefølgerG11", "12345678");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    setupRoutes();
    server.begin();
}

void RobotWifi::handle() {
    server.handleClient();
}

String RobotWifi::buildPage() {
    String p = "<!doctype html><html><head>";
    p += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    p += "<title>Linjefolger</title></head>";
    p += "<body style='font-family:sans-serif;max-width:480px;margin:20px'>";
    p += "<h2>Linjefolger</h2>";

    p += "<p>Status: <b id='status'>" + String(running ? "KJORER" : "STOPPET") + "</b></p>";
    p += "<p>Kalibrering: <b id='kal'>" + String(calibrating ? "PAGAR..." : (sensor.isCalibrated() ? "FERDIG" : "IKKE KALIBRERT")) + "</b></p>";

    p += "<p>";
    p += "<button onclick='fetch(\"/start\")' style='padding:10px 20px;font-size:16px;margin:4px'>START</button> ";
    p += "<button onclick='fetch(\"/stopp\")' style='padding:10px 20px;font-size:16px;margin:4px'>STOPP</button> ";
    p += "<button onclick='fetch(\"/kalibrer\")' style='padding:10px 20px;font-size:16px;margin:4px'>KALIBRER</button>";
    p += "</p><hr>";

    // ========== DATALOGGER ==========
    p += "<h3>Datalogger</h3>";
    p += "<p>Samples: <b id='logcount'>" + String(logger.count()) + "/" + String(logger.capacity()) + "</b>";
    p += " | <b id='logstate'>" + String(logger.isLogging() ? "LOGGER" : "STOPPET") + "</b></p>";
    p += "<p>";
    p += "<button onclick='fetch(\"/log/start\")' style='padding:10px 20px;font-size:16px;margin:4px'>Start logging</button> ";
    p += "<button onclick='fetch(\"/log/stop\")' style='padding:10px 20px;font-size:16px;margin:4px'>Stopp logging</button> ";
    p += "<button onclick='fetch(\"/log/reset\")' style='padding:10px 20px;font-size:16px;margin:4px'>Nullstill</button><br>";
    p += "<a href='/log/download' download='robotlog.csv'><button style='padding:12px 24px;font-size:16px;margin:4px;background:#4caf50;color:white;border:none;border-radius:4px;cursor:pointer'>LAST NED CSV</button></a>";
    p += "</p><hr>";

    p += "<h3>Basehastighet (0-255)</h3>";
    p += "<form action='/set'><input name='base' type='number' min='0' max='255' value='" + String(baseSpeed) + "'> ";
    p += "<input type='submit' value='Set'></form>";

    p += "<h3>Reguleringshastighet (0-255)</h3>";
    p += "<form action='/set'><input name='reg' type='number' min='0' max='255' value='" + String(regSpeed) + "'> ";
    p += "<input type='submit' value='Set'></form>";

    p += "<hr><h3>PID</h3>";
    p += "<form action='/set'>";
    p += "Kp: <input name='kp' type='number' step='0.0001' value='" + String(kp, 5) + "'><br><br>";
    p += "Ki: <input name='ki' type='number' step='0.00001' value='" + String(ki, 5) + "'><br><br>";
    p += "Kd: <input name='kd' type='number' step='0.00001' value='" + String(kd, 5) + "'><br><br>";
    p += "<input type='submit' value='Oppdater PID'></form>";

    p += "<script>"
         "async function poll(){"
         "  try{"
         "    const r=await fetch('/status');"
         "    const s=await r.json();"
         "    document.getElementById('status').textContent=s.running?'KJORER':'STOPPET';"
         "    document.getElementById('kal').textContent=s.calibrating?'PAGAR...':s.calibrated?'FERDIG':'IKKE KALIBRERT';"
         "    document.getElementById('logcount').textContent=s.logcount+'/'+s.logcap;"
         "    document.getElementById('logstate').textContent=s.logging?'LOGGER':'STOPPET';"
         "  }catch(e){}"
         "}"
         "setInterval(poll,500);poll();"
         "</script>";

    p += "</body></html>";
    return p;
}

void RobotWifi::setupRoutes() {

    server.on("/status", HTTP_GET, [this]() {
        String json = "{\"running\":" + String(running ? "true" : "false") + ",";
        json += "\"calibrating\":" + String(calibrating ? "true" : "false") + ",";
        json += "\"calibrated\":" + String(sensor.isCalibrated() ? "true" : "false") + ",";
        json += "\"logging\":" + String(logger.isLogging() ? "true" : "false") + ",";
        json += "\"logcount\":" + String(logger.count()) + ",";
        json += "\"logcap\":" + String(logger.capacity()) + "}";
        server.send(200, "application/json", json);
    });

    server.on("/", HTTP_GET, [this]() {
        server.send(200, "text/html", buildPage());
    });

    server.on("/start", HTTP_GET, [this]() {
        running = true;
        Serial.println("START");
        server.send(200, "text/plain", "OK");
    });

    server.on("/stopp", HTTP_GET, [this]() {
        running = false;
        Serial.println("STOPP");
        server.send(200, "text/plain", "OK");
    });

    server.on("/kalibrer", HTTP_GET, [this]() {
        running = false;
        calibrating = true;
        server.send(200, "text/plain", "Kalibrerer...");
        sensor.calibrate(5000);
        calibrating = false;
    });

    server.on("/set", HTTP_GET, [this]() {
        if (server.hasArg("base")) baseSpeed = constrain(server.arg("base").toInt(), 0, 255);
        if (server.hasArg("reg"))  regSpeed  = constrain(server.arg("reg").toInt(),  0, 255);
        if (server.hasArg("kp"))   kp = server.arg("kp").toFloat();
        if (server.hasArg("ki"))   ki = server.arg("ki").toFloat();
        if (server.hasArg("kd"))   kd = server.arg("kd").toFloat();

        Serial.printf("Base:%d Reg:%d Kp:%.5f Ki:%.5f Kd:%.5f\n",
                      baseSpeed, regSpeed, kp, ki, kd);

        server.sendHeader("Location", "/");
        server.send(302);
    });

    // ========== LOGGER-ENDEPUNKTER ==========

    server.on("/log/start", HTTP_GET, [this]() {
        logger.start();
        Serial.println("Logger startet");
        server.send(200, "text/plain", "Logger startet");
    });

    server.on("/log/stop", HTTP_GET, [this]() {
        logger.stop();
        Serial.println("Logger stoppet");
        server.send(200, "text/plain", "Logger stoppet");
    });

    server.on("/log/reset", HTTP_GET, [this]() {
        logger.reset();
        Serial.println("Logger nullstilt");
        server.send(200, "text/plain", "Logger nullstilt");
    });

    server.on("/log/download", HTTP_GET, [this]() {
        logger.stop();

        server.sendHeader("Content-Disposition", "attachment; filename=robotlog.csv");
        server.setContentLength(CONTENT_LENGTH_UNKNOWN);
        server.send(200, "text/csv", "");
        server.sendContent("time_s,error,correction\n");

        char line[48];
        uint16_t total = logger.count();
        for (uint16_t i = 0; i < total; i++) {
            const Logger::Sample& s = logger.at(i);
            float t_s = s.t_ms / 1000.0f;
            snprintf(line, sizeof(line), "%.3f,%d,%d\n", t_s, s.error, s.correction);
            server.sendContent(line);
            if ((i & 0x1F) == 0) yield();
        }
        server.client().stop();
        Serial.printf("CSV lastet ned (%u samples)\n", total);
    });
}