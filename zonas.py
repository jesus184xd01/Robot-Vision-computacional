#!/usr/bin/env python3
# robot_seguimiento_zonas.py

import os
import time
import threading
import json
import numpy as np
import cv2
from pathlib import Path
from http.server import BaseHTTPRequestHandler, HTTPServer

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import hailo
import paho.mqtt.client as mqtt

from hailo_apps.hailo_app_python.apps.detection_simple.detection_pipeline_simple import GStreamerDetectionApp
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer

# ── Configuración ────────────────────────────────────────────
UMBRAL        = 0.35
PERSON_LABEL  = "person"
STREAM_PORT   = 8080
FRAMES_ESPERA = 10

# ── Zonas verticales (normalizadas 0.0 - 1.0) ───────────────
ZONA_IZQ = 0.25   # borde derecho de zona izquierda
ZONA_DER = 0.75   # borde izquierdo de zona derecha

# ── MQTT ─────────────────────────────────────────────────────
BROKER     = "localhost"
PORT       = 1883
TOPIC_CMD  = "robot/control/comando"
TOPIC_DET  = "robot/vision/deteccion"
TOPIC_MODO = "robot/control/modo"
TOPIC_EST  = "robot/estado"
INTERVALO  = 0.3

# ── Estado compartido ────────────────────────────────────────
frame_actual       = None
frame_lock         = threading.Lock()
detecciones_act    = []
ultimo_cmd         = ""
ultimo_envio       = 0.0
modo_auto          = True
frames_sin_persona = 0

# ── MQTT cliente ─────────────────────────────────────────────
cliente_mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("[MQTT] Conectado al broker")
        client.subscribe(TOPIC_MODO)
    else:
        print(f"[MQTT] Error: {rc}")

def on_message(client, userdata, msg):
    global modo_auto
    payload = msg.payload.decode()
    if msg.topic == TOPIC_MODO:
        if payload == "manual":
            modo_auto = False
            print("[MODO] Control cedido al operador — automático pausado")
            cliente_mqtt.publish(TOPIC_EST, "modo_manual")
        elif payload == "auto":
            modo_auto = True
            print("[MODO] Retomando seguimiento automático")
            cliente_mqtt.publish(TOPIC_EST, "modo_auto")

cliente_mqtt.on_connect = on_connect
cliente_mqtt.on_message = on_message
cliente_mqtt.connect(BROKER, PORT)
cliente_mqtt.loop_start()

def enviar_comando(cmd: str):
    global ultimo_cmd, ultimo_envio
    ahora = time.time()
    if cmd == ultimo_cmd and ahora - ultimo_envio < INTERVALO:
        return
    cliente_mqtt.publish(TOPIC_CMD, cmd)
    ultimo_cmd   = cmd
    ultimo_envio = ahora
    print(f"[CMD] → {cmd}")

def zona_dominante(x1, x2):
    """
    Calcula qué porcentaje del bbox cae en cada zona vertical
    y devuelve la zona donde está la mayor parte de la persona.

    Zonas:
      IZQ    : 0.00 - 0.25
      CENTRO : 0.25 - 0.75
      DER    : 0.75 - 1.00
    """
    bw = x2 - x1
    if bw <= 0:
        return "centro", 0.0, 0.0, 0.0

    pct_izq    = max(0, min(x2, ZONA_IZQ) - x1)           / bw
    pct_centro = max(0, min(x2, ZONA_DER) - max(x1, ZONA_IZQ)) / bw
    pct_der    = max(0, x2 - max(x1, ZONA_DER))            / bw

    zona = max(
        ("izq",    pct_izq),
        ("centro", pct_centro),
        ("der",    pct_der),
        key=lambda z: z[1]
    )[0]

    return zona, pct_izq, pct_centro, pct_der

# ── Stream MJPEG ─────────────────────────────────────────────
class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(b"""
            <html><body style='margin:0;background:#000'>
            <img src='/stream' style='width:100%;height:100vh;object-fit:contain'>
            </body></html>""")

        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-type",
                "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with frame_lock:
                        if frame_actual is None:
                            time.sleep(0.05)
                            continue
                        f = frame_actual.copy()

                    h, w = f.shape[:2]

                    # Dibujar zonas verticales
                    iz_px = int(w * ZONA_IZQ)
                    dr_px = int(w * ZONA_DER)
                    cv2.line(f, (iz_px, 0), (iz_px, h), (100, 100, 255), 1)
                    cv2.line(f, (dr_px, 0), (dr_px, h), (100, 100, 255), 1)
                    cv2.putText(f, "IZQ",    (10,       20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100,100,255), 1)
                    cv2.putText(f, "CENTRO", (iz_px+10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100,100,255), 1)
                    cv2.putText(f, "DER",    (dr_px+10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100,100,255), 1)

                    # Dibujar bboxes con porcentajes por zona
                    for det in detecciones_act:
                        x1 = int(det["x1"] * w)
                        y1 = int(det["y1"] * h)
                        x2 = int(det["x2"] * w)
                        y2 = int(det["y2"] * h)
                        cv2.rectangle(f, (x1,y1), (x2,y2), (0,255,100), 2)
                        cv2.putText(f,
                            f"{det['zona'].upper()} {det['pct_zona']:.0%}",
                            (x1, y1-8), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0,255,100), 1)

                    modo_txt   = "AUTO" if modo_auto else "MANUAL"
                    color_modo = (0,255,100) if modo_auto else (0,165,255)
                    cv2.putText(f, f"MODO: {modo_txt}", (10, h-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_modo, 2)

                    _, jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    self.wfile.write(b"--frame\r\n")
                    self.send_header("Content-type", "image/jpeg")
                    self.send_header("Content-length", len(jpg))
                    self.end_headers()
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b"\r\n")
                    time.sleep(0.033)
            except Exception:
                pass

def iniciar_stream():
    server = HTTPServer(("0.0.0.0", STREAM_PORT), StreamHandler)
    print(f"[STREAM] http://localhost:{STREAM_PORT}")
    server.serve_forever()

threading.Thread(target=iniciar_stream, daemon=True).start()

# ── Clase callback ───────────────────────────────────────────
class RobotCallbackData(app_callback_class):
    def _init_(self):
        super()._init_()

# ── Distancias ───────────────────────────────────────────────
DISTANCIA_CERCA = 0.72

def app_callback(pad, info, user_data):
    global detecciones_act, frame_actual, frames_sin_persona

    user_data.increment()

    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK

    # Capturar frame
    format, w, h = get_caps_from_pad(pad)
    if format is not None and w is not None and h is not None:
        frame = get_numpy_from_buffer(buffer, format, w, h)
        bgr   = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # ── CLAHE — mejora imagen con poca luz ──────
        lab     = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe   = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        l       = clahe.apply(l)
        lab     = cv2.merge((l, a, b))
        bgr     = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        with frame_lock:
            frame_actual = bgr

    # Si está en modo manual no procesar detecciones
    if not modo_auto:
        return Gst.PadProbeReturn.OK

    # Detecciones
    roi  = hailo.get_roi_from_buffer(buffer)
    dets = roi.get_objects_typed(hailo.HAILO_DETECTION)

    personas    = []
    dets_render = []

    for det in dets:
        if det.get_label() == PERSON_LABEL and det.get_confidence() >= UMBRAL:
            bbox   = det.get_bbox()
            x1     = bbox.xmin()
            x2     = bbox.xmin() + bbox.width()
            bbox_h = bbox.height()

            # ── Zona dominante ───────────────────────
            zona, pct_izq, pct_centro, pct_der = zona_dominante(x1, x2)
            pct_zona = {"izq": pct_izq, "centro": pct_centro, "der": pct_der}[zona]

            personas.append({
                "confianza": det.get_confidence(),
                "zona":      zona,
                "pct_zona":  pct_zona,
                "bbox_h":    bbox_h,
            })
            dets_render.append({
                "x1":        x1,
                "y1":        bbox.ymin(),
                "x2":        x2,
                "y2":        bbox.ymin() + bbox_h,
                "confianza": det.get_confidence(),
                "zona":      zona,
                "pct_zona":  pct_zona,
                "bbox_h":    bbox_h,
            })

    detecciones_act = dets_render

    if personas:
        # Resetear contador al detectar persona
        frames_sin_persona = 0

        persona  = max(personas, key=lambda p: p["confianza"])
        zona     = persona["zona"]
        pct_zona = persona["pct_zona"]
        conf     = persona["confianza"]
        bbox_h   = persona["bbox_h"]

        if bbox_h > DISTANCIA_CERCA:
            cmd = "persona_cerca"
        elif zona == "izq":
            cmd = "persona_izquierda"
        elif zona == "der":
            cmd = "persona_derecha"
        else:
            cmd = "persona_frente"

        print(f"[VISION] frame:{user_data.get_count()} zona:{zona}({pct_zona:.0%}) h:{bbox_h:.2f} conf:{conf:.0%} → {cmd}")
        cliente_mqtt.publish(TOPIC_DET, json.dumps({
            "objeto":    "persona",
            "confianza": round(conf,     2),
            "zona":      zona,
            "pct_zona":  round(pct_zona, 2),
            "bbox_h":    round(bbox_h,   2),
            "centro_x":  round((x1 + x2) / 2, 2),
            "comando":   cmd,
        }))
        enviar_comando(cmd)

    else:
        # Buffer de frames — espera antes de mandar stop
        frames_sin_persona += 1
        print(f"[VISION] frame:{user_data.get_count()} — sin persona ({frames_sin_persona}/{FRAMES_ESPERA})")
        if frames_sin_persona >= FRAMES_ESPERA:
            enviar_comando("sin_persona")

    return Gst.PadProbeReturn.OK

# ── Entry point ──────────────────────────────────────────────
if __name__ == "__main__":
    project_root = Path(__file__).resolve().parent.parent
    env_file     = project_root / ".env"
    os.environ["HAILO_ENV_FILE"] = str(env_file)

    print("=" * 50)
    print("  Robot Mecanum — Seguimiento por zonas")
    print(f"  Modelo  : yolov8s (detección, Hailo-8L)")
    print(f"  Umbral  : {UMBRAL} (optimizado poca luz)")
    print(f"  Zonas   : IZQ <{ZONA_IZQ} | CENTRO | DER >{ZONA_DER}")
    print(f"  Stream  : http://localhost:{STREAM_PORT}")
    print("  Ctrl+C   para salir")
    print("=" * 50)

    user_data = RobotCallbackData()

    try:
        app = GStreamerDetectionApp(app_callback, user_data)
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        enviar_comando("stop")
        cliente_mqtt.loop_stop()
        cliente_mqtt.disconnect()
        print("[INFO] Robot detenido.")
