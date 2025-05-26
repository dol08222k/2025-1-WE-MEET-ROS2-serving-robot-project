from flask import Flask, request
import paho.mqtt.client as mqtt
import json

app = Flask(__name__)

# MQTT 설정
MQTT_BROKER = 'localhost'         # 또는 외부 브로커 주소
MQTT_PORT = 1883
MQTT_TOPIC = '/table'

# MQTT 클라이언트 초기화
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.loop_start()

@app.route('/')
def index():
    return '정상 실행중'

@app.route('/table', methods=['POST'])
def esp_data():
    data = request.json
    print(f"ESP32 sent: {data}")

    # MQTT로 전송
    try:
        mqtt_payload = data.get("value", "unknown")
        result = mqtt_client.publish(MQTT_TOPIC, mqtt_payload)
        status = result[0]
        if status == 0:
            print(f"✅ MQTT Published to {MQTT_TOPIC}: {mqtt_payload}")
        else:
            print(f"❌ MQTT Publish failed with code {status}")
    except Exception as e:
        print(f"❌ MQTT Publish failed: {e}")

    return 'OK', 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
