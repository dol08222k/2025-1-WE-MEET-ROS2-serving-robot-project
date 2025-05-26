import firebase_admin
from firebase_admin import credentials, db
import re
import subprocess

# 🔐 Firebase 서비스 계정 키 불러오기
cred = credentials.Certificate("esp32-server-cbeda-firebase-adminsdk-fbsvc-25da53f276.json")

# 🔗 Firebase Realtime Database URL
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://esp32-server-cbeda-default-rtdb.firebaseio.com/'
})

def extract_tunnel_url(output):
    match = re.search(r'https://[a-zA-Z0-9\-]+\.trycloudflare\.com', output)
    return match.group(0) if match else None

# 🚀 cloudflared 실행
process = subprocess.Popen(
    ["cloudflared", "tunnel", "--url", "http://localhost:5000"],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

# 📡 URL 추출 후 Firebase에 등록
for line in process.stdout:
    print(line.strip())
    url = extract_tunnel_url(line)
    if url:
        print(f"🔗 터널 URL: {url}")
        db.reference("esp32/tunnel_url").set(url)