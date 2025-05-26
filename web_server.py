from flask import Flask, request, render_template_string, redirect, url_for
import datetime

app = Flask(__name__)

current_orders = []
completed_orders = []

@app.route('/order', methods=['GET', 'POST'])
def order():
    if request.method == 'POST':
        new_order = request.json
        new_order['timestamp'] = datetime.datetime.now().strftime('%H:%M:%S')
        current_orders.append(new_order)
        return 'OK'

    # view 파라미터로 어떤 탭을 보여줄지 결정
    view = request.args.get('view', 'current')  # 기본은 'current'

    html = f"""
    <html>
    <head>
        <title>서빙로봇 주문 내역</title>
        <style>
            body {{
                font-family: 'Segoe UI', sans-serif;
                background-color: #f4f6f8;
                padding: 20px;
            }}
            h1 {{
                color: #333;
            }}
            .tabs {{
                margin-bottom: 20px;
            }}
            button {{
                padding: 8px 16px;
                margin-right: 10px;
                border: none;
                background-color: #007BFF;
                color: white;
                border-radius: 5px;
                cursor: pointer;
                transition: background-color 0.3s;
            }}
            button:hover {{
                background-color: #0056b3;
            }}
            .order-block {{
                background-color: white;
                padding: 16px;
                margin-bottom: 16px;
                border-radius: 8px;
                box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
                opacity: 1;
                transition: opacity 0.5s ease, transform 0.5s ease;
            }}
            .order-block.fade-out {{
                opacity: 0;
                transform: translateX(50px);
            }}
            .timestamp {{
                font-size: 0.9em;
                color: gray;
                margin-bottom: 6px;
            }}
        </style>
        <script>
            function completeOrder(index) {{
                const block = document.getElementById('order-' + index);
                block.classList.add('fade-out');
                setTimeout(() => {{
                    fetch('/complete/' + index, {{ method: 'POST' }})
                        .then(() => window.location.href = '/order?view=current');
                }}, 500);
            }}

            function switchView(view) {{
                window.location.href = '/order?view=' + view;
            }}

            window.onload = function() {{
                setInterval(() => {{
                    const urlParams = new URLSearchParams(window.location.search);
                    const view = urlParams.get('view') || 'current';
                    window.location.href = '/order?view=' + view;
                }}, 5000);
            }};
        </script>
    </head>
    <body>
        <h1>📦 서빙 로봇 주문 관리</h1>
        <div class="tabs">
            <button onclick="switchView('current')">현재 주문</button>
            <button onclick="switchView('completed')">완료된 주문</button>
        </div>

        <div id="current" style="display: {'block' if view == 'current' else 'none'};">
            <h2>🕒 현재 주문</h2>
    """

    for idx, order in enumerate(current_orders):
        html += f"<div class='order-block' id='order-{idx}'>"
        html += f"<div class='timestamp'>{order.get('timestamp', '')} | 테이블 {order.get('table', '알 수 없음')}번</div>"
        html += "<ul>"
        for item, count in order.get('items', {}).items():
            html += f"<li>{item}: {count}개</li>"
        html += f"</ul><button onclick=\"completeOrder({idx})\">✖ 완료</button></div>"

    html += f"""
        </div>
        <div id="completed" style="display: {'block' if view == 'completed' else 'none'};">
            <h2>✅ 완료된 주문</h2>
    """

    for order in completed_orders:
        html += "<div class='order-block'>"
        html += f"<div class='timestamp'>{order.get('timestamp', '')} | 테이블 {order.get('table', '알 수 없음')}번</div>"
        html += "<ul>"
        for item, count in order.get('items', {}).items():
            html += f"<li>{item}: {count}개</li>"
        html += "</ul></div>"

    html += "</div></body></html>"

    return render_template_string(html)

@app.route('/complete/<int:index>', methods=['POST'])
def complete(index):
    try:
        completed_orders.append(current_orders.pop(index))
    except IndexError:
        pass
    return ('', 204)

@app.route('/')
def home():
    return redirect(url_for('order'))

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=50000)
