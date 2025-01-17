from flask import Flask, request, jsonify, render_template
import requests
import json
import time

app = Flask(__name__, template_folder='templates')

# 全局缓存 Access Token
access_token_cache = {
    "token": None,
    "expires_at": 0
}

# 替换为您的百度云 API Key 和 Secret Key
API_KEY = "XRR1BDQXqaboHIswuXoiqi5J"
SECRET_KEY = "fKe14R8keYAjRKUznj40jTvADujX9V3a"

def get_access_token():
    """
    获取百度文心一言 API 的 access_token，使用缓存以减少多次请求。
    """
    global access_token_cache

    # 如果 token 未失效，直接返回缓存的 token
    if time.time() < access_token_cache["expires_at"]:
        return access_token_cache["token"]

    url = "https://aip.baidubce.com/oauth/2.0/token"
    params = {
        "grant_type": "client_credentials",
        "client_id": API_KEY,
        "client_secret": SECRET_KEY
    }

    try:
        response = requests.post(url, params=params)
        response.raise_for_status()
        data = response.json()

        if "access_token" in data:
            # 更新缓存
            access_token_cache["token"] = data["access_token"]
            access_token_cache["expires_at"] = time.time() + data["expires_in"] - 60  # 提前 60 秒更新
            return data["access_token"]
        else:
            print(f"获取 Access Token 失败: {data}")
            return None
    except requests.RequestException as e:
        print(f"请求 Access Token 时发生错误: {e}")
        return None

def send_message(messages):
    """
    调用文心一言接口发送对话消息。
    """
    access_token = get_access_token()
    if not access_token:
        print("无法获取 Access Token，请检查 API Key 和网络连接。")
        return None

    url = f"https://aip.baidubce.com/rpc/2.0/ai_custom/v1/wenxinworkshop/chat/completions_pro?access_token={access_token}"
    headers = {"Content-Type": "application/json"}
    payload = json.dumps({"messages": messages})

    try:
        response = requests.post(url, headers=headers, data=payload)
        response.raise_for_status()
        return response.json()
    except requests.RequestException as e:
        print(f"调用文心一言 API 时发生错误: {e}")
        return None

@app.route('/')
def index():
    """
    返回前端页面
    """
    return render_template('index.html')

@app.route('/ask', methods=['POST'])
def ask():
    """
    处理 POST 请求，调用文心一言 API。
    请求格式：
    {
        "question": "你的问题"
    }
    """
    user_question = request.json.get('question')
    if not user_question:
        return jsonify({"error": "问题不能为空"}), 400

    # 构建对话消息
    messages = [
        {"role": "user", "content": user_question}
    ]

    # 调用文心一言 API
    result = send_message(messages)

    if result and "result" in result:
        return jsonify({"answer": result["result"]})
    else:
        return jsonify({"error": "文心一言 API 调用失败或无结果"}), 500

if __name__ == '__main__':
    app.run(debug=True)
