from flask import Flask, render_template, Response, redirect, url_for
import requests
from flask_cors import CORS

app = Flask(__name__)

# stream_url = "http://10.147.19.165:8080/?action=stream" # Localhost
# stream_url = "https://1566-191-156-233-27.ngrok-free.app/?action=stream" # Ngrok

@app.route('/')
def index():
    return redirect(url_for('home'))

@app.route('/home')
def home():
    return render_template('home.html')

@app.route('/video_feed')
def video_feed():
    # Add ngrok-skip-browser-warning header to bypass the warning page
    headers = {
        'ngrok-skip-browser-warning': 'true'
    }

    # Send request to the ngrok stream
    try:
        response = requests.get(stream_url, headers=headers, stream=True)
        
        if response.status_code == 200:
            # Stream the video from the ngrok stream
            return Response(response.iter_content(chunk_size=1024), content_type='multipart/x-mixed-replace; boundary=frame')
        else:
            return "Failed to fetch the video stream", 500
    except Exception as e:
        return f"An error occurred: {e}", 500

if __name__ == '__main__':
    app.run(debug=True)