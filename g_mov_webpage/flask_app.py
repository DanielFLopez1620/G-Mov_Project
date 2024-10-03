from flask import Flask, render_template, request, redirect, url_for

app = Flask(__name__)

@app.route('/')
def index():
    return redirect(url_for('home'))

@app.route('/home')
def home():
    stream_url = "http://<ip_raspberry>:8080/?action=stream"
    return render_template('home.html', stream_url=stream_url)

if __name__ == '__main__':
    app.run(debug=True)