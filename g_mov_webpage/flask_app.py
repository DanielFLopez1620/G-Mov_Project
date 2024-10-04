from flask import Flask, render_template, redirect, url_for
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

@app.route('/')
def index():
    return redirect(url_for('home'))

@app.route('/home')
def home():
    return render_template('home.html')    

if __name__ == '__main__':
    app.run(debug=True)