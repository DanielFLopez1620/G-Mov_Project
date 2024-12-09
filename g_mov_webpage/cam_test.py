from flask import Flask, Response
import rclpy
from mjpeg_streamer import MJPEGStreamer

app = Flask(__name__)
streamer_node = None

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            frame = streamer_node.get_frame()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
            time.sleep(0.05)  # Control the frame rate
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def main(args=None):
    global streamer_node
    rclpy.init(args=args)
    streamer_node = MJPEGStreamer()

    # Start Flask in a separate thread
    threading.Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000}).start()
    rclpy.spin(streamer_node)

    streamer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
