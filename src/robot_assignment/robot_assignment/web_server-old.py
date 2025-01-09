import rclpy
from rclpy.node import Node
import sqlite3
import os
from flask import Flask, render_template_string, redirect, url_for
from threading import Thread

DB_PATH = os.path.join(os.path.expanduser('~'), 'toy_detections.db')

app = Flask(__name__)

@app.route('/')
def index():
    # Connect to DB
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # Fetch all objects
    cursor.execute("""
        SELECT color, shape, pos_x, pos_y, pos_z, count, last_seen
        FROM objects
        ORDER BY last_seen DESC
    """)
    rows = cursor.fetchall()
    conn.close()

    # Compute total object count
    total_objects = sum(row[5] for row in rows) if rows else 0

    # Decide tidy/untidy
    tidy_status = "Tidy"
    status_color = "bg-success text-white"  # green
    if total_objects > 5:
        tidy_status = "Untidy"
        status_color = "bg-danger text-white"  # red

    # Example Bootstrap HTML template
    html_template = """
    <!DOCTYPE html>
    <html>
      <head>
        <title>Toy Detection Results</title>
        <!-- Bootstrap CSS -->
        <link 
          rel="stylesheet" 
          href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css"
        />
        <meta http-equiv="refresh" content="5"> <!-- auto-refresh every 5s -->
      </head>
      <body class="bg-light">
        <div class="container my-4">

          <!-- Status Banner -->
          <div class="row">
            <div class="col">
              <div class="alert {{ status_color }}" role="alert">
                <h4 class="alert-heading">Children's Space: {{ tidy_status }}</h4>
                <p>Total Toy Count = {{ total_objects }}</p>
              </div>
            </div>
          </div>

          <!-- Menubar -->
          <div class="row">
            <a href="{{ url_for('clear_database') }}" class="btn btn-danger">Clear Database</a>
            <a href="{{ url_for('restart_node') }}" class="btn btn-warning">Restart Node</a>
          </div>

          <h2 class="mb-3">Latest Toy Detections</h2>
          <table class="table table-bordered table-striped table-hover">
            <thead class="table-dark">
              <tr>
                <th>Timestamp</th>
                <th>Shape</th>
                <th>Color</th>
                <th>Location (x,y,z)</th>
                <th>Count</th>
              </tr>
            </thead>
            <tbody>
            {% for row in rows %}
              <tr>
                <!-- row = (color, shape, pos_x, pos_y, pos_z, count, last_seen) -->
                <td>{{ row[6] }}</td>  <!-- last_seen -->
                <td>{{ row[1] }}</td>  <!-- shape -->
                <td>{{ row[0] }}</td>  <!-- color -->
                <td>({{ row[2] }}, {{ row[3] }}, {{ row[4] }})</td>  <!-- pos_x, pos_y, pos_z -->
                <td>{{ row[5] }}</td>  <!-- count -->
              </tr>
            {% endfor %}
            </tbody>
          </table>
        </div>
        <!-- Bootstrap JS (optional) -->
        <script 
          src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/js/bootstrap.bundle.min.js">
        </script>

        <!-- Credit -->
        <footer class="text-center mt-4 bg-light py-3">
            <p class="mb-1">Special thanks to my lecturers and classmates</p>
            <div class="container">
                <p class="mb-1"><a href="mailto:rpolvara@lincoln.ac.uk">Dr. Riccardo Polvara</a></p>
                <p class="mb-1"><a href="mailto:gcielniak@lincoln.ac.uk">Dr. Grzegor Cielniak</a></p>
                <p class="mb-1"><a href="mailto:27455742@students.lincoln.ac.uk">Ndi-Tah Anyeh</a></p>
            </div>
        </footer>
      </body>
    </html>
    """

    return render_template_string(
        html_template,
        rows=rows,
        total_objects=total_objects,
        tidy_status=tidy_status,
        status_color=status_color
    )

@app.route('/clear_database')
def clear_database():
    # Connect to DB
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # Clear the database
    cursor.execute("DELETE FROM objects")
    conn.commit()
    conn.close()

    return redirect(url_for('index'))

@app.route('/restart_node')
def restart_node():
    # Restart the ros2 Node(s)
    os.system('colcon build --symlink-install')
    os.system('source install/setup.bash')
    os.system('ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/robot_assignment/worlds/custom_world2.world')
    os.system('rviz2 -d /opt/ros/lcas/src/limo_ros2/src/limo_gazebosim/rviz/urdf.rviz')
    os.system('ros2 launch limo_navigation limo_navigation.launch.py')
    os.system('ros2 run robot_assignment color_3d_detection')
    os.system('ros2 run robot_assignment detector_3d')
    os.system('ros2 run robot_assignment demo_inspection')
    os.system('ros2 run robot_assignment counter_3d')

    return redirect(url_for('index'))

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.get_logger().info('Web server node has been started.')

def run_flask_app():
    app.run(host='0.0.0.0', port=5000)

def main(args=None):
    rclpy.init(args=args)
    web_server_node = WebServerNode()

    flask_thread = Thread(target=run_flask_app)
    flask_thread.start()

    rclpy.spin(web_server_node)

    web_server_node.destroy_node()
    rclpy.shutdown()
    flask_thread.join()

if __name__ == '__main__':
    main()