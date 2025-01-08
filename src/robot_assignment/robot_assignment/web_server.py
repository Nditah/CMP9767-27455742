import rclpy
from rclpy.node import Node
import sqlite3
import os
from flask import Flask, render_template_string

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
        <title>Object Detection Results</title>
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

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.get_logger().info("WebServerNode started, launching Flask on http://0.0.0.0:5000")

def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down Flask.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
