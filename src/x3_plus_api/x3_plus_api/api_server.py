import os
import threading
import rclpy
from flask import Flask, jsonify, request
from rclpy.executors import ExternalShutdownException
from x3_plus_api.ros2_bridge import ROS2Bridge

app = Flask(__name__)
ros_node: ROS2Bridge = None

# Configuración base del servidor
HOST = '0.0.0.0'
PORT = 5000
DOMAIN_NAME = 'x3plus.api'

# Formato de la respuesta JSON para el estado del robot
@app.route('/', methods=['GET'])
def index():
    return jsonify({
        "message": "Welcome to the X3 Plus API server. Use /api/v1/status for robot status and /api/v1/teleop to control teleoperation.",
        "version": "1.0.0",
        "endpoints": {
            "/api/v1/status": "GET - Get the current status of the robot (mobile and arm states).",
            "/api/v1/teleop": "POST - Activate or deactivate teleoperation. Send JSON {'enable': true/false}.",
            "/api/v1/capture/start": "POST - Start capturing images and states. Optionally enable teleoperation with {'auto_teleop': true/false}.",
            "/api/v1/capture/end": "POST - Stop capturing and return a summary of the episode. Optionally disable teleoperation with {'stop_teleop': true/false}."
        }
    }), 200

def ros_spin_thread():
    """Run rclpy.spin in a separate thread to handle ROS2 callbacks."""
    try:
        rclpy.spin(ros_node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except Exception as e:
        if rclpy.ok():
            ros_node.get_logger().error(f"Exception in ROS2 spin thread: {e}")


# Endpoints de estado y teleoperación

# Estado del robot - Método GET
@app.route('/api/v1/status', methods=['GET'])
def get_status():
    """Return status = {mobile, arm} y and teleop_active."""
    return jsonify(ros_node.get_snapshot()), 200

# Activar/desactivar teleoperación - Método POST
@app.route('/api/v1/teleop', methods=['POST'])
def control_teleop():
    """Activate/Deactivate teleoperation."""
    data = request.get_json(silent=True) or {}
    enable = bool(data.get('enable', False))
    active = ros_node.set_teleop(enable)
    return jsonify({
        "status": "ok",
        "teleop_active": active,
        "message": "Teleoperation activated" if active else "Teleoperation deactivated"
    }), 200

# Endpoints de captura de imágenes y estados

@app.route('/api/v1/capture/start', methods=['POST'])
def start_capture():
    """Start of image capture = {I_1, ..., I_n} and status = {mobile, arm}."""
    data = request.get_json(silent=True) or {}
    if data.get('auto_teleop', True):
        ros_node.set_teleop(True)
    ros_node.start_capture()
    return jsonify({
        "status": "recording_started",
        "teleop_active": ros_node.teleop_active
    }), 200

@app.route('/api/v1/capture/end', methods=['POST'])
def end_capture():
    """Stop the capture (end ->) and returns the episode resume for X3_LeKiwi."""
    if request.get_json(silent=True, force=False) and request.json.get('stop_teleop', True):
        ros_node.set_teleop(False)
    
    episode_data = ros_node.stop_capture()
    return jsonify({
        "status": "episode_finished",
        "dataset_target": "~/LeRobot/X3_LeKiwi",
        "resume": {
            "captured_frames": episode_data["total_frames"],
            "sample_images": episode_data["Image"][:3], # Primeros 3 frames como vista previa
            "last_state": episode_data["Sync_States"][-1] if episode_data["Sync_States"] else None
        }
    }), 200

# Función principal para iniciar el servidor Flask y el nodo ROS2

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = ROS2Bridge()

    # Correr rclpy.spin en un hilo separado para no bloquear Flask
    spin_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    spin_thread.start()

    try:
        # Levantar API en el puerto 5000
        app.run(host=HOST, port=PORT, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        if ros_node is not None:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()