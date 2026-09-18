

#!/usr/bin/env bash
# Avvia il container (se non già avviato) e poi lancia il nodo hand tracking

# Avvia il container (se non già avviato)
docker compose up -d

echo "Per favore, apri due terminali distinti e segui i prossimi passaggi:"

echo "Terminale 1: Avvio del nodo hand tracking"
echo "Esegui il comando:"
echo "docker compose exec ros2 bash -lc 'source /ros_ws/install/setup.bash && ros2 launch hand_tracking_ros htr_realsense_launch.py'"

echo ""
echo "Terminale 2: Registrazione rosbag"
echo "Esegui il comando:"
echo "docker compose exec ros2 bash -lc 'source /ros_ws/install/setup.bash && ros2 bag record /camera/camera/color/image_raw /camera/camera/depth/image_rect_raw /camera/camera/color/camera_info /camera/camera/depth/camera_info /tf /hand_tracking/image_annotated'"

echo ""
echo "Quando vuoi fermare la registrazione, premi Ctrl+C nel terminale 2 e immetti il nome della bag quando richiesto."
