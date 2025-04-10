# Run the Docker container (default command from docker-compose.yml)
echo "🚀 Running the ROS 2 project in Docker..."
docker-compose -f docker-compose.dev.yml up --build
docker exec -it multi_robots bash
echo "Connect to container in VSCode to edit files."