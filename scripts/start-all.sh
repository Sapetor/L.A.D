#!/bin/bash
# scripts/start-all.sh - Start all L.A.D services with auto-detected IP

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   L.A.D Platform - Starting Services  ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}\n"

# Detect IP using Node.js
echo -e "${GREEN}[1/6]${NC} Detecting local IP address..."
cd AVEDU/avedu
IP=$(node scripts/detect-ip.js 2>&1 | grep "Detected local IP:" | awk '{print $NF}')
cd ../..

if [ -z "$IP" ]; then
    echo -e "${YELLOW}⚠️  Could not auto-detect IP, using config file${NC}"
    IP=$(cat config/ip_config.json | grep exposed_ip | cut -d'"' -f4)
fi

echo -e "${GREEN}✓${NC} Using IP: ${BLUE}$IP${NC}\n"

# Update React .env.local with detected IP
echo -e "${GREEN}[2/6]${NC} Updating React environment variables..."
cat > AVEDU/avedu/.env.local <<EOF
# .env.local - Local development overrides (gitignored)
# This file is automatically managed by the start script

# Bind to all network interfaces to allow LAN connections
HOST=0.0.0.0

# Allow connections from any host (required for LAN access)
DANGEROUSLY_DISABLE_HOST_CHECK=true

# API base - uses proxy during development
REACT_APP_API_BASE=/api

# Detected network IP for ROS/Static connections
REACT_APP_HOST=$IP
EOF
echo -e "${GREEN}✓${NC} React .env.local updated with IP: ${BLUE}$IP${NC}\n"

# Update Docker CORS configuration
echo -e "${GREEN}[3/6]${NC} Updating Docker CORS configuration..."
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    sed -i '' "s|CORS_ALLOW_ORIGIN=http://localhost:3000,http://127.0.0.1:3000,http://[^:]*:3000|CORS_ALLOW_ORIGIN=http://localhost:3000,http://127.0.0.1:3000,http://$IP:3000|" qcar_docker/docker-compose.yml
else
    # Linux
    sed -i "s|CORS_ALLOW_ORIGIN=http://localhost:3000,http://127.0.0.1:3000,http://[^:]*:3000|CORS_ALLOW_ORIGIN=http://localhost:3000,http://127.0.0.1:3000,http://$IP:3000|" qcar_docker/docker-compose.yml
fi
echo -e "${GREEN}✓${NC} CORS updated for IP: ${BLUE}$IP${NC}\n"

# Start ROS Docker
echo -e "${GREEN}[4/6]${NC} Starting ROS 2 Docker (rosbridge + QCar + Gazebo)..."
cd qcar_docker
docker compose up -d
cd ..
echo -e "${GREEN}✓${NC} ROS services running on:"
echo -e "   - rosbridge: ws://$IP:9090"
echo -e "   - static server: http://$IP:7000"
echo -e "${YELLOW}⏳${NC} Gazebo initialization takes 60-90 seconds...\n"

# Start Django Backend
echo -e "${GREEN}[5/6]${NC} Starting Django backend..."
cd LAD/lad
if [ ! -d "../.venv" ]; then
    echo -e "${YELLOW}⚠️  Virtual environment not found, creating...${NC}"
    python3 -m venv ../.venv
    source ../.venv/bin/activate
    pip install -r requirements.txt 2>&1 | grep -E "Successfully|Requirement already"
else
    source ../.venv/bin/activate
fi

# Run migrations if needed
python manage.py migrate --no-input 2>&1 | grep -E "Applying|No migrations"

# Start Django in background
python manage.py runserver 0.0.0.0:8000 &
DJANGO_PID=$!
echo $DJANGO_PID > /tmp/lad_django.pid
cd ../..
echo -e "${GREEN}✓${NC} Django API running on: http://$IP:8000\n"

# Start React Frontend
echo -e "${GREEN}[6/6]${NC} Starting React frontend..."
cd AVEDU/avedu
npm start &
REACT_PID=$!
echo $REACT_PID > /tmp/lad_react.pid
cd ../..

echo -e "\n${GREEN}════════════════════════════════════════${NC}"
echo -e "${GREEN}   All services started successfully!   ${NC}"
echo -e "${GREEN}════════════════════════════════════════${NC}\n"

echo -e "${BLUE}🌐 Access your application:${NC}"
echo -e "   Local:   ${GREEN}http://localhost:3000${NC}"
echo -e "   Network: ${GREEN}http://$IP:3000${NC}\n"

echo -e "${BLUE}🔌 ROS endpoints:${NC}"
echo -e "   rosbridge: ${GREEN}ws://$IP:9090${NC}"
echo -e "   Static server: ${GREEN}http://$IP:7000${NC}"
echo -e "   Django API: ${GREEN}http://$IP:8000${NC}\n"

echo -e "${YELLOW}⏳ Wait 60-90 seconds for Gazebo to initialize${NC}\n"

echo -e "${YELLOW}ℹ️  To stop all services, run:${NC}"
echo -e "   ./scripts/stop-all.sh\n"

echo -e "${YELLOW}ℹ️  To view logs:${NC}"
echo -e "   Django:  tail -f LAD/lad/django.log"
echo -e "   React:   Check terminal output"
echo -e "   Docker:  docker compose -f qcar_docker/docker-compose.yml logs -f\n"

# Keep script running (optional)
# wait
