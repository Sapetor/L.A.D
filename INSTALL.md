# L.A.D Platform - Installation Guide

This guide will help you install and set up the L.A.D (Learn Autonomous Driving) platform on your computer.

## Prerequisites

Before you begin, ensure you have the following installed:

- **Python** 3.10 or 3.11 ([Download](https://www.python.org/downloads/))
- **Node.js** 18.x or higher and npm ([Download](https://nodejs.org/))
- **Docker Desktop** ([Download](https://www.docker.com/products/docker-desktop/))
- **Git** ([Download](https://git-scm.com/downloads))

## Quick Installation

### Windows

1. **Clone the repository**
   ```cmd
   git clone <repository-url>
   cd L.A.D
   ```

2. **Run the automated installer**

   Simply double-click `installer.bat` or run from command line:
   ```cmd
   installer.bat
   ```

3. **Start the platform**
   ```cmd
   scripts\start-all.bat
   ```

4. **Access the application**
   - Open your browser to: `http://localhost:3000`
   - Django admin: `http://localhost:8000/admin`

### Linux/Mac

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd L.A.D
   ```

2. **Run the automated installer**
   ```bash
   chmod +x installer.sh
   ./installer.sh
   ```

3. **Start the platform**
   ```bash
   chmod +x scripts/*.sh
   ./scripts/start-all.sh
   ```

4. **Access the application**
   - Open your browser to: `http://localhost:3000`
   - Django admin: `http://localhost:8000/admin`

## What the Installer Does

The automated installer will:

1. ✅ **Check prerequisites** - Verifies Python, Node.js, and Docker are installed
2. ✅ **Create Python virtual environment** - Isolates Python dependencies
3. ✅ **Install backend dependencies** - Installs Django and required packages from `requirements.txt`
4. ✅ **Set up database** - Runs migrations to create database schema
5. ✅ **Load sample data** - Imports pre-configured units, levels, and objectives
6. ✅ **Create admin account** - Prompts you to create a superuser
7. ✅ **Install frontend dependencies** - Runs `npm install` for React app
8. ✅ **Build Docker images** - Creates ROS 2 simulation environment (takes 10-20 min)

## Post-Installation

### Create Your First Admin Account

During installation, you'll be prompted to create a superuser account:

```
Username: admin
Email: admin@example.com
Password: ********
Password (again): ********
```

This account gives you access to the Django admin panel at `http://localhost:8000/admin`

### Verify Installation

After running `start-all`, you should see:

**Backend (Django)**
```
Django version 5.1.3, using settings 'core.settings'
Starting development server at http://0.0.0.0:8000/
```

**Frontend (React)**
```
Compiled successfully!
You can now view avedu in the browser.

  Local:            http://localhost:3000
  On Your Network:  http://192.168.x.x:3000
```

**ROS Docker**
```
[INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
```

### Sample Content

The installer loads the following sample content:

**Units:**
- ROS - Introduction to ROS concepts
- Vehicle Dynamics - Understanding vehicle physics
- Autonomous Vehicles - Autonomous driving principles

**Levels:**
- ROS-Basic - Basic ROS concepts and interaction
- Turtlesim - ROS Turtlesim tutorial
- Rviz - 3D visualization with RViz
- Gazebo Simulation - QCar autonomous vehicle simulation
- Vehicle Dynamics - Interactive vehicle physics lessons

**Objectives:**
Each level includes objectives that students complete to earn points.

## Troubleshooting

### Python not found

**Windows:**
```cmd
# Download from https://www.python.org/downloads/
# Make sure to check "Add Python to PATH" during installation
```

**Linux/Mac:**
```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install python3 python3-pip python3-venv

# macOS (using Homebrew)
brew install python@3.11
```

### Node.js not found

Download and install from: https://nodejs.org/

Verify installation:
```bash
node --version
npm --version
```

### Docker not available

**Windows/Mac:**
- Download Docker Desktop from https://www.docker.com/products/docker-desktop/
- Start Docker Desktop before running the installer

**Linux:**
```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Install Docker Compose
sudo apt-get install docker-compose-plugin

# Add your user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

### Virtual environment activation fails

**Error:** "Directory not found" when activating virtual environment

**Cause:** The `.venv` directory is not committed to Git (it's in `.gitignore`), so it must be created during installation.

**Solution:**
```bash
# Linux/Mac
cd LAD/lad
rm -rf ../.venv  # Remove corrupted venv if exists
python3 -m venv ../.venv
source ../.venv/bin/activate
pip install -r requirements.txt

# Windows
cd LAD\lad
rmdir /s /q ..\.venv  # Remove corrupted venv if exists
python -m venv ..\.venv
..\.venv\Scripts\activate.bat
pip install -r requirements.txt
```

**Note:** The updated installer scripts now automatically check and recreate the virtual environment if the activation script is missing.

### Installation fails at database migrations

```bash
cd LAD/lad
source ../.venv/bin/activate  # Windows: ..\.venv\Scripts\activate
python manage.py migrate --run-syncdb
```

### Installation fails at npm install

```bash
cd AVEDU/avedu
rm -rf node_modules package-lock.json  # Windows: rmdir /s node_modules & del package-lock.json
npm install
```

### Docker build takes too long or fails

The first Docker build can take 10-20 minutes. If it fails:

```bash
cd qcar_docker
docker compose build --no-cache
```

## Manual Installation (Advanced)

If the automated installer doesn't work, you can install manually:

### 1. Backend Setup

```bash
cd LAD/lad

# Create virtual environment
python -m venv ../.venv

# Activate virtual environment
source ../.venv/bin/activate  # Windows: ..\.venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run migrations
python manage.py migrate

# Load fixtures
python manage.py loaddata fixtures/initial_data.json

# Create superuser
python manage.py createsuperuser
```

### 2. Frontend Setup

```bash
cd AVEDU/avedu
npm install
```

### 3. Docker Setup

```bash
cd qcar_docker
docker compose build
```

### 4. Start Services

Use the provided start scripts:
- Windows: `scripts\start-all.bat`
- Linux/Mac: `./scripts/start-all.sh`

Or start manually in separate terminals:

```bash
# Terminal 1 - Backend
cd LAD/lad
source ../.venv/bin/activate
python manage.py runserver 0.0.0.0:8000

# Terminal 2 - Frontend
cd AVEDU/avedu
npm start

# Terminal 3 - ROS Docker
cd qcar_docker
docker compose up
```

## Next Steps

1. **Login to the platform** at `http://localhost:3000`
2. **Explore the admin panel** at `http://localhost:8000/admin`
3. **Review the documentation** in `README.md` and `CLAUDE.md`
4. **Test ROS connectivity** by navigating to a ROS level (e.g., Turtlesim)
5. **Set up LAN access** for multi-device deployment (see README.md)

## Support

For additional help:
- Check the main [README.md](README.md)
- Review [CLAUDE.md](CLAUDE.md) for detailed architecture documentation
- Open an issue on GitHub
- Check the troubleshooting section in the main README

## Uninstallation

To remove the installation:

1. **Stop all services**
   ```bash
   # Windows
   scripts\stop-all.bat

   # Linux/Mac
   ./scripts/stop-all.sh
   ```

2. **Remove virtual environment**
   ```bash
   rm -rf LAD/.venv  # Windows: rmdir /s LAD\.venv
   ```

3. **Remove node modules**
   ```bash
   rm -rf AVEDU/avedu/node_modules  # Windows: rmdir /s AVEDU\avedu\node_modules
   ```

4. **Remove Docker images** (optional)
   ```bash
   docker compose -f qcar_docker/docker-compose.yml down --rmi all
   ```

5. **Delete the repository folder**
   ```bash
   cd ..
   rm -rf L.A.D  # Windows: rmdir /s L.A.D
   ```

---

**Installation complete! Enjoy learning autonomous driving with L.A.D!** 🚗🤖
