# L.A.D. Kubernetes Deployment

Kubernetes manifests for deploying L.A.D. (Learn Autonomous Driving) platform.

## Architecture

| Component | Description | Port |
|-----------|-------------|------|
| Frontend | React web UI | 3000 |
| Backend | Django REST API | 8000 |
| ROS | ROS 2 Humble + Gazebo | 9090, 7000, 8080 |

## Quick Start

### 1. Update Image Names

Edit the deployment files and replace `YOUR_DOCKERHUB_USER` with your Docker Hub username:

```bash
# In deployment-frontend.yaml, deployment-backend.yaml, deployment-ros.yaml:
image: YOUR_DOCKERHUB_USER/lad-frontend:latest
# Change to:
image: yourusername/lad-frontend:latest
```

Also update `update.sh`:
```bash
DOCKERHUB_USER="yourusername"
```

### 2. Copy Files to Server

```bash
# From your local machine
scp -r k8s/ root@YOUR_SERVER_IP:/opt/lad/
```

### 3. Initial Setup (First Time Only)

SSH to server and apply all manifests:

```bash
ssh root@YOUR_SERVER_IP
cd /opt/lad/k8s

# Option A: Apply all at once
kubectl apply -f .

# Option B: Apply in order (recommended first time)
kubectl apply -f namespace.yaml
kubectl apply -f configmap.yaml
kubectl apply -f secrets.yaml
kubectl apply -f pvc.yaml
kubectl apply -f deployment-frontend.yaml
kubectl apply -f deployment-backend.yaml
kubectl apply -f deployment-ros.yaml
kubectl apply -f service-frontend.yaml
kubectl apply -f service-backend.yaml
kubectl apply -f service-ros.yaml
kubectl apply -f ingress.yaml

# Or use the helper script
./update.sh setup
```

### 4. Updating After Code Changes

After pushing to GitHub (which triggers Docker Hub build):

```bash
ssh root@YOUR_SERVER_IP
cd /opt/lad/k8s

# Update all components (pulls latest images & restarts)
./update.sh update

# Or update specific component
./update.sh update-frontend
./update.sh update-backend
./update.sh update-ros
```

## Files

| File | Description |
|------|-------------|
| `namespace.yaml` | Creates `lad` namespace |
| `configmap.yaml` | Environment configuration |
| `secrets.yaml` | Secret template (edit with real values!) |
| `pvc.yaml` | Persistent storage (3 volumes) |
| `deployment-*.yaml` | Pod deployments |
| `service-*.yaml` | Internal services |
| `ingress.yaml` | External routing |
| `update.sh` | Helper script for updates |

## Useful Commands

```bash
# Check status
kubectl -n lad get pods
kubectl -n lad get services

# View logs
kubectl -n lad logs -f deployment/lad-frontend
kubectl -n lad logs -f deployment/lad-backend
kubectl -n lad logs -f deployment/lad-ros

# Restart a deployment
kubectl -n lad rollout restart deployment/lad-frontend

# Describe pod (for debugging)
kubectl -n lad describe pod <pod-name>
```

## Workflow Summary

```
Local: git push
    ↓
GitHub Actions: builds Docker images
    ↓
Docker Hub: stores images (yourusername/lad-*)
    ↓
Server: ./update.sh update (pulls & restarts)
```
