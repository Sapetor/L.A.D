# Debugging API Connection Issues

## Changes Made

### 1. Added Comprehensive Logging

#### Frontend - fileApi.js
- Added logging to `createCanvas()` function
- Added logging to `executeCommand()` function
- Enhanced error handling in `handleResponse()` with detailed error info

#### Frontend - 03-CreatingPackageInteractive.jsx
- Added logging to `initializeWorkspace()`
- Added logging to `executePackageCreation()`
- Shows canvas ID, command, and result details

### 2. What to Check in Browser Console

When you open the slide, check the browser console (F12) for:

```
[FileAPI] Creating canvas: {name: "...", description: "..."}
[FileAPI] API endpoint: /api/workspace/canvases/
[FileAPI] Auth headers: {Content-Type: "...", Authorization: "..."}
```

#### If you see "API request failed (401)" or "Unauthorized":
- User is not logged in
- Token is expired
- Solution: Log in to the platform first

#### If you see "API request failed (404)":
- API endpoint doesn't exist
- Check that backend is running on port 8000
- Verify proxy configuration

#### If you see "API request failed (500)":
- Backend error
- Check Django console for Python errors
- Check if Docker container is running

### 3. Backend Verification

Check these components are running:

```bash
# 1. Django backend
cd LAD/lad && python manage.py runserver
# Should show: Starting development server at http://127.0.0.1:8000/

# 2. Docker container
docker ps --filter "name=qcar_docker-ros-1"
# Should show: qcar_docker-ros-1 - Up ...

# 3. Frontend dev server
cd AVEDU/avedu && npm start
# Should show: Compiled successfully!
```

### 4. Test API Manually

You can test the API endpoints manually:

```bash
# Get auth token first (if you have a user)
curl -X POST http://localhost:8000/api/token/ \
  -H "Content-Type: application/json" \
  -d '{"username":"your_username","password":"your_password"}'

# Test create canvas (replace TOKEN)
curl -X POST http://localhost:8000/api/workspace/canvases/ \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer TOKEN" \
  -d '{"name":"test_canvas","description":"test"}'
```

### 5. Common Issues and Solutions

#### Issue: "Error: API request failed (401)"
**Cause**: Not authenticated
**Solution**:
1. Make sure you're logged in to the platform
2. Check localStorage has a token: `localStorage.getItem('token')`
3. If no token, go to login page first

#### Issue: "Error: API request failed (404)"
**Cause**: Backend not running or wrong URL
**Solution**:
1. Start Django backend: `cd LAD/lad && python manage.py runserver`
2. Verify proxy in package.json points to http://127.0.0.1:8000
3. Check API_BASE in config.js is '/api'

#### Issue: "Error: Failed to fetch"
**Cause**: Network error or CORS issue
**Solution**:
1. Check backend is running
2. Verify CORS settings in Django settings.py
3. Check browser network tab for actual error

#### Issue: Command executes but package not created
**Cause**: Docker container not running or ROS2 not installed
**Solution**:
1. Verify Docker: `docker ps | grep ros`
2. Test command manually: `docker exec qcar_docker-ros-1 which ros2`
3. Check Docker container has ROS2 installed

### 6. Debugging Flow

1. **Open Browser Console (F12)**
2. **Navigate to the Package Creator slide**
3. **Look for log messages**:
   - `[Package Creator] Creating canvas...`
   - `[FileAPI] Creating canvas: ...`
   - `[FileAPI] Canvas created successfully: ...`

4. **If creation works, connect the blocks**:
   - `[Package Creator] Executing command: ...`
   - `[FileAPI] Executing command: ...`
   - `[FileAPI] Command executed: ...`
   - `[Package Creator] Command result: ...`

5. **Check the status message** in the UI for user-friendly errors

### 7. Next Steps

If issues persist after checking the above:

1. Share the browser console logs
2. Share the Django console output
3. Verify Docker container is ros-enabled: `docker exec qcar_docker-ros-1 ros2 --help`
