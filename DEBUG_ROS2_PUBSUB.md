# Debugging ROS2 Publisher/Subscriber Issues

## Why You Don't See Messages

When you run a ROS2 subscriber, it **waits silently** for messages. You'll only see output when:
1. A publisher is sending messages to the same topic
2. The message arrives and the callback is triggered

The "command is running" means the node is active and waiting - this is correct behavior!

## Quick Debugging Steps

### Step 1: Check if both nodes are running

Open **two separate terminals** (or use tmux/screen):

**Terminal 1 - Publisher:**
```bash
cd ~/your_workspace_id
source install/setup.bash
ros2 run my_robot_pkg SimplePublisher
```

**Terminal 2 - Subscriber:**
```bash
cd ~/your_workspace_id
source install/setup.bash
ros2 run my_robot_pkg SimpleListener
```

You should now see messages in Terminal 2!

### Step 2: Verify nodes are registered

In a third terminal (or stop one of the nodes temporarily):

```bash
# List all running nodes
ros2 node list

# Should show:
# /simple_publisher
# /simple_listener
```

### Step 3: Verify topic connection

```bash
# List all active topics
ros2 topic list

# Should show:
# /chatter
# /parameter_events
# /rosout

# Check who's publishing to /chatter
ros2 topic info /chatter

# Should show:
# Publisher count: 1
# Subscription count: 1

# Echo messages directly (this will show if publisher is working)
ros2 topic echo /chatter
```

### Step 4: Check message rate

```bash
# Check if messages are actually being published
ros2 topic hz /chatter

# Should show something like:
# average rate: 1.000
# min: 1.000s max: 1.000s std dev: 0.00000s window: 2
```

## Common Issues

### Issue 1: "No executable found"

**Cause:** Entry points not configured correctly in setup.py

**Fix:**
```python
# In setup.py, ensure both are listed:
entry_points={
    'console_scripts': [
        'SimplePublisher = my_robot_pkg.SimplePublisher:main',
        'SimpleListener = my_robot_pkg.SimpleListener:main',
    ],
},
```

Then rebuild:
```bash
rm -rf build install log
colcon build --packages-select my_robot_pkg
source install/setup.bash
```

### Issue 2: Topic Names Don't Match

**Publisher topic:**
```python
self.publisher_ = self.create_publisher(String, '/chatter', 10)
```

**Subscriber topic:**
```python
self.subscription = self.create_subscription(String, '/chatter', ...)
```

Both must use **exactly the same topic name**!

### Issue 3: Not Sourcing Workspace

Every new terminal needs:
```bash
cd ~/your_workspace_id
source install/setup.bash
```

Without this, ROS2 won't find your executables!

### Issue 4: Message Types Don't Match

Publisher and subscriber must use the **same message type**:
- Both use `std_msgs/String` ✅
- One uses `std_msgs/String`, other uses `std_msgs/Int32` ❌

## Testing Without Multiple Terminals

If you can't open multiple terminals easily, use these alternatives:

### Option 1: Use background processes

```bash
# Start publisher in background
ros2 run my_robot_pkg SimplePublisher &

# Start subscriber in foreground (you'll see output)
ros2 run my_robot_pkg SimpleListener

# Stop background publisher when done
killall SimplePublisher
```

### Option 2: Test with ros2 pub command

```bash
# Manually publish a test message
ros2 topic pub /chatter std_msgs/msg/String "data: 'Test message'" --once

# Or publish continuously
ros2 topic pub /chatter std_msgs/msg/String "data: 'Test message'" --rate 1
```

Then run your subscriber:
```bash
ros2 run my_robot_pkg SimpleListener
# Should now show: Received: "Test message"
```

### Option 3: Use tmux (terminal multiplexer)

```bash
# Install tmux if not available
apt-get update && apt-get install -y tmux

# Start tmux session
tmux

# Split screen horizontally (Ctrl+B then ")
# Or vertically (Ctrl+B then %)

# In left pane:
ros2 run my_robot_pkg SimplePublisher

# Switch to right pane (Ctrl+B then arrow key)
ros2 run my_robot_pkg SimpleListener

# You should see messages flowing!
```

## Expected Output

### Publisher Output:
```
[INFO] [1234567890.123456789] [simple_publisher]: SimplePublisher started!
[INFO] [1234567890.123456790] [simple_publisher]: Publishing: "Hello World: 0"
[INFO] [1234567891.123456789] [simple_publisher]: Publishing: "Hello World: 1"
[INFO] [1234567892.123456789] [simple_publisher]: Publishing: "Hello World: 2"
```

### Subscriber Output:
```
[INFO] [1234567890.123456789] [simple_listener]: Subscriber started on /chatter
[INFO] [1234567890.123456790] [simple_listener]: Received: "Hello World: 0"
[INFO] [1234567891.123456789] [simple_listener]: Received: "Hello World: 1"
[INFO] [1234567892.123456789] [simple_listener]: Received: "Hello World: 2"
```

## ROS2 Graph Visualization

Check the entire system visually:

```bash
# Install rqt_graph if needed
apt-get update && apt-get install -y ros-humble-rqt-graph

# Launch graph viewer
rqt_graph
```

You should see:
```
[SimplePublisher] ---> /chatter ---> [SimpleListener]
```

## Quick Verification Commands

```bash
# 1. Check package is built
ros2 pkg list | grep my_robot_pkg

# 2. Check executables exist
ros2 pkg executables my_robot_pkg
# Should output:
# my_robot_pkg SimplePublisher
# my_robot_pkg SimpleListener

# 3. Check nodes are running
ros2 node list

# 4. Check topics are active
ros2 topic list

# 5. Check topic info
ros2 topic info /chatter

# 6. Echo topic (see raw messages)
ros2 topic echo /chatter

# 7. Check message rate
ros2 topic hz /chatter
```

## If Still Not Working

### Enable debug logging:

**For Publisher:**
```bash
ros2 run my_robot_pkg SimplePublisher --ros-args --log-level DEBUG
```

**For Subscriber:**
```bash
ros2 run my_robot_pkg SimpleListener --ros-args --log-level DEBUG
```

### Check Python imports:

```bash
# Make sure ROS2 modules can be imported
python3 -c "import rclpy; print('rclpy OK')"
python3 -c "from std_msgs.msg import String; print('std_msgs OK')"
python3 -c "import my_robot_pkg; print('my_robot_pkg OK')"
python3 -c "import my_robot_pkg.SimplePublisher; print('SimplePublisher OK')"
python3 -c "import my_robot_pkg.SimpleListener; print('SimpleListener OK')"
```

### Check file permissions:

```bash
# Ensure Python files are readable
ls -la my_robot_pkg/my_robot_pkg/*.py

# Should show readable files (rw-r--r-- or similar)
```

## Complete Test Script

Save this as `test_pubsub.sh`:

```bash
#!/bin/bash

echo "=== ROS2 Pub/Sub Test ==="

# Source workspace
source install/setup.bash

# Check executables
echo "1. Checking executables..."
ros2 pkg executables my_robot_pkg

# Start publisher in background
echo "2. Starting publisher..."
ros2 run my_robot_pkg SimplePublisher &
PUB_PID=$!
sleep 2

# Start subscriber for 5 seconds
echo "3. Starting subscriber (5 seconds)..."
timeout 5 ros2 run my_robot_pkg SimpleListener

# Cleanup
echo "4. Stopping publisher..."
kill $PUB_PID

echo "=== Test Complete ==="
```

Run it:
```bash
chmod +x test_pubsub.sh
./test_pubsub.sh
```

## Common Topic Names to Try

If `/chatter` doesn't work, try these standard topics:

```bash
# For testing with built-in ROS2 demos
/test_topic
/sensor_data
/robot/status
/cmd_vel      # Common for robot velocity commands
/odom         # Common for odometry data
```

## Pro Tip: Use `ros2 run` with verbose mode

```bash
# See more detailed output
ros2 run my_robot_pkg SimpleListener --ros-args -r __node:=my_listener_node --log-level INFO
```

This will show you exactly what's happening with your node!
