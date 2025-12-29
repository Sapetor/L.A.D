# Fixing ROS2 Entry Points Errors

## Your Errors Explained

### Error 1: sensor_node
```
AttributeError: 'NoneType' object has no attribute 'group'
```
**Cause**: Incorrect entry_points format in setup.py

### Error 2: SimplePublisher
```
no executable found
```
**Cause**: Entry point not registered correctly or wrong format

## Common Issues

### Issue #1: Package Name with Hyphens vs Underscores ⚠️

Notice in your error:
```python
load_entry_point('my-robot-pkg==0.0.0', ...)  # ← HYPHENS!
```

**THE RULE:**
- Python modules **MUST** use underscores: `my_robot_pkg`
- setuptools **converts** package names to hyphens during install
- Entry points **MUST** reference the module with underscores

### Issue #2: Wrong Entry Point Format

**WRONG FORMATS ❌:**
```python
# Missing package name
'sensor_node = sensor_node:main'

# Using hyphens in module
'sensor_node = my-robot-pkg.sensor_node:main'

# Wrong separator
'sensor_node = my_robot_pkg/sensor_node:main'

# Missing function
'sensor_node = my_robot_pkg.sensor_node'
```

**CORRECT FORMAT ✅:**
```python
'executable_name = package_module.file_name:function_name'
#                  ^^^^^^^^^^^^^ ^^^^^^^^^ ^^^^^^^^^^^^^
#                  underscores!  no .py!   usually 'main'
```

## Correct setup.py

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_robot_pkg'  # ← Underscores!

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Robot package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Format: 'executable = package.module:function'
            # IMPORTANT: Use underscores in package name!
            'sensor_node = my_robot_pkg.sensor_node:main',
            'SimplePublisher = my_robot_pkg.SimplePublisher:main',
        ],
    },
)
```

## Required Directory Structure

```
my_robot_pkg/                    # Root folder
├── my_robot_pkg/                # Package folder (MUST match package_name)
│   ├── __init__.py              # REQUIRED! (can be empty)
│   ├── sensor_node.py           # Your sensor node
│   └── SimplePublisher.py       # Your publisher node
├── resource/
│   └── my_robot_pkg             # Empty marker file
├── package.xml
└── setup.py
```

## Fix Steps

### Step 1: Check Your Files

```bash
# In terminal, navigate to your workspace
cd /workspace/.../your_workspace_id/

# Check directory structure
ls -la my_robot_pkg/
ls -la my_robot_pkg/my_robot_pkg/

# MUST see:
# my_robot_pkg/__init__.py
# my_robot_pkg/sensor_node.py
# my_robot_pkg/SimplePublisher.py
```

### Step 2: Verify __init__.py Exists

```bash
# Create it if missing
touch my_robot_pkg/my_robot_pkg/__init__.py
```

### Step 3: Check setup.py Format

Your `entry_points` section should look EXACTLY like this:

```python
entry_points={
    'console_scripts': [
        'sensor_node = my_robot_pkg.sensor_node:main',
        'SimplePublisher = my_robot_pkg.SimplePublisher:main',
    ],
},
```

### Step 4: Clean and Rebuild

```bash
# Remove old build artifacts
rm -rf build/ install/ log/

# Rebuild from scratch
colcon build --packages-select my_robot_pkg

# Source the workspace
source install/setup.bash
```

### Step 5: Verify Registration

```bash
# Check if executables are registered
ros2 pkg executables my_robot_pkg

# Should output:
# my_robot_pkg sensor_node
# my_robot_pkg SimplePublisher
```

### Step 6: Run

```bash
# Test sensor_node
ros2 run my_robot_pkg sensor_node

# In another terminal (after sourcing):
ros2 run my_robot_pkg SimplePublisher
```

## Example Node Files

### sensor_node.py

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher_ = self.create_publisher(String, '/sensor_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        self.get_logger().info('Sensor Node started!')

    def timer_callback(self):
        msg = String()
        msg.data = f'Sensor Reading: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### SimplePublisher.py

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, '/chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        self.get_logger().info('SimplePublisher started!')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Debugging Commands

```bash
# 1. Check Python module can be imported
python3 -c "import my_robot_pkg.sensor_node; print('OK')"
python3 -c "import my_robot_pkg.SimplePublisher; print('OK')"

# 2. Check installed entry points
pip list | grep my-robot-pkg
pip show my-robot-pkg

# 3. Check if files exist in install directory
ls -la install/my_robot_pkg/lib/my_robot_pkg/

# 4. See actual entry point script
cat install/my_robot_pkg/lib/my_robot_pkg/sensor_node

# 5. List all ROS2 packages
ros2 pkg list | grep my_robot

# 6. List package executables
ros2 pkg executables my_robot_pkg
```

## Still Getting Errors?

### If you see "module 'my_robot_pkg' has no attribute 'sensor_node'":

1. Make sure the file is named `sensor_node.py` (not `SensorNode.py`)
2. Entry point should be: `my_robot_pkg.sensor_node:main`

### If you see "No module named 'my_robot_pkg'":

1. Check `__init__.py` exists in `my_robot_pkg/my_robot_pkg/`
2. Rebuild with `colcon build --packages-select my_robot_pkg`
3. Source with `source install/setup.bash`

### Common Pattern:

```
Package folder:     my_robot_pkg/
Python module:      my_robot_pkg/my_robot_pkg/
File:               my_robot_pkg/my_robot_pkg/sensor_node.py
Entry point:        'sensor_node = my_robot_pkg.sensor_node:main'
                                    ^^^^^^^^^^^^^ ^^^^^^^^^^^
                                    module name   file name (no .py!)
```

## Quick Fix Command Sequence

```bash
# 1. Create __init__.py if missing
touch my_robot_pkg/my_robot_pkg/__init__.py

# 2. Clean everything
rm -rf build install log

# 3. Rebuild
colcon build --packages-select my_robot_pkg

# 4. Source
source install/setup.bash

# 5. Verify
ros2 pkg executables my_robot_pkg

# 6. Run
ros2 run my_robot_pkg sensor_node
```
