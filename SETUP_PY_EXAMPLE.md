# Correct setup.py Structure for ROS2 Package

## Example setup.py for `my_robot_pkg` package

```python
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if you have them
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 package for robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # IMPORTANT: Format is 'executable_name = package.module:function'
            # The executable name is what you'll use with 'ros2 run'
            'SimplePublisher = my_robot_pkg.SimplePublisher:main',

            # You can add more executables here:
            # 'another_node = my_robot_pkg.another_module:main',
        ],
    },
)
```

## Example SimplePublisher.py

**Location**: `my_robot_pkg/SimplePublisher.py` (inside the package folder)

```python
#!/usr/bin/env python3
"""
Simple ROS2 Publisher Node
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # Create publisher
        self.publisher_ = self.create_publisher(String, '/chatter', 10)

        # Create timer (1 Hz = once per second)
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

## Directory Structure

Your ROS2 package should look like this:

```
my_robot_pkg/
├── my_robot_pkg/
│   ├── __init__.py          # IMPORTANT: Must exist (can be empty)
│   └── SimplePublisher.py   # Your node file
├── resource/
│   └── my_robot_pkg         # Empty marker file
├── package.xml
└── setup.py
```

## Common Issues

### 1. "No executable found"

**Causes:**
- Missing `__init__.py` in package folder
- Wrong entry_points format in setup.py
- Not running `colcon build` after changes
- Not sourcing setup.bash after build

**Solution:**
```bash
# 1. Make sure __init__.py exists
touch my_robot_pkg/__init__.py

# 2. Rebuild
colcon build --packages-select my_robot_pkg

# 3. Source the workspace
source install/setup.bash

# 4. Try to run
ros2 run my_robot_pkg SimplePublisher
```

### 2. ModuleNotFoundError

**Causes:**
- Package folder name doesn't match package_name in setup.py
- Missing __init__.py

**Solution:**
- Ensure `my_robot_pkg/` folder exists
- Ensure `my_robot_pkg/__init__.py` exists
- Rebuild with `colcon build`

### 3. Entry Points Format

**WRONG:**
```python
'SimplePublisher = SimplePublisher:main'  # ❌ Missing package name
```

**CORRECT:**
```python
'SimplePublisher = my_robot_pkg.SimplePublisher:main'  # ✅ Full path
```

## Steps to Create Working Package

1. **Create package structure:**
```bash
cd ~/your_workspace
mkdir -p my_robot_pkg/my_robot_pkg
mkdir -p my_robot_pkg/resource
touch my_robot_pkg/my_robot_pkg/__init__.py
touch my_robot_pkg/resource/my_robot_pkg
```

2. **Create files:**
- Create `setup.py` (use example above)
- Create `package.xml`
- Create `my_robot_pkg/SimplePublisher.py` (use example above)

3. **Build:**
```bash
colcon build --packages-select my_robot_pkg
```

4. **Source:**
```bash
source install/setup.bash
```

5. **Run:**
```bash
ros2 run my_robot_pkg SimplePublisher
```

6. **Verify it's publishing:**
```bash
# In another terminal
source install/setup.bash
ros2 topic list
ros2 topic echo /chatter
```
