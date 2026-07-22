import sys
import os

# Add build directory to python path if libbno055.so/dylib is built there
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "build")))

try:
    import libbno055
    print("Successfully imported libbno055!")
except ImportError as e:
    print(f"ImportError: {e}")
    sys.exit(1)

# Test Vector3 and Quaternion classes
v = libbno055.Vector3(1.0, 2.0, 3.0)
assert v.x == 1.0 and v.y == 2.0 and v.z == 3.0
print(f"Vector3 test passed: {v}")

q = libbno055.Quaternion(1.0, 0.0, 0.0, 0.0)
assert q.w == 1.0 and q.x == 0.0
print(f"Quaternion test passed: {q}")

euler = libbno055.to_euler_degrees(q)
assert euler.x == 0.0 and euler.y == 0.0 and euler.z == 0.0
print(f"to_euler_degrees test passed: {euler}")

# Test BNO055 instantiation and OpMode
imu = libbno055.BNO055(0x28, "mock_device")
assert imu is not None
print("BNO055 Python binding test completed successfully!")
