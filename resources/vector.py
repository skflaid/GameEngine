#vector.py
import numpy as np

class Vector3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar):
        return Vector3D(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def __neg__(self):
        return Vector3D(-self.x, -self.y, -self.z)
    
    def __itruediv__(self, scalar):
        if scalar != 0:
            self.x /= scalar
            self.y /= scalar
            self.z /= scalar
        return self
    
    def __truediv__(self, scalar):
        if scalar != 0:
            return Vector3D(self.x / scalar, self.y / scalar, self.z / scalar)
        else:
            raise ValueError("Cannot divide by zero")

    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other):
        return Vector3D(self.y * other.z - self.z * other.y,
                        self.z * other.x - self.x * other.z,
                        self.x * other.y - self.y * other.x)

    def magnitude(self):
        return np.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalize(self):
        mag = self.magnitude()
        if mag == 0:
            return Vector3D(0, 0, 0)
        return self * (1.0 / mag)

    def to_array(self):
        return np.array([self.x, self.y, self.z])
    
    def __str__(self):
        # 읽기 쉬운 포맷으로 출력
        return f"Vector3D(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"

    def __repr__(self):
        # 디버깅용 포맷
        return f"Vector3D(x={self.x}, y={self.y}, z={self.z})"
    
    def distance_to(self, other):
        return np.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2)

    @staticmethod
    def from_array(arr):
        return Vector3D(arr[0], arr[1], arr[2])