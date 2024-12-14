import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from resources.vector import Vector3D

class Cube:
    def __init__(self, center, size=1.0, mass=1.0, bounce=0.5):
        self.center = Vector3D(*center)  # 글로벌 좌표
        self.size = size
        self.mass = mass
        self.bounce = bounce
        self.velocity = Vector3D(0, 0, 0)  # 글로벌 이동 속도
        self.angular_velocity = Vector3D(0, 0, 0)  # 각속도 (각축별 라디안/초 단위)
        self.rotation_axes = [Vector3D(1, 0, 0), Vector3D(0, 1, 0), Vector3D(0, 0, 1)]  # 기본 X, Y, Z축 회전
        self.rotation_angles = [0.0, 0.0, 0.0]  # 각축별 회전 각도 (라디안 단위)
        self.inertia = (1 / 6) * self.mass * (self.size ** 2)  # 관성 모멘트 계산

    def rotate_vector(self, vector, axis, theta):
        """Rodrigues' Rotation Formula를 사용하여 벡터 회전"""
        axis = axis.normalize()
        k = axis.to_array()
        v = vector.to_array()
        rotated_vector = (
            v * np.cos(theta) +
            np.cross(k, v) * np.sin(theta) +
            k * np.dot(k, v) * (1 - np.cos(theta))
        )
        return Vector3D.from_array(rotated_vector)

    def get_rotated_axes(self):
        """회전된 축을 계산"""
        rotated_axes = []
        for axis, angle in zip(self.rotation_axes, self.rotation_angles):
            # 회전된 축 계산
            rotated_axis = self.rotate_vector(self.center, axis, angle)  # axis와 angle을 전달
            rotated_axes.append(Vector3D(rotated_axis.x, rotated_axis.y, rotated_axis.z))  # Vector3D로 변환

        return rotated_axes


    def get_obb_vertices(self):
        """OBB 정점을 글로벌 좌표로 계산"""
        half_size = self.size / 2
        local_vertices = [
            Vector3D(x, y, z)
            for x in (-half_size, half_size)
            for y in (-half_size, half_size)
            for z in (-half_size, half_size)
        ]

        # 다중 회전축을 순차적으로 적용하여 로컬 정점 회전
        rotated_vertices = local_vertices
        for axis, angle in zip(self.rotation_axes, self.rotation_angles):
            rotated_vertices = [self.rotate_vector(vertex, axis, angle) for vertex in rotated_vertices]

        # 글로벌 좌표로 변환
        return [self.center + vertex for vertex in rotated_vertices]

    def draw(self):
        """OpenGL을 사용하여 큐브 그리기"""
        glPushMatrix()
        glTranslatef(self.center.x, self.center.y, self.center.z)
        glBegin(GL_LINES)
        glColor3f(1.0, 1.0, 1.0)

        # 큐브의 OBB 정점 계산
        obb_vertices = self.get_obb_vertices()

        # 큐브의 에지를 순서대로 그리기
        edges = [
            (0, 1), (1, 3), (3, 2), (2, 0),  # 앞면
            (4, 5), (5, 7), (7, 6), (6, 4),  # 뒷면
            (0, 4), (1, 5), (2, 6), (3, 7)   # 측면 연결
        ]
        for edge in edges:
            start, end = edge
            glVertex3f(obb_vertices[start].x, obb_vertices[start].y, obb_vertices[start].z)
            glVertex3f(obb_vertices[end].x, obb_vertices[end].y, obb_vertices[end].z)

        glEnd()
        glPopMatrix()

    def update(self, dt):
        """글로벌 좌표 내에서 회전 및 이동 처리"""
        # 각속도에 따른 회전 각도 업데이트
        damping_factor = 0.98  # 감쇠율 (1에 가까울수록 천천히 감소)
        for i, axis in enumerate(self.rotation_axes):
            delta_angle = self.angular_velocity.to_array()[i] * dt
            self.rotation_angles[i] += delta_angle
        # 각속도 감소
        self.angular_velocity *= damping_factor

        # 글로벌 속도에 따라 큐브의 위치 업데이트
        self.center.x += self.velocity.x * dt
        self.center.y += self.velocity.y * dt
        self.center.z += self.velocity.z * dt
