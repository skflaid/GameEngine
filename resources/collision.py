import numpy as np
from resources.vector import Vector3D
from resources.mesh import Cube
from OpenGL.GL import *
from OpenGL.GLU import *

def collision_with_rotation(cube, collision_point, normal):
    # 상대 속도 계산 
    relative_velocity = cube.velocity


    # 상대 속도의 법선 방향 성분
    velocity_along_normal = relative_velocity.dot(normal)

    # 충돌이 진행 중이 아니라면(분리 중) 아무것도 하지 않음
    if velocity_along_normal > 0:
        return

    # 반발 계수 (e)
    e = cube.bounce

    # 질량 역수
    mass_inverse = 1 / cube.mass if cube.mass != 0 else 0

    # 충돌 지점에서 중심까지의 벡터 
    r = collision_point - cube.center

    # 충격량 크기 계산 
    impulse_magnitude = -(1 + e) * velocity_along_normal / mass_inverse

    # 충격량 벡터 
    impulse = normal * impulse_magnitude

    # 선속도 갱신 
    cube.velocity += impulse * mass_inverse

    # 각속도 갱신 
    angular_impulse = r.cross(impulse)
    angular_change = angular_impulse * (1 / cube.inertia)
    cube.angular_velocity += angular_change

def check_wall_collisions(cube):
    obb_vertices = cube.get_obb_vertices()
    collision_points = []  # 충돌 포인트를 담을 리스트
    normals = []  # 충돌 법선 벡터를 담을 리스트


    for vertex in obb_vertices:
        # 벽과 충돌한 점을 계산
        if vertex.x < -5:
            normals.append(Vector3D(1, 0, 0))
            collision_points.append(Vector3D(-5, vertex.y, vertex.z))
        elif vertex.x > 5:
            normals.append(Vector3D(-1, 0, 0))
            collision_points.append(Vector3D(5, vertex.y, vertex.z))
        elif vertex.y < -5:
            normals.append(Vector3D(0, 1, 0))
            collision_points.append(Vector3D(vertex.x, -5, vertex.z))
        elif vertex.y > 5:
            normals.append(Vector3D(0, -1, 0))
            collision_points.append(Vector3D(vertex.x, 5, vertex.z))
        elif vertex.z < -5:
            normals.append(Vector3D(0, 0, 1))
            collision_points.append(Vector3D(vertex.x, vertex.y, -5))
        elif vertex.z > 5:
            normals.append(Vector3D(0, 0, -1))
            collision_points.append(Vector3D(vertex.x, vertex.y, 5))

    # 충돌 포인트가 없으면 함수 종료
    if not collision_points:
        return

    # 여러 충돌 지점의 평균 위치와 법선 벡터를 계산
    avg_collision_point = Vector3D(0, 0, 0)
    avg_normal = Vector3D(0, 0, 0)
    for i in range(len(collision_points)):
        avg_collision_point += collision_points[i]
        avg_normal += normals[i]
    avg_collision_point /= len(collision_points)
    avg_normal = avg_normal.normalize()

    # 충돌 처리
    collision_with_rotation(cube, avg_collision_point, avg_normal)


def overlap_on_axis(cube1, cube2, axis):
    """주어진 축에 대한 프로젝션이 겹치는지 확인합니다."""
    if np.linalg.norm(axis) < 1e-6:  # 거의 0인 축 건너뛰기
        return True

    axis = axis / np.linalg.norm(axis)  # 축을 정규화
    projection1 = project(cube1, axis)
    projection2 = project(cube2, axis)

    tolerance = 1e-6  # 오차 허용치
    return not (projection1[1] < projection2[0] - tolerance or projection2[1] < projection1[0] - tolerance)


def project(cube, axis):
    """주어진 축에 대해 큐브를 프로젝션하고 [min, max] 범위를 반환합니다."""
    vertices = cube.get_obb_vertices()  # 회전된 꼭짓점을 가져옵니다.
    axis_vector = Vector3D.from_array(axis) if isinstance(axis, np.ndarray) else axis
    projections = [vertex.dot(axis_vector) for vertex in vertices]  # 각 꼭짓점을 축에 투영
    return [min(projections), max(projections)]


def check_collision(cube1, cube2):
    """SAT를 사용하여 충돌을 체크합니다."""
    # 회전된 축들을 계산하여 가져옵니다.
    rotated_axes1 = cube1.get_rotated_axes()
    rotated_axes2 = cube2.get_rotated_axes()
    
    # 각 큐브의 회전 축을 모두 가져오고, 두 큐브의 교차 축을 계산합니다.
    axes = np.vstack((np.array([axis.to_array() for axis in rotated_axes1]),
                      np.array([axis.to_array() for axis in rotated_axes2])))

    # 큐브의 회전 축들에 대한 교차 곱을 구해서 교차 축을 생성
    # 교차 곱을 계산하는 부분 수정
    cross_products = []
    for a in rotated_axes1:
        for b in rotated_axes2:
            cross_product = np.cross(a.to_array(), b.to_array())
            if np.linalg.norm(cross_product) > 1e-6:  # 유효한 축만 추가
                cross_products.append(cross_product)

    # cross_products가 비어있지 않을 경우에만 vstack 수행
    if cross_products:
        axes = np.vstack((axes, cross_products))


    depth = float('inf')
    collision_normal = None

    # 각 축에 대해 투영이 겹치는지 확인
    for axis in axes:
        if not overlap_on_axis(cube1, cube2, axis):
            return None, None  # 충돌하지 않음

        # 두 큐브의 프로젝션을 구하고, 최소 깊이를 계산
        projection1 = project(cube1, axis)
        projection2 = project(cube2, axis)
        axis_depth = min(projection1[1], projection2[1]) - max(projection1[0], projection2[0])

        if axis_depth < depth:
            depth = axis_depth
            if np.linalg.norm(axis) > 1e-6:
                collision_normal = Vector3D.from_array(axis / np.linalg.norm(axis))  # 충돌 법선 계산

    # 최소 침투 깊이가 너무 작으면 충돌로 간주하지 않음
    if depth < 1e-4:
        return None, None

    return collision_normal, depth




def cube_collision(cube1, cube2, collision_point, normal):
    """충돌 해결을 위한 메소드"""
    # 상대 속도 계산
    relative_velocity = cube1.velocity - cube2.velocity

    # 상대 속도의 법선 방향 성분
    velocity_along_normal = relative_velocity.dot(normal)

    # 충돌이 진행 중이 아니라면(분리 중) 아무것도 하지 않음
    if velocity_along_normal > 0:
        return

    # 반발 계수 (e)
    e = min(cube1.bounce, cube2.bounce)

    # 질량 역수
    mass_inverse1 = 1 / cube1.mass if cube1.mass != 0 else 0
    mass_inverse2 = 1 / cube2.mass if cube2.mass != 0 else 0

    # 충격량 크기 계산 
    impulse_magnitude = -(1 + e) * velocity_along_normal / (mass_inverse1 + mass_inverse2)

    # 충격량 벡터 
    impulse = normal * impulse_magnitude

    # 선속도 갱신 
    cube1.velocity += impulse * mass_inverse1
    cube2.velocity -= impulse * mass_inverse2  # 반대 방향으로 적용

    # 각속도 갱신 
    r1 = collision_point - cube1.center
    angular_impulse1 = r1.cross(impulse)
    angular_change1 = angular_impulse1 * (1 / cube1.inertia)
    cube1.angular_velocity += angular_change1

    r2 = collision_point - cube2.center
    angular_impulse2 = r2.cross(-impulse)  # 반대 방향으로 적용
    angular_change2 = angular_impulse2 * (1 / cube2.inertia)
    cube2.angular_velocity += angular_change2

def cubes_contact_points(cube1, cube2):
    """두 큐브 간의 접촉점을 계산합니다."""
    epsilon = 1e-12
    min_distance = float('inf')
    contact_point_1 = None
    contact_point_2 = None

    for vertex1 in cube1.get_obb_vertices():
        for i in range(len(cube2.get_obb_vertices())):
            va = cube2.get_obb_vertices()[i]
            vb = cube2.get_obb_vertices()[(i + 1) % len(cube2.get_obb_vertices())]

            cp, distance = point_to_line_segment_projection(vertex1, va, vb)

            if contact_point_1 is not None and abs(distance - min_distance) < epsilon and not cp.distance_to(contact_point_1) < epsilon:
                contact_point_2 = cp
            elif distance < min_distance:
                min_distance = distance
                contact_point_2 = None
                contact_point_1 = cp

    for vertex2 in cube2.get_obb_vertices():
        for i in range(len(cube1.get_obb_vertices())):
            va = cube1.get_obb_vertices()[i]
            vb = cube1.get_obb_vertices()[(i + 1) % len(cube1.get_obb_vertices())]

            cp, distance = point_to_line_segment_projection(vertex2, va, vb)

            if contact_point_1 is not None and abs(distance - min_distance) < epsilon and not cp.distance_to(contact_point_1) < epsilon:
                contact_point_2 = cp
            elif distance < min_distance:
                min_distance = distance
                contact_point_2 = None
                contact_point_1 = cp

    return [cp for cp in [contact_point_1, contact_point_2] if cp is not None]


def point_to_line_segment_projection(point, va, vb):
    """점과 선분 간의 최단 거리와 접촉점을 계산합니다."""
    ab = vb - va
    ap = point - va
    ab2 = ab.dot(ab)
    ap_ab = ap.dot(ab)
    t = ap_ab / ab2

    # t 값 클램핑
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0

    closest_point = va + ab * t  # Vector3D의 곱셈 사용
    distance = (point - closest_point).magnitude()  # Vector3D의 거리 계산 사용

    return closest_point, distance
