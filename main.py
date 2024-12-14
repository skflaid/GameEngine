# main.py
import pygame
from resources.rendering import Renderer
from resources.mesh import Cube
import numpy as np
from resources.vector import Vector3D
from resources.collision import check_wall_collisions, check_collision, cube_collision, cubes_contact_points
import sys

class Engine:
    def __init__(self):
        self.renderer = Renderer()
        self.renderer.initialize()
        self.cubes = []
        self.create_cubes()
        self.clock = pygame.time.Clock()
        self.dragging = False
        self.drag_start = None
        self.drag_end = None
        self.selected_cube = None
    
    def create_cubes(self):
        """초기 큐브 생성"""
        center = (0,0,0)
        center2 = (3,1,0)
        cube = Cube(center=center, size=3.0, mass=3.0, bounce=0.9)
        self.cubes.append(cube)
        cube = Cube(center=center2, size=3.0, mass=3.0, bounce=0.9)
        self.cubes.append(cube)
    
    def run(self):
        """엔진 루프"""
        while True:
            dt = self.clock.tick(60) / 1000.0  # 초 단위로 변환
            self.handle_events()
            self.update_physics(dt)
            self.renderer.clear()
            self.renderer.draw(self.cubes)
            self.renderer.update_display()
    
    def handle_events(self):
        """이벤트 처리"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # 왼쪽 클릭
                    self.dragging = True
                    self.drag_start = pygame.mouse.get_pos()
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1 and self.dragging:
                    self.dragging = False
                    self.drag_end = pygame.mouse.get_pos()
                    self.throw_cube()
            elif event.type == pygame.MOUSEMOTION:
                if self.dragging:
                    self.drag_end = pygame.mouse.get_pos()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:  # Q 키를 눌렀을 때
                    self.cubes[0].velocity.z += 1.0  # z 방향으로 속도 추가
                elif event.key == pygame.K_e:  # E 키를 눌렀을 때
                    self.cubes[0].velocity.z -= 1.0  # -z 방향으로 속도 추가

    def handle_cube_collisions(self):
        for i in range(len(self.cubes)):
            for j in range(i + 1, len(self.cubes)):
                cube1 = self.cubes[i]
                cube2 = self.cubes[j]
                
                normal, depth = check_collision(cube1, cube2)
                if normal is not None:
                    # 충돌 지점 계산
                    contact_points = cubes_contact_points(cube1, cube2)
                    if contact_points:
                        # contact_points의 평균을 Vector3D로 계산
                        collision_point = Vector3D(
                            sum(cp.x for cp in contact_points) / len(contact_points),
                            sum(cp.y for cp in contact_points) / len(contact_points),
                            sum(cp.z for cp in contact_points) / len(contact_points)
                        )

                        # 충돌 처리
                        cube_collision(cube1, cube2, collision_point, normal)

    def throw_cube(self):
        """큐브를 던지는 동작 처리"""
        if self.drag_start and self.drag_end:
            dx = self.drag_end[0] - self.drag_start[0]
            dy = self.drag_end[1] - self.drag_start[1]
            force = 10.0
            self.cubes[0].velocity = self.cubes[0].velocity + Vector3D(dx * force / 800, -dy * force / 600, 0)
            self.cubes[0].angular_velocity = self.cubes[0].angular_velocity + Vector3D(dy * force / 800, dx * force / 600, 0)
    
    def update_physics(self, dt):
        """물리 업데이트 및 충돌 처리"""
        for cube in self.cubes:
            cube.update(dt)
            check_wall_collisions(cube)

        
        self.handle_cube_collisions()

if __name__ == "__main__":
    engine = Engine()
    engine.run()