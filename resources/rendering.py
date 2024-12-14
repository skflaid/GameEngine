import pygame
from OpenGL.GL import *
from OpenGL.GLU import *

class Renderer:
    def __init__(self, width=1600, height=900):
        self.width = width
        self.height = height

    def initialize(self):
        """Pygame과 OpenGL 초기화"""
        pygame.init()
        pygame.display.set_mode((self.width, self.height), pygame.DOUBLEBUF | pygame.OPENGL)
        gluPerspective(45, (self.width / self.height), 0.1, 100.0)  # 원근 투영의 최대 거리 증가
        glTranslatef(0.0, 0.0, -30)  # 카메라를 Z축으로 더 멀리 이동

    def clear(self):
        """화면 초기화"""
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        glEnable(GL_DEPTH_TEST)

    def draw(self, cube_list):
        """모든 큐브 그리기"""
        for cube in cube_list:
            cube.draw()

    def update_display(self):
        """디스플레이 업데이트"""
        pygame.display.flip()
