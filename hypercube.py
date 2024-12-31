import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import math
import time

class Hypercube:
    def __init__(self, dimensions):
        self.dimensions = dimensions
        self.vertices = self.generate_vertices()
        self.rotation_angles = [0.0] * (dimensions * (dimensions - 1) // 2)

    def generate_vertices(self):
        vertices = []
        def generate(current, dim):
            if dim == 0:
                vertices.append(np.array(current, dtype=float))
                return
            generate(current + [0], dim - 1)
            generate(current + [1], dim - 1)
        generate([], self.dimensions)
        return vertices


    def get_rotation_matrix(self, axis1, axis2, angle):
        dim = self.dimensions
        mat = np.eye(dim, dtype=float)
        c = math.cos(angle)
        s = math.sin(angle)

        mat[axis1, axis1] = c
        mat[axis1, axis2] = -s
        mat[axis2, axis1] = s
        mat[axis2, axis2] = c
        return mat

    def rotate_vertices(self, time, rotation_speeds):
        rotated_vertices = [np.array(v) for v in self.vertices]
        
        k = 0
        for i in range(self.dimensions):
            for j in range(i + 1, self.dimensions):
               self.rotation_angles[k] += rotation_speeds[k] * time
               rot_mat = self.get_rotation_matrix(i, j, self.rotation_angles[k])
               rotated_vertices = [np.dot(rot_mat, v) for v in rotated_vertices]
               k += 1
        return rotated_vertices

    def project_vertices(self, vertices):
        projected = []
        for v in vertices:
            projected.append(v[:3]) # projecting to the 3d space
        return projected

def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, pygame.DOUBLEBUF | pygame.OPENGL)
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -10)

    hypercube_4d = Hypercube(4)
    hypercube_5d = Hypercube(5)

    rotation_speeds_4d = [0.5, 0.4, 0.3, 0.2, 0.4, 0.2]
    rotation_speeds_5d = [0.5, 0.4, 0.3, 0.2, 0.1, 0.4, 0.3, 0.2, 0.1, 0.3]

    t_prev = time.time()
    scale = 1
    point_size = 4

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

            if event.type == pygame.KEYDOWN:
                 if event.key == pygame.K_UP:
                    scale += 0.1
                 if event.key == pygame.K_DOWN:
                     scale -= 0.1

        # Clear
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        # 4D Tesseract
        t_curr = time.time()
        dt = t_curr - t_prev
        t_prev = t_curr
        
        rotated_vertices_4d = hypercube_4d.rotate_vertices(dt, rotation_speeds_4d)
        projected_vertices_4d = hypercube_4d.project_vertices(rotated_vertices_4d)
       
        glPointSize(point_size)
        glBegin(GL_POINTS)
        for v in projected_vertices_4d:
             glColor3f(1.0, 1.0, 1.0)
             glVertex3f(*v * scale)
        glEnd()

       # 5D Hypercube
        rotated_vertices_5d = hypercube_5d.rotate_vertices(dt, rotation_speeds_5d)
        projected_vertices_5d = hypercube_5d.project_vertices(rotated_vertices_5d)
       
        glPointSize(point_size)
        glBegin(GL_POINTS)
        for v in projected_vertices_5d:
             glColor3f(1.0, 1.0, 0.0)
             glVertex3f(*v * scale)
        glEnd()

        pygame.display.flip()
        time.sleep(0.01)  # Add a small delay to reduce CPU usage

if __name__ == '__main__':
    main()