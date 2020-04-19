"""Visualise pitch roll and yaw values of an IMU
with pygame and a 3D Wavefront object.

IMU data format:
    pitch, roll, yaw
    Example:    -1.89,0.18,210.63

Objectloader Wavefront object (*.obj) format:
vertices:
    v 1.000000 -1.000000 -1.000000
faces:
    f 2//1 3//1 4//1
"""

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

import serial
import random

from objectloader import Objectloader

def read_data(ser):
    """Read data from serial connection"""
    ser.flush()
    while ser.inWaiting() == 0:
        pass
    
    values = None
    while values is None:
        values = ser.readline()[:-2].decode().split(',')
        if len(values) == 3:
            try:
                data = tuple(map(lambda x: float(x), values))
                return data
            except ValueError:
                print('ValueError')
                values = None
        else:
            values = None

def Axis():
    """Create colored axis. x (red), y (blue), z (green)"""
    glBegin(GL_LINES)
    # draw line for x axis
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(1.0, 0.0, 0.0)
    # draw line for y axis
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 1.0, 0.0)
    # draw line for Z axis
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, 1.0)
    glEnd()

def MyObject(vertices, faces, edges):
    """Create openGL object made from faces and edges"""
    # faces
    glBegin(GL_TRIANGLES)
    for id,face in enumerate(faces):
        glColor3f(*colors[id])
        for vertex in face:
            glVertex3fv(vertices[vertex])
    glEnd()

    edges
    glBegin(GL_LINES)
    for edge in edges:
        glColor3f(1.0,1.0,1.0)
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

def main(object_):
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, OPENGL|DOUBLEBUF)

    # 3D-Object
    vertices = object_.get_vertices()
    faces = object_.get_faces()
    edges = object_.get_edges()

    # with no IMU
    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -8)
    glRotatef(20, 1.0, 0, 0)
    
    # random colors for each face
    global colors
    colors = [(random.uniform(0, 1),random.uniform(0, 1),random.uniform(0, 1)) for face in faces]

    with serial.Serial('COM4', 115200, timeout=1) as ser:
        while True:
            event = pygame.event.poll()
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                pygame.quit()  #* quit pygame properly
                break
            
            # USW with IMU
            glLoadIdentity()    # replaces the current matrix with the identity matrix (reset to origin)
            gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
            glTranslatef(0.0, 0.0, -8)
            glRotatef(20, 1.0, 0, 0)

            # rotate
            pitch, roll, yaw = read_data(ser)
            
            # pitch, roll, yaw = (5,5,0)

            glRotatef(-1*yaw, 0.0, 1.0, 0.0)
            glRotatef(-1*roll, 1.0, 0.0, 0.0)
            glRotatef(-1*pitch, 0.0, 0.0, 1.0)


            # clear frames
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
            
            MyObject(vertices, faces, edges)
            Axis()

            pygame.display.flip()
            pygame.time.wait(10)

if __name__ == '__main__':

    # Wavefront object
    filename = 'drone.obj'
    # filename = 'cube.obj'
    myObj = Objectloader(filename)

    main(myObj)