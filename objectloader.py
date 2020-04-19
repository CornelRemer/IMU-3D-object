"""Objectloader
Load vertices and faces from a Wavefront object and generate edges from faces.

Required *.obj format:
vertices:
    v 1.000000 -1.000000 -1.000000
faces:
    f 2//1 3//1 4//1
"""

import re

class Objectloader:
    """Converts a Blender *.obj file into vertices, edges and faces.

    The __init__ takes one argument:
        filename (str): Name of the *.obj file.
        File must be located in the scripts directory
    
    Attributes:
        filename (str): filename of the Wavefron object
        vertices (list): List with all vertices
        faces (list):   List with tuples of vertices
        edges (list):   List with tuples of two vertices

    """

    def __init__(self, filename):
        self.filename = filename
        self.vertices = []
        self.edges = []
        self.faces = []

    def read_object(self, filename):
        """Read .obj file and create generator"""
        with open(filename, 'r') as f:
            for line in f:
                yield line

    def convert_faces_to_edges(self, face):
        """Generate edges from one object face"""
        for i in range(1, len(face)):
            self.edges.append((face[i-1], face[i]))

        self.edges.append((face[-1], face[0]))

    def remove_edge_dublicates(self):
        """Removes edges which appears twice i.e. [(0,2), (2,0)]"""
        _sorted_edges = []
        for edge in self.edges:
            _sorted_edges.append(tuple(sorted(edge))) # sort all vertices from one edge

        return tuple({*_sorted_edges})

    def get_vertices(self):
        """Read vertices from .obj file"""
        for line in self.read_object(self.filename):
            if line.startswith('v '):
                _vertex_coordinates = re.findall(r"[-+]?\d*\.\d+|\d+", line)  # -12.4 , 12.4, 12,432, 2
                _vertex = tuple(map(lambda coord: float(coord), _vertex_coordinates))
                self.vertices.append(_vertex)
        return tuple(self.vertices)

    def get_faces(self):
        """Read faces from .obj file"""
        for line in self.read_object(self.filename):
            if line.startswith('f '):
                _face_vertices = re.findall(r"\d*//", line)
                _vertices_lst = []
                for vertex in _face_vertices:
                    _vertices_lst.append(int(vertex[:-2])-1)
                self.faces.append(tuple(_vertices_lst))
        return tuple(self.faces)

    def get_edges(self):
        """Creates edges from every face in faces and removes double edges"""
        if len(self.faces) == 0:
            raise Exception("self.get_edges() \nFuction needs faces to generate edges. Use 'self.get_faces() first.'")
        for face in self.faces:
            self.convert_faces_to_edges(face)

        self.edges = self.remove_edge_dublicates()
        return tuple(self.edges)
    
    def get_object_structur(self):
        """Collect vertices, faces and edges from object"""
        self.vertices = self.get_vertices()
        self.faces = self.get_faces()
        self.edges = self.get_edges()
    
    def print_object(self):
        print(f'vertices = {self.vertices}')
        print('\n')
        print(f'faces = {self.faces}')
        print('\n')
        print(f'edges = {self.edges}')
    
if __name__ == '__main__':
    filename = 'cube.obj'
    # filename = 'Cylone.obj'
    # filename = 'monkey.obj'

    myobject = Objectloader(filename)

    # vertices = myobject.get_vertices()
    # faces = myobject.get_faces()
    # edges = myobject.get_edges()

    myobject.get_object_structur()

    # print(f'vertives = {vertices}')
    # print(f'faces = {faces}')
    # print(f'edges = {edges}')
    myobject.print_object()
