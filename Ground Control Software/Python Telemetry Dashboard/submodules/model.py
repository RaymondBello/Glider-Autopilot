#!/usr/bin/env python
# -*- coding: utf-8 -*-



import numpy as np

class Model:
	"""
 Load a 3D model from a .obj file and generates a matrix with the information of the mesh.
 From the file, import information from faces and vertices:
 v 'x' 'and' 'z'
 F 'V1' 'V2' 'V3'
	"""
 
	triangles = []
	vertices= []
	tam = 0
	def __init__(self,filepath):
		self.loadObj(filepath)
		self.setVerts()
		self.returnMesh()
		
	
	def loadObj(self,filepath):
		'Load the .OBJ file and organizes vertices indexed by faces'
		modelFile = open(filepath,"r")
		triangles = []
		vertices = []
		for line in modelFile.readlines():
			line = line.strip()
			if len(line)==0 or line.startswith("#"):
				continue
			data = line.split(" ")
			if data[0]=="v":
				vertices.append((float(data[1]),float(data[2]),float(data[3])))
			if data[0]=="f":
				vertex1 = vertices[int(data[1].split("/")[0])-1]
				vertex2 = vertices[int(data[2].split("/")[0])-1]
				vertex3 = vertices[int(data[3].split("/")[0])-1]
				triangles.append((vertex1,vertex2,vertex3))
				self.tam+=1
		self.triangles = triangles
		self.verts=np.empty((self.tam, 3, 3), dtype=np.float32)
		'Generates a matrix of (TAM, 3, 3), where TAM is the number of faces.Each face is composed of 3 vertices'
#	
	def setVerts(self):
		'Fill the vertice matrix for each triangular face'
		i = 0
		for triangle in self.triangles:
			self.verts[i,0,:]=(triangle[0][0],triangle[0][1],triangle[0][2])
			self.verts[i,1,:]=(triangle[1][0],triangle[1][1],triangle[1][2])
			self.verts[i,2,:]=(triangle[2][0],triangle[2][1],triangle[2][2])
			i+=1
	
	def returnMesh(self):
		'Returns the vertice matrix so that it can be used as meshdata'
		return self.verts

