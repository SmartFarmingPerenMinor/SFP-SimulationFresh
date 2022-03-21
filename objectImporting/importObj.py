#!/usr/bin/env python3.8

import jinja2
from os import path

class GazeboWorldGenerator():
	objects = []
	outputText = ""

	def addTree(self, objName, solid = False):
		treePth = path.abspath("../catkin_ws/src/test/models/peren.dae")
		templateLoader = jinja2.FileSystemLoader("templates")
		templateEnv = jinja2.Environment(loader=templateLoader)
		tree = templateEnv.get_template("tree")
		
		if (solid):
			self.objects.append(tree.render(name=objName, bCommand="", mesh=treePth, eCommand=""))
		else:
			self.objects.append(tree.render(name=objName, bCommand="<!--", mesh=treePth, eCommand="-->"))

	def clear(self):
		self.objects = []
		self.outputText = ""

	def __generate(self):
		templateLoader = jinja2.FileSystemLoader("templates")
		templateEnv = jinja2.Environment(loader=templateLoader)
		world = templateEnv.get_template("world")
		self.outputText = world.render(objects=self.objects)
		return self.outputText

	def save(self, worldName="myWorld.world"):
		self.__generate()
		fptr = f"../catkin_ws/src/test/worlds/{worldName}"
		f = open(fptr, "w")
		f.write(f"{self.outputText}")
		f.close()
		self.clear

worldgen = GazeboWorldGenerator()
worldgen.addTree("tree1")
worldgen.save()
