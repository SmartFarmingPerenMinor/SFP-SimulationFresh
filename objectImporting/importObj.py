#!/usr/bin/env python3.8

import jinja2
from os import path

class GazeboWorldGenerator():
    objects = []
    outputText = ""

    def getListOfImportableObjects(self) -> str:
        return "The object that can be imported are:\ntree\nrow of tree's\nToo import objects type \"done\"\nGive action: "

    def addTree(self, objName: str, xPos: float, yPos: float, zPos: float, solid=False):
        treePth = path.abspath("../catkin_ws/src/test/models/peren.dae")
        z = zPos -0.7
        tree = {"type": "tree", "name": objName, "x": xPos,
                "y": yPos, "z": z, "solid": solid, "mesh": treePth}

        self.objects.append(tree)

    def addLineOfTrees(self):
        x = float(input("give x for the midle of the line: "))
        y = float(input("give y for the midle of the line: "))
        z = float(input("give z for the midle of the line: "))
        treeCount = int(input("give desired amount of tree's: "))
        RotateXAxis = input("\"true or false\"\ntranslate over the x-Axis: ")
        RotateYAxis = input("\"true or false\"\ntranslate over the y-Axis: ")
        RotateZAxis = input("\"true or false\"\ntranslate over the z-Axis: ")
        return self.addRowOfTrees(x, y, z, RotateXAxis, RotateYAxis, RotateZAxis, treeCount)

    def addRowOfTrees(self, x, y, z, RotateXAxis, RotateYAxis, RotateZAxis, treeCount) -> float:
        center = treeCount/2
        for tree in range(0, treeCount, 1):
            pos = {"x": 0, "y":0, "z":0}

            if (RotateXAxis == "true"):
                pos["x"] = self.__calcOffset(0.7, x, center, tree)
            else:
                pos["x"] = x

            if(RotateYAxis == "true"):
                pos["y"] = self.__calcOffset(0.7, y, center, tree)
            else:
                pos["y"] = y

            if(RotateZAxis == "true"):
                pos["z"] = self.__calcOffset(0.7, z, center, tree)
            else:
                pos["z"] = z

            name = f"tree{tree}"
            self.addTree(name, pos["x"], pos["y"], pos["z"])

    def clear(self):
        self.objects = []
        self.outputText = ""

    def save(self, worldName="myWorld.world"):
        self.__generateWorld()
        fptr = f"../catkin_ws/src/test/worlds/{worldName}"
        f = open(fptr, "w")
        f.write(f"{self.outputText}")
        f.close()
        self.clear

    def __calcOffset(self, offset, axis, centerOfline, index) -> float:
        if (index < centerOfline):
            return (axis - (offset * (centerOfline - index)))
        elif (index == centerOfline):
            return axis
        else:
            return (axis + (offset * (index -centerOfline)))
        return axis

    def __generateWorld(self):
        templateLoader = jinja2.FileSystemLoader("templates")
        templateEnv = jinja2.Environment(loader=templateLoader)
        world = templateEnv.get_template("world")

        self.outputText = world.render(objects=self.__generateObjList())
        return self.outputText

    def __generateTree(self, objName: str, xPos: float, yPos: float, zPos: float, treePth: str, solid: bool):
        templateLoader = jinja2.FileSystemLoader("templates")
        templateEnv = jinja2.Environment(loader=templateLoader)
        tree = templateEnv.get_template("tree")

        if (solid):
            return tree.render(name=objName, x=xPos, y=yPos, z=zPos, bCommand="", mesh=treePth, eCommand="")
        else:
            return tree.render(name=objName, x=xPos, y=yPos, z=zPos, bCommand="<!--", mesh=treePth, eCommand="-->")

    def __generateObjList(self):
        objList = []
        for i in range(0, len(self.objects)):
            obj = self.objects[i]
            if (obj["type"] == "tree"):
                objList.append(self.__generateTree(
                    obj["name"], obj["x"], obj["y"], obj["z"], obj["mesh"], obj["solid"]))

        return objList


def main():
    worldgen = GazeboWorldGenerator()
    running = True
    while (running):
        print(worldgen.getListOfImportableObjects())
        option = input()
        if (option == "done"):
            worldgen.save()
            break
        elif (option == "tree"):
            name = input("give tree name: ")
            x = float(input("give x position, bv 0.7: "))
            y = float(input("give y position, bv 0.7: "))
            z = float(input("give z position, bv 0.0: "))
            solid = bool(input("is the object solid?\nGive \"True\" if obj is solid\nGive \"False\" if obj is not solid\nThe object is: "))
            worldgen.addTree(name, x, y, z, solid)
        elif (option == "row of tree's"):
            worldgen.addLineOfTrees()


if __name__ == '__main__':
    main()
