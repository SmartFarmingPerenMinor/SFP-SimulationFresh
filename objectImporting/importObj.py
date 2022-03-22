#!/usr/bin/env python3.8

import jinja2
from os import path


class GazeboWorldGenerator():
    objects = []
    outputText = ""

    def getListOfImportableObjects(self) -> str:
        return "The object that can be imported are:\ntree\nToo import objects type \"done\"\nGive action: "

    def addTree(self, objName: str, xPos: float, yPos: float, zPos: float, solid=False):
        treePth = path.abspath("../catkin_ws/src/test/models/peren.dae")
        z = zPos -0.7
        tree = {"type": "tree", "name": objName, "x": xPos,
                "y": yPos, "z": z, "solid": solid, "mesh": treePth}
        #res = tree["z"]
        #print(f"z: {z}, tree[\"z\"]: {res}")
        self.objects.append(tree)

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
            x = float(input("give x position: "))
            y = float(input("give y position: "))
            z = float(input("give z position: "))
            solid = bool(input("is the object solid?\nGive \"True\" if obj is solid\nGive \"False\" if obj is not solid\nThe object is: "))
            worldgen.addTree(name, x, y, z, solid)


if __name__ == '__main__':
    main()
