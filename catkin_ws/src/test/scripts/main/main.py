#!/usr/bin/python3

import sys

from mover import endEffectorMover
from camera import cameraViewer
from world import worldBuilder
def main():
    endEffectorMoverObject = endEffectorMover(sys.argv)
    worldBuilderObj = worldBuilder(endEffectorMoverObject)
    worldBuilderObj.addPlane("ground_plane")
    cameraViewerObject = cameraViewer()
    endEffectorMoverObject.promptLocationAndMove()


if __name__ == "__main__":
    main()