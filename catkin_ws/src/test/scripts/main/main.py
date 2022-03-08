#!/usr/bin/python3

import sys

from mover import endEffectorMover
from camera import cameraViewer

def main():
    endEffectorMoverObject = endEffectorMover(sys.argv)

    cameraViewerObject = cameraViewer()

    endEffectorMoverObject.promptLocationAndMove()




if __name__ == "__main__":
    main()