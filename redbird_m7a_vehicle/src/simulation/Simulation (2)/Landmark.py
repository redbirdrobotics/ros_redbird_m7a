from Target_Robot import Target_Robot

class Landmark(object):
    def __init__(self, x, y, angle, color, name):
        self._x = x
        self._y = y
        self._angle = angle
        self._line = []
        self._color = color

        self._name = name

    def def_veloc(self, robot):
        pass