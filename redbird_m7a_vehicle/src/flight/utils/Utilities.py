#!/usr/bin/python

# TODO: Add header docstring

class Utilities:
    @staticmethod
    def is_near(coord1, coord2, allowed_range=0.25):
        """Returns true if the supplied cartesian coordinates are within the specified range.

        Args:
            coord1 (tuple): Tuple of first coodinate set in the form (X, Y, Z).
            coord2 (tuple): Tuple of second coordinate set in the form (X, Y, Z).
        """
        return abs(coord1[0] - coord2[0]) < allowed_range and \
        abs(coord1[1] - coord2[1]) < allowed_range and \
        abs(coord1[2] - coord2[2]) < allowed_range
