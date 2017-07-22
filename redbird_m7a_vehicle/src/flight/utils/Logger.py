#!/usr/bin/python

"""Logger.py: A utility that allows for quick logging to a file and terminal."""

from datetime import datetime
from enum import Enum

__author__ = "Alex Bennett"


class Colors(Enum):
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class Logger(object):
    def __init__(self, file_name):
        # Generate file name
        self._file_name = file_name + "_" + str(datetime.now().strftime("%Y%m%d_%H-%M-%S")) + ".txt"

        # Open file with given file name in append mode
        self._file = open(self._file_name, "a+")

        # Write header
        self._file.write("---------- START OF LOG ----------\n")
        self._file.write("Datetime: %s\n\n" % datetime.now())
        self._file.close()

    def close(self):
        self._file = open(self._file_name, "a+")
        self._file.write("----------- END OF LOG -----------\n")
        self._file.close()

    def log_data(self, msg):
        # Open file
        self._file = open(self._file_name, "a+")

        # Write to console
        print Colors.OKBLUE + "[%s][DATA] %s" % (datetime.now().time(), msg) + Colors.ENDC

        # Write to file
        self._file.write("[%s][DATA] %s\n" % (datetime.now().time(), msg))

        # Close file
        self._file.close()

    def log_info(self, msg):
        # Open file
        self._file = open(self._file_name, "a+")

        # Write to console
        print Colors.OKGREEN + "[%s][INFO] %s" % (datetime.now().time(), msg) + Colors.ENDC

        # Write to file
        self._file.write("[%s][INFO] %s\n" % (datetime.now().time(), msg))

        # Close file
        self._file.close()

    def log_warning(self, msg):
        # Open file
        self._file = open(self._file_name, "a+")

        # Write to console
        print Colors.WARNING + "[%s][WARNING] %s" % (datetime.now().time(), msg) + Colors.ENDC

        # Write to file
        self._file.write("[%s][WARNING] %s\n" % (datetime.now().time(), msg))

        # Close file
        self._file.close()

    def log_error(self, msg):
        # Open file
        self._file = open(self._file_name, "a+")

        # Write to console
        print Colors.FAIL + "[%s][ERROR] %s" % (datetime.now().time(), msg) + Colors.ENDC

        # Write to file
        self._file.write("[%s][ERROR] %s\n" % (datetime.now().time(), msg))

        # Close file
        self._file.close()
