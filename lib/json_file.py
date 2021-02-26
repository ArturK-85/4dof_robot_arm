import json
import os

def load(filename, rw='r'):
    """ Load .json file, default read-only """

    try:
        with open(filename, rw) as f:
            load_data = json.load(f)
            return load_data

    except IOError as e:
        print("Sorry! file not found.")
        return False

def write(filename, write_data):
    """ Write data into .json file """

    with open(filename, 'w') as f:
        json.dump(write_data,f)

    print("Your data is writed")
