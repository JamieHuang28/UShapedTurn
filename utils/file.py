from easydict import EasyDict

import json
def parseInput(file_path):
    uturn_input = EasyDict()
    with open(file_path, "r") as f:
        uturn_json = json.load(f)
        uturn_input = EasyDict(uturn_json)
    
    return uturn_input