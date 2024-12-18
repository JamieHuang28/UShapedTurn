from easydict import EasyDict as edict

def Scene1():
    obj = edict()
    obj.xs = [-2.0, -2.0, -2.0, -2.0, -2.0]
    obj.ys = [-4.0, -2.0, 0.0, 2.0, 4.0]
    return obj

def Scene2():
    obj = edict()
    obj.xs = [-1.0, -1.0, -1.0, -1.0, -1.0]
    obj.ys = [-4.0, -2.0, 0.0, 2.0, 4.0]
    return obj

def Scene3():
    obj = edict()
    obj.xs = [-2.0, -2.0, -2.0, -2.0, -2.0, 0.0, 0.0]
    obj.ys = [-4.0, -2.0, 0.0, 2.0, 4.0, 3.0, 4.0]
    return obj

def getObjects(scene_name: str):
    scenes = {}
    scenes['Scene1'] = Scene1()
    scenes['scene2'] = Scene2()
    scenes['scene3'] = Scene3()
    if scene_name not in scenes:
        return {}
    return scenes[scene_name]

if __name__ == "__main__":
    import matplotlib
    # matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    import numpy as np
    
    objects = getObjects('Scene1')
    fig, ax = plt.subplots()
    ax.scatter(objects.xs, objects.ys)
    plt.axis('equal')
    # plt.xlim(-5, 5)
    # plt.ylim(-5, 5)
    plt.savefig('scene.png')
    
    # plt.show()