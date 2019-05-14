from adsk.core import Vector3D, Point3D


def to_string(x):
    """
    provides string representations of common types
    """
    if type(x) in (Vector3D, Point3D):
        return "{:.3f},{:.3f},{:.3f}".format(x.x, x.y, x.z)
    return str(x)


def log(file, *msgs):
    line = " ".join([to_string(msg) for msg in msgs])
    line = "{}\n".format(line)

    with open(file, 'a') as f:
        f.write(line)


if __name__ == '__main__':
    import doctest
    doctest.testmod()

    from functools import partial

    import os
    F = os.tmpfile()
    l = partial(log, F)
    l('hello')
    with open(F.name,'r') as f:
        line = f.readline()
        assert line == 'hello\n', line
