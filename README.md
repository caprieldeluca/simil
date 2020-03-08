# simil

Find similarity transformation parameters, given a set of 3-D control points, using dual quaternions.

----
## Summary

A partial implementation of the algorithm described by Zeng et al., 2018<sup>[1]</sup>.

Given a set of 3-D control points, the algorithm solves an optimization
problem to find the parameters of the similarity transformation
that minimizes the error of the solution, applying the mathematical
concepts of dual numbers and quaternions.

Source and target control points coordinates are passed as arguments to
the `process` function, which returns the values for M (multiplier
factor), R (rotation matrix), and T (translation vector).

Once the parameters have been solved, transform coordinates with the
following formula:
    
```
XYZ_t = M * R @ XYZ_s + T
```   
Where:
- ``XYZ_t`` are the coordinates of the target points.
- ``M`` is the multiplier factor (`lambda_i`).
- ``R`` is the rotation matrix (`r_matrix`).
- ``XYZ_s`` are the coordinates of the source points.
- ``T`` is the translation (column) vector (`t_vector`).

Per point weights can be used.  

The solution can be forced to mirror and/or to fixed scale.  

----

## Notes

Requires `numpy`.

----

## Reference


[1] Huaien Zeng, Xing Fang, Guobin Chang and Ronghua Yang (2018)
A dual quaternion algorithm of the Helmert transformation problem.
Earth, Planets and Space (2018) 70:26.
[https://doi.org/10.1186/s40623-018-0792-x](https://doi.org/10.1186/s40623-018-0792-x)

----  

## License

[MIT License](https://github.com/gabriel-de-luca/simil/raw/master/LICENSE)

Copyright (c) 2020 Gabriel De Luca

----  

## Installation

Save the [simil.py](https://raw.githubusercontent.com/gabriel-de-luca/simil/master/simil.py) file in one directory of the Python interpreter [Module Search Path](https://docs.python.org/3/tutorial/modules.html#the-module-search-path).

----

## Examples

Common usage.

```python
>>> import numpy as np
>>> np.set_printoptions(precision=3, suppress=True)
>>> import simil
>>> source_points = [[0, 0, 0],
...                  [0, 2, 2],
...                  [2, 3, 1],
...                  [3, 1, 2],
...                  [1, 1, 3]]
>>> target_points = [[3.0, 7.0, 5.0],
...                  [6.0, 7.0, 2.0],
...                  [4.5, 4.0, 0.5],
...                  [6.0, 2.5, 3.5],
...                  [7.5, 5.5, 3.5]]
>>> m, r, t = simil.process(source_points, target_points)
>>> m
1.5000000000000016
>>> r
array([[-0.,  0.,  1.],
       [-1., -0., -0.],
       [ 0., -1.,  0.]])
>>> t
array([[3.],
       [7.],
       [5.]])
```

To transform, we need coordinates (instead of points) in the rows,
so transpose:

```python
>>> source_coords = np.array(source_points).T
>>> target_coords = m * r @ source_coords + t
>>> print(target_coords.T)
[[3.  7.  5. ]
 [6.  7.  2. ]
 [4.5 4.  0.5]
 [6.  2.5 3.5]
 [7.5 5.5 3.5]]
```

To force a fixed scale of 1.25:

```python
>>> m, r, t = simil.process(source_points,
...                         target_points, 
...                         scale=False, 
...                         lambda_0=1.25)
>>> m
1.25
>>> print((m * r @ source_coords + t).T)
[[3.4  6.7  4.65]
 [5.9  6.7  2.15]
 [4.65 4.2  0.9 ]
 [5.9  2.95 3.4 ]
 [7.15 5.45 3.4 ]]
```

To force mirroring the source points: 

```python
>>> m, r, t = simil.process(source_points, target_points, lambda_0=-1)
>>> print((m * r @ source_coords + t).T)
[[4.385 6.758 3.124]
 [5.329 4.987 3.951]
 [4.739 3.984 2.475]
 [6.097 4.987 1.648]
 [6.451 5.283 3.301]]
```

Per point weights can be passed as a list:

```python
>>> alpha_0 = [100, 20, 2, 20, 50]
>>> m, r, t = simil.process(source_points,
...                         target_points,
...                         alpha_0=alpha_0,
...                         scale=False,
...                         lambda_0=1)
>>> print((m * r @ source_coords + t).T)
[[3.604 6.703 4.698]
 [5.604 6.703 2.698]
 [4.604 4.703 1.698]
 [5.604 3.703 3.698]
 [6.604 5.703 3.698]]
```

----  

## Process function help

```
>>> help(simil.process)
Help on function process in module simil:

process(source_points, target_points, alpha_0=None, scale=True, lambda_0=1.0)
    Find similarity transformation parameters given a set of control points

    Parameters
    ----------
    source_points : array_like
        The function will try to cast it to a numpy array with shape:
        ``(n, 3)``, where ``n`` is the number of points.
        Two points is the minimum requeriment (in that case, the solution
        will map well all points that belong in the rect that passes
        through both control points).
    target_points : array_like
        The function will try to cast it to a numpy array with shape:
        ``(n, 3)``, where ``n`` is the number of points.
        The function will check that there are as many target points
        as source points.
    alpha_0 : array_like, optional
        Per point weights.
        If provided, the function will try to cast to a numpy array with
        shape: ``(n,)``.
    scale : boolean, optional
        Allow to find a multiplier factor different from lambda_0.
        Default is True.
    lambda_0 : float, optional
        Multiplier factor to find the first solution. Default is 1.0.
        If `scale=True`, a recursion is implemented to find a better
        value. If it is negative, forces mirroring. Can't be zero.

    Returns
    -------
    lambda_i : float
        Multiplier factor.
    r_matrix : numpy.ndarray
        Rotation matrix.
    t_vector : numpy.ndarray
        Translation (column) vector.
```

