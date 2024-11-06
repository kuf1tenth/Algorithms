# Algorithms

Add your own branch and push just your algorithms.

**Note:** To get other nodes to work, you need to edit the `setup.py` file under `console_scripts`:

```python
'console_scripts': [
    'cooper = kuf1tenth.cooper:main',
    'brake = kuf1tenth.brake:main',
    'otherNode = kuf1tenth.otherNode:main',
],