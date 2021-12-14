# controls-algorithms

# First time generating initial Sphinx docs
Run the following line from the root project directory
```
sphinx-apidoc -F -A "Control Algorithms" -V "0.1" -o docs src
```

In the docs/ directory, delete the src.rst file as it is not needed

## Building the html docs
From the docs/ directory, run:
```
make html
```