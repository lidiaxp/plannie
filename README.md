
# Plannie

Plannie is a framework to path planning in Python, simulated environments, and flights in real environments.


## Installation

The project can be installed in any directory on the computer with the following commands: 

```bash
  git clone https://github.com/lidiaxp/plannie.git
  pip install -r requirements.txt
```
    
## Features

- Classical, meta heuristic, and machine learning path planning techniques in 2D and 3D;
- Deep benchmark with path length, time, memory, CPU, battery, fligth time, among others;
- Integration with Python, Gazebo ([MRS Simulator](https://ctu-mrs.github.io/) and [Sphinx](https://developer.parrot.com/docs/sphinx/whatissphinx.html)), and real environmts
- Avoid dynamic obstacles;
- Travelling salesman problem and coverage path planning algorithms;
- Decision maker supports modular localization, mapping, controller, and vision scripts;
- Decision maker suport multi-UAVs.

## Running Tests

The environment is defined in helper/ambiente.py to 2D environments and in 3D/helper/ambiente.py to 3D environmets.

To run 2D tests in Python:

```bash
  python3 unknownEnvironment.py
```

To run 2D tests in simulator and real environments:

```bash
  python3 movimentoROS.py
```

To run 3D tests in Python:

```bash
  cd 3D
  python3 unknownEnvironment3D.py
```

To run 3D tests in simulator and real environments:

```bash
  cd 3D
  python3 movimentoROS3D.py
```
## License

MIT License

Copyright (c) 2021 Lidia Gianne Souza da Rocha

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

  
## Feedback

If you have any feedback, please reach out to us at lidia@estudante.ufscar.br.

  