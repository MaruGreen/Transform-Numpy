# Install
pip install -e .

# Usage
from transform_np import Transform
my_trans = Transform(translation=[0, 0, 0], rotation=[0, 0, 0, 1])

# Notice
该模块可供初学者作学习、理解、快速验证三维坐标变换之用途。

在实际生产中，应在充分理解变换原理后，先进行数学化简，再调用相应的C++或torch.jit接口编写运算脚本，以获得合理的运算效率。
