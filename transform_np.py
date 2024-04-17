# Author: LI SHIDI
# Date of first initialization: July 20, 2018
# Date of last edition: Jan 28, 2022
# This module can process robot pose transformation

import copy
import numpy as np


def _cross(a, b):
    assert len(a) == 3
    assert len(b) == 3
    c = [a[1] * b[2] - a[2] * b[1],
         a[2] * b[0] - a[0] * b[2],
         a[0] * b[1] - a[1] * b[0]]
    return c


def _list_mat_mul(mat1, mat2):
    m = len(mat1)
    n = len(mat1[0])
    s = len(mat2)
    t = len(mat2[0])
    assert n == s
    output = []
    for i in range(m):
        row = []
        for j in range(t):
            acc = 0
            for k in range(n):
                acc += mat1[i][k] * mat2[k][j]
            row.append(acc)
        output.append(row)
    return output


def _dot(vec1, vec2):
    assert len(vec1) == len(vec2)
    output = 0
    for i in range(len(vec1)):
        output += vec1[i] * vec2[i]
    return output


MATRIX = 0
EULER = 1
QUAT = 2


class Transform:

    def __init__(self, translation=None, rotation=None):

        # default value
        if rotation is None:
            rotation = [0, 0, 0, 1]
        if translation is None:
            translation = [0, 0, 0]

        self.__translation = None
        self.__ypr = None
        self.__matrix3x3 = None
        self.__quaternion = None
        self._style = [False, False, False]

        self.set_translation(translation)

        # forward-up
        if len(rotation) == 2:
            self.set_forward_up(rotation[0], rotation[1])

        # 3x3 matrix or ypr
        elif len(rotation) == 3:
            try:
                len(rotation[0])
            except TypeError:
                self.set_ypr(rotation)
            else:
                self.set_matrix3x3(rotation)

        # quaternion
        elif len(rotation) == 4:
            self.set_quaternion(rotation)

        else:
            raise ValueError('Invalid rotation input!')

    def __ypr_to_matrix3x3(self):
        yaw = self.__ypr[0]
        pitch = self.__ypr[1]
        roll = self.__ypr[2]
        cg = np.cos(roll)
        sg = np.sin(roll)
        cb = np.cos(pitch)
        sb = np.sin(pitch)
        ca = np.cos(yaw)
        sa = np.sin(yaw)
        cc = cg * ca
        cs = cg * sa
        sc = sg * ca
        ss = sg * sa

        self.__matrix3x3 = [[ca * cb, sb * sc - cs, sb * cc + ss],
                            [sa * cb, sb * ss + cc, sb * cs - sc],
                            [-sb, cb * sg, cb * cg]]

    def __quaternion_to_matrix3x3(self):
        x = self.__quaternion[0]
        y = self.__quaternion[1]
        z = self.__quaternion[2]
        w = self.__quaternion[3]

        xs = x * 2.
        ys = y * 2.
        zs = z * 2.
        wx = w * xs
        wy = w * ys
        wz = w * zs
        xx = x * xs
        xy = x * ys
        xz = x * zs
        yy = y * ys
        yz = y * zs
        zz = z * zs

        self.__matrix3x3 = [[1 - yy - zz, xy - wz, xz + wy],
                            [xy + wz, 1 - xx - zz, yz - wx],
                            [xz - wy, yz + wx, 1 - xx - yy]]

    def __matrix3x3_to_ypr(self, is_pitch_acute):
        r = self.__matrix3x3
        self.__ypr = [0, 0, 0]  # alpha (yaw), beta (pitch), gamma (roll)

        # if gimbal lock
        if np.abs(r[2][0]) >= 1:
            self.__ypr[0] = 0
            # gimbal locked down
            if r[2][0] < 0:
                self.__ypr[2] = np.arctan2(r[0][1], r[0][2])
                self.__ypr[1] = np.pi / 2
            # gimbal locked up
            else:
                self.__ypr[2] = np.arctan2(-r[0][1], -r[0][2])
                self.__ypr[1] = -np.pi / 2
        # not gimbal lock
        else:
            self.__ypr[1] = np.arcsin(-r[2][0])
            if is_pitch_acute:
                self.__ypr[2] = np.arctan2(r[2][1] / np.cos(self.__ypr[1]), r[2][2] / np.cos(self.__ypr[1]))
                self.__ypr[0] = np.arctan2(r[1][0] / np.cos(self.__ypr[1]), r[0][0] / np.cos(self.__ypr[1]))
            else:
                if self.__ypr[1] >= 0:
                    self.__ypr[1] = np.pi - self.__ypr[1]
                else:
                    self.__ypr[1] = -np.pi - self.__ypr[1]
                self.__ypr[2] = np.arctan2(r[2][1] / np.cos(self.__ypr[1]), r[2][2] / np.cos(self.__ypr[1]))
                self.__ypr[0] = np.arctan2(r[1][0] / np.cos(self.__ypr[1]), r[0][0] / np.cos(self.__ypr[1]))

    def __matrix3x3_to_quaternion(self):
        r = self.__matrix3x3
        trace = r[0][0] + r[1][1] + r[2][2]
        output = [0, 0, 0, 0]

        # 1 + trace = 4 * w ** 2
        if 1 + trace > 0.1:
            # error will be huge if |w| is too small
            s = np.sqrt(1 + trace)
            # consider w > 0 to minimize rotation angle
            output[3] = s * 0.5
            if output[3] > 1.:
                assert output[3] - 1. < 1e-8
                assert (r[2][1] - r[1][2]) ** 2 < 1e-8
                assert (r[0][2] - r[2][0]) ** 2 < 1e-8
                assert (r[1][0] - r[0][1]) ** 2 < 1e-8
                output[3] = 1.
                s = output[3] * 2.
            s = 0.5 / s
            output[0] = (r[2][1] - r[1][2]) * s
            output[1] = (r[0][2] - r[2][0]) * s
            output[2] = (r[1][0] - r[0][1]) * s
        else:
            i = (2 if r[1][1] < r[2][2] else 1) if r[0][0] < r[1][1] else (2 if r[0][0] < r[2][2] else 0)
            j = (i + 1) % 3
            k = (j + 1) % 3
            s = np.sqrt(r[i][i] - r[j][j] - r[k][k] + 1)
            output[i] = s * 0.5
            s = 0.5 / s
            output[3] = (r[k][j] - r[j][k]) * s
            output[j] = (r[j][i] + r[i][j]) * s
            output[k] = (r[k][i] + r[i][k]) * s

        self.__quaternion = output
        self.__check_quaternion_validity()

    def __check_quaternion_validity(self):
        norm = 0
        for i in self.__quaternion:
            norm += i ** 2
        if norm == 0:
            raise ZeroDivisionError('Invalid quaternion!')
        norm = norm ** 0.5
        if norm > 1.001 or norm < 0.999:
            print('Warning: the norm of quaternion is', norm, '!')
            self.__quaternion = [i / norm for i in self.__quaternion]
        if self.__quaternion[-1] < 0.:
            self.__quaternion = [-i for i in self.__quaternion]

    def set_translation(self, tran):
        try:
            assert len(tran) == 3
        except AssertionError:
            raise ValueError('Invalid translation input!')
        finally:
            if type(tran).__name__ == 'numpy.ndarray' or type(tran).__name__ == 'ndarray':
                self.__translation = tran.tolist()
            elif type(tran).__name__ == 'list':
                self.__translation = tran.copy()
            else:
                raise TypeError('Input type must be numpy.ndarray or list!')

    def set_matrix3x3(self, rot):
        try:
            assert len(rot) == 3
            for ele in rot:
                assert len(ele) == 3
            # todo: 函数set_matrix3x3并没有自由度冗余检查，不建议使用，建议使用set_forward_up代替
        except AssertionError:
            raise ValueError('Invalid rotation input!')
        finally:
            if type(rot).__name__ == 'numpy.ndarray' or type(rot).__name__ == 'ndarray':
                self.__matrix3x3 = rot.tolist()
                self._style[MATRIX] = True
                self._style[EULER] = False
                self._style[QUAT] = False
            elif type(rot).__name__ == 'list':
                self.__matrix3x3 = copy.deepcopy(rot)
                self._style[MATRIX] = True
                self._style[EULER] = False
                self._style[QUAT] = False
            else:
                raise TypeError('Input type must be numpy.ndarray or list!')

    def set_forward_up(self, fwd, up):
        try:
            assert len(fwd) == 3
            assert len(up) == 3
            len_fwd = (fwd[0] ** 2 + fwd[1] ** 2 + fwd[2] ** 2) ** 0.5
            len_up = (up[0] ** 2 + up[1] ** 2 + up[2] ** 2) ** 0.5
            assert len_fwd > 1e-2
            assert len_up > 1e-2
            fwd = [_ / len_fwd for _ in fwd]
            up = [_ / len_up for _ in up]
        except TypeError:
            raise ValueError('Invalid rotation input!')
        except AssertionError:
            raise ValueError('Invalid rotation input!')
        finally:
            left = _cross(up, fwd)
            up = _cross(fwd, left)
            temp = Transform(rotation=[fwd, left, up]).__transpose().get_quaternion()
            self.set_quaternion(temp)

    def set_ypr(self, rot):
        try:
            assert len(rot) == 3
        except AssertionError:
            raise ValueError('Invalid rotation input!')
        finally:
            if type(rot).__name__ == 'numpy.ndarray' or type(rot).__name__ == 'ndarray':
                self.__ypr = rot.tolist()
                self._style[MATRIX] = False
                self._style[EULER] = True
                self._style[QUAT] = False
            elif type(rot).__name__ == 'list':
                self.__ypr = rot.copy()
                self._style[MATRIX] = False
                self._style[EULER] = True
                self._style[QUAT] = False
            else:
                raise TypeError('Input type must be numpy.ndarray or list!')

    def set_quaternion(self, rot):
        try:
            assert len(rot) == 4
        except AssertionError:
            raise ValueError('Invalid rotation input!')
        finally:
            if type(rot).__name__ == 'numpy.ndarray' or type(rot).__name__ == 'ndarray':
                self.__quaternion = rot.tolist()
                self._style[MATRIX] = False
                self._style[EULER] = False
                self._style[QUAT] = True
            elif type(rot).__name__ == 'list':
                self.__quaternion = rot.copy()
                self._style[MATRIX] = False
                self._style[EULER] = False
                self._style[QUAT] = True
            else:
                raise TypeError('Input type must be numpy.ndarray or list!')
            self.__check_quaternion_validity()

    def get_forward(self):
        return self.get_forward_up()[0]

    def get_up(self):
        return self.get_forward_up()[1]

    def get_forward_up(self):
        temp = self.__transpose().get_matrix3x3()
        return temp[0], temp[2]

    def get_left(self):
        return self.__transpose().get_matrix3x3()[1]

    def get_translation(self):
        return self.__translation.copy()

    def get_matrix3x3(self):
        if self._style[MATRIX]:
            return copy.deepcopy(self.__matrix3x3)
        elif self._style[EULER]:
            self.__ypr_to_matrix3x3()
            self._style[MATRIX] = True
            return copy.deepcopy(self.__matrix3x3)
        elif self._style[QUAT]:
            self.__quaternion_to_matrix3x3()
            self._style[MATRIX] = True
            return copy.deepcopy(self.__matrix3x3)

    def get_ypr(self, is_pitch_acute=True):
        if self._style[EULER]:
            return self.__ypr.copy()
        elif self._style[MATRIX]:
            self.__matrix3x3_to_ypr(is_pitch_acute)
            self._style[EULER] = True
            return self.__ypr.copy()
        elif self._style[QUAT]:
            self.__quaternion_to_matrix3x3()
            self._style[MATRIX] = True
            self.__matrix3x3_to_ypr(is_pitch_acute)
            self._style[EULER] = True
            return self.__ypr.copy()

    def get_quaternion(self):
        if self._style[QUAT]:
            return self.__quaternion.copy()
        elif self._style[MATRIX]:
            self.__matrix3x3_to_quaternion()
            self._style[QUAT] = True
            return self.__quaternion.copy()
        elif self._style[EULER]:
            self.__ypr_to_matrix3x3()
            self._style[MATRIX] = True
            self.__matrix3x3_to_quaternion()
            self._style[QUAT] = True
            return self.__quaternion.copy()

    def get_matrix4x4(self):
        # create an empty 4x4 matrix
        trans = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
        # put in the translation
        for i in range(3):
            trans[i][3] = self.__translation[i]
        trans[3][3] = 1

        # put in the rotation matrix
        mat = self.get_matrix3x3()
        for i in range(3):
            for j in range(3):
                trans[i][j] = mat[i][j]

        return trans

    def get_axis_angle(self):
        quat = self.get_quaternion()
        w = quat[-1]
        if w == 1.:
            return [0, 0, 1], 0.
        angle = 2 * np.arccos(w)
        sin_half_angle = np.sin(angle / 2.)
        nx = quat[0] / sin_half_angle
        ny = quat[1] / sin_half_angle
        nz = quat[2] / sin_half_angle
        return [nx, ny, nz], angle

    def __mul__(self, other):
        # get 3x3 matrix and multiply
        mat = self.get_matrix3x3()
        new_rotation = _list_mat_mul(mat, other.get_matrix3x3())
        # calculate the translation
        new_translation = [0, 0, 0]
        for i in range(3):
            new_translation[i] = self.__translation[i] + _dot(mat[i], other.__translation)

        return Transform(translation=new_translation, rotation=new_rotation)

    def inverse(self):
        # inverse rotation
        new_quaternion = [0, 0, 0, 0]
        quaternion = self.get_quaternion()
        for i in range(3):
            new_quaternion[i] = -quaternion[i]
        new_quaternion[3] = quaternion[3]

        # inverse translation
        output = Transform([0, 0, 0], new_quaternion)
        new_translation = [0, 0, 0]
        new_mat = output.get_matrix3x3()
        for i in range(3):
            new_translation[i] = _dot(new_mat[i], self.__translation)
            new_translation[i] = -new_translation[i]

        output.set_translation(new_translation)
        return output

    def __transpose(self):
        return Transform(rotation=self.inverse().get_quaternion())

    def __str__(self):
        context = 'The translation is %s' % self.__translation + '\n'

        quaternion = self.get_quaternion()
        context += 'The quaternion is %s' % quaternion

        return context

    def trans_position_local_to_global_3d(self, pos):
        other = Transform(translation=pos)
        return self.__mul__(other).get_translation()

    def trans_position_local_to_global_2d(self, pos):
        pos3d = [pos[0], pos[1], 0]
        return self.trans_position_local_to_global_3d(pos3d)[0:2]

    def trans_position_global_to_local_3d(self, pos):
        other = Transform(translation=pos)
        return (self.inverse() * other).get_translation()

    def trans_position_global_to_local_2d(self, pos):
        pos3d = [pos[0], pos[1], 0]
        return self.trans_position_global_to_local_3d(pos3d)[0:2]

    def rot_vector_local_to_global_3d(self, vector):
        rotator = Transform(rotation=self.get_quaternion())
        other = Transform(translation=vector)
        return (rotator * other).get_translation()

    def rot_vector_local_to_global_2d(self, vector):
        vector3d = [vector[0], vector[1], 0]
        return self.rot_vector_local_to_global_3d(vector3d)[0:2]

    def rot_vector_global_to_local_3d(self, vector):
        rotator = Transform(rotation=self.get_quaternion())
        other = Transform(translation=vector)
        return (rotator.inverse() * other).get_translation()

    def rot_vector_global_to_local_2d(self, vector):
        vector3d = [vector[0], vector[1], 0]
        return self.rot_vector_global_to_local_3d(vector3d)[0:2]
