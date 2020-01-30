from enum import Enum


# ptpMode: int // PTP mode (Value range 0-9)
# x, y, z, r: float
# x,y,z,r are the parameters of ptpMode, which can be set as coordinates, joint angles, or angle increments

class PtPMode(Enum):
    """
    JUMP mode, (x,y,z,r) is the target point in Cartesian coordinate system
    MOVJ mode, (x,y,z,r) is the target point in Cartesian coordinate system
    MOVL mode, (x,y,z,r) is the target point in Cartesian coordinate system
    JUMP mode, (x,y,z,r) is the target point in Jointcoordinate system
    MOVJ mode, (x,y,z,r) is the target point in Joint coordinate system
    MOVL mode, (x,y,z,r) is the target point in Joint coordinate system
    MOVJ mode, (x,y,z,r) is the angle increment in Joint coordinate system
    MOVL mode, (x,y,z,r) is the Cartesian coordinate increment in Joint coordinate system
    MOVJ mode, (x,y,z,r) is the Cartesian coordinate increment in Cartesian coordinate system
    JUMP mode, (x,y,z,r) is the Cartesian coordinate increment in Cartesian coordinate system
    """
    JUMP_XYZ = 0x00
    MOVJ_XYZ = 0x01
    MOVL_XYZ = 0x02
    JUMP_ANGLE = 0x03
    MOVJ_ANGLE = 0x04
    MOVL_ANGLE = 0x05
    MOVJ_INC = 0x06
    MOVL_INC = 0x07
    MOVJ_XYZ_INC = 0x08
    JUMP_MOVL_XYZ = 0x09
